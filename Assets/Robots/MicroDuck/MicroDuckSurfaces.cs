using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Mujoco;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.SceneManagement;
#if UNITY_EDITOR
using Unity.Collections;
using UnityEditor;
#endif

internal sealed class MicroDuckSurfaces : IDisposable
{
    const int ARLayer = 7;
    const int SimulationLayer = 30;
    const float SurfaceThickness = 0.02f;
    // Compare shape in meters after scale/shear, independent of tracked rigid pose.
    const float GeometryEpsilonSquared = 0.00000001f; // 0.1 mm
    const float PoseEpsilonSquared = 0.000000000001f; // 1 micrometer
    const float TriangleNormalEpsilonSquared = 0.0000000001f;

    [Flags]
    internal enum Change
    {
        None = 0,
        Pose = 1,
        Geometry = 2,
        Added = 4,
        Removed = 8
    }

    sealed class Surface
    {
        public Collider collider;
        public bool box;
        public Mesh mesh;
        public bool meshCaptured;
        public string bodyName;
        public int mocapId = -1;
        public bool poseCaptured;
        public bool poseDirty;
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 nativeUp;
        public readonly List<Vector3> vertices = new();
        public readonly List<Vector3> sourceVertices = new();
        public readonly List<int> triangles = new();
    }

    readonly Transform anchor;
    readonly List<GameObject> roots = new();
    readonly List<Collider> found = new();
    readonly List<Collider> colliderScratch = new();
    readonly List<Surface> surfaces = new();
    readonly Dictionary<Collider, Surface> surfacesByCollider = new();
    readonly HashSet<Collider> foundSet = new();
    readonly HashSet<int> diagnosed = new();
    readonly List<Vector3> vertexScratch = new();
    readonly List<int> triangleScratch = new();
    readonly List<int> vertexIndexScratch = new();
    bool disposed;
    int nextSurfaceId;

    public MicroDuckSurfaces(Transform anchor)
    {
        this.anchor = anchor != null ? anchor : throw new ArgumentNullException(nameof(anchor));
    }

    public Change Refresh()
    {
        if (disposed)
            throw new ObjectDisposedException(nameof(MicroDuckSurfaces));

        var anchorScene = anchor.gameObject.scene;
        if (!anchorScene.IsValid() || !anchorScene.isLoaded)
        {
            var hadSurfaces = surfaces.Count != 0;
            surfaces.Clear();
            surfacesByCollider.Clear();
            return hadSurfaces ? Change.Removed : Change.None;
        }
        found.Clear();
        CollectScene(anchorScene);
        var anchorPhysics = anchorScene.GetPhysicsScene();
        for (var sceneIndex = 0; sceneIndex < SceneManager.sceneCount; sceneIndex++)
        {
            var scene = SceneManager.GetSceneAt(sceneIndex);
            if (!scene.IsValid() || !scene.isLoaded || scene == anchorScene ||
                scene.GetPhysicsScene() == anchorPhysics)
                continue;
            CollectScene(scene);
        }
        foundSet.Clear();
        for (var i = 0; i < found.Count; i++)
            foundSet.Add(found[i]);

        var changed = Change.None;
        for (var i = surfaces.Count - 1; i >= 0; i--)
        {
            var surface = surfaces[i];
            if (surface.collider == null || !foundSet.Contains(surface.collider))
            {
                surfacesByCollider.Remove(surface.collider);
                surfaces.RemoveAt(i);
                changed |= Change.Removed;
            }
        }
        for (var i = 0; i < found.Count; i++)
        {
            var collider = found[i];
            var layer = collider.gameObject.layer;
            if (IsDuckCollider(collider) || !collider.enabled || !collider.gameObject.activeInHierarchy ||
                collider.isTrigger || (collider.attachedRigidbody != null && !collider.attachedRigidbody.isKinematic) ||
                (layer != ARLayer && layer != SimulationLayer))
            {
                if (RemoveSurface(collider)) changed |= Change.Removed;
                continue;
            }
            var surface = FindSurface(collider);
            var isBox = collider is BoxCollider;
            var meshCollider = collider as MeshCollider;
            if (!isBox && (meshCollider == null || meshCollider.sharedMesh == null))
            {
                Diagnose(collider, "is an unsupported surface shape; only BoxCollider and MeshCollider surfaces are supported.");
                if (RemoveSurface(collider)) changed |= Change.Removed;
                continue;
            }
            if (collider is BoxCollider box && (box.size.x <= 0f || box.size.y <= 0f || box.size.z <= 0f))
            {
                Diagnose(collider, "has a zero or negative BoxCollider dimension and cannot be emitted as a native collision mesh.");
                if (RemoveSurface(collider)) changed |= Change.Removed;
                continue;
            }
            #if !UNITY_EDITOR
            if (meshCollider != null && !meshCollider.sharedMesh.isReadable)
            {
                Diagnose(collider, "uses a non-readable mesh and cannot provide a bounded native surface in a player build.");
                if (RemoveSurface(collider)) changed |= Change.Removed;
                continue;
            }
            #endif
            if (surface == null)
            {
                surface = new Surface {
                    collider = collider, box = isBox,
                    bodyName = "microduck_surface_body_" + (nextSurfaceId++).ToString(CultureInfo.InvariantCulture)
                };
                surfaces.Add(surface);
                surfacesByCollider.Add(collider, surface);
                changed |= Change.Added;
            }
            try
            {
                changed |= Capture(surface);
            }
            catch (Exception exception)
            {
                Diagnose(collider, $"could not be read as a bounded native surface: {exception.Message}");
                if (RemoveSurface(collider)) changed |= Change.Removed;
            }
        }
        return changed;
    }

    bool RemoveSurface(Collider collider)
    {
        if (!surfacesByCollider.TryGetValue(collider, out var surface))
            return false;
        surfacesByCollider.Remove(collider);
        surfaces.Remove(surface);
        return true;
    }

    void CollectScene(Scene scene)
    {
        roots.Clear();
        scene.GetRootGameObjects(roots);
        for (var i = 0; i < roots.Count; i++)
        {
            roots[i].GetComponentsInChildren(false, colliderScratch);
            found.AddRange(colliderScratch);
        }
    }

    bool IsDuckCollider(Collider collider) => collider.transform == anchor || collider.transform.IsChildOf(anchor);

    Surface FindSurface(Collider collider) =>
        surfacesByCollider.TryGetValue(collider, out var surface) ? surface : null;

    void Diagnose(Collider collider, string reason)
    {
        var id = collider.GetEntityId().GetHashCode();
        if (diagnosed.Add(id))
            Debug.LogWarning($"MicroDuck surface '{collider.name}' {reason}", collider);
    }

    Change Capture(Surface surface)
    {
        var collider = surface.collider;
        var colliderToAnchor = anchor.worldToLocalMatrix * collider.transform.localToWorldMatrix;
        var position = (Vector3)colliderToAnchor.GetColumn(3);
        var rotation = Quaternion.Inverse(anchor.rotation) * collider.transform.rotation;
        // q and -q encode the same rigid pose.
        if (surface.poseCaptured && Quaternion.Dot(surface.rotation, rotation) < 0f)
            rotation = new Quaternion(-rotation.x, -rotation.y, -rotation.z, -rotation.w);
        var difference = new Vector4(rotation.x - surface.rotation.x, rotation.y - surface.rotation.y,
            rotation.z - surface.rotation.z, rotation.w - surface.rotation.w);
        var change = Change.None;
        if (!surface.poseCaptured || (position - surface.position).sqrMagnitude > PoseEpsilonSquared ||
            difference.sqrMagnitude > 0.00000000000001f)
        {
            surface.position = position;
            surface.rotation = rotation;
            surface.poseCaptured = true;
            surface.poseDirty = true;
            change |= Change.Pose;
        }

        // Strip only rigid rotation. Retain the full residual linear transform,
        // including nonuniform parent scale, reflections, and shear.
        var shape = Matrix4x4.Rotate(Quaternion.Inverse(rotation)) * colliderToAnchor;
        bool geometryChanged;
        if (surface.box)
        {
            var box = (BoxCollider)collider;
            vertexScratch.Clear();
            var half = box.size * 0.5f;
            for (var i = 0; i < 8; i++)
            {
                var local = box.center + new Vector3((i & 1) == 0 ? -half.x : half.x,
                    (i & 2) == 0 ? -half.y : half.y, (i & 4) == 0 ? -half.z : half.z);
                vertexScratch.Add(shape.MultiplyVector(local));
            }
            geometryChanged = !SameVertices(surface.vertices, vertexScratch);
            if (geometryChanged)
            {
                surface.vertices.Clear();
                surface.vertices.AddRange(vertexScratch);
                surface.triangles.Clear();
                var mirrored = shape.determinant < 0f;
                for (var i = 0; i < BoxTriangles.Length; i += 3)
                {
                    surface.triangles.Add(BoxTriangles[i]);
                    surface.triangles.Add(BoxTriangles[i + (mirrored ? 2 : 1)]);
                    surface.triangles.Add(BoxTriangles[i + (mirrored ? 1 : 2)]);
                }
            }
        }
        else
        {
            var mesh = ((MeshCollider)collider).sharedMesh;
            var topologyChanged = false;
            if (mesh.isReadable)
            {
                vertexScratch.Clear();
                mesh.GetVertices(vertexScratch);
                if (!SameVertices(surface.sourceVertices, vertexScratch, 0f))
                {
                    surface.sourceVertices.Clear();
                    surface.sourceVertices.AddRange(vertexScratch);
                }
                triangleScratch.Clear();
                for (var submesh = 0; submesh < mesh.subMeshCount; submesh++)
                {
                    vertexIndexScratch.Clear();
                    mesh.GetTriangles(vertexIndexScratch, submesh, true);
                    triangleScratch.AddRange(vertexIndexScratch);
                }
                topologyChanged = !SameTriangles(surface.triangles, triangleScratch);
                if (topologyChanged)
                {
                    surface.triangles.Clear();
                    surface.triangles.AddRange(triangleScratch);
                }
                surface.mesh = mesh;
                surface.meshCaptured = true;
            }
            else
            {
                #if UNITY_EDITOR
                if (!surface.meshCaptured || surface.mesh != mesh)
                {
                    CaptureNonReadableMesh(mesh, surface);
                    topologyChanged = true;
                }
                #else
                throw new InvalidOperationException("A bounded player surface must have a readable mesh.");
                #endif
            }
            vertexScratch.Clear();
            for (var i = 0; i < surface.sourceVertices.Count; i++)
                vertexScratch.Add(shape.MultiplyVector(surface.sourceVertices[i]));
            geometryChanged = topologyChanged || !SameVertices(surface.vertices, vertexScratch);
            if (geometryChanged)
            {
                surface.vertices.Clear();
                surface.vertices.AddRange(vertexScratch);
            }
        }
        if (geometryChanged)
        {
            surface.nativeUp = ToNative(Quaternion.Inverse(surface.rotation) * Vector3.up);
            change |= Change.Geometry;
        }
        return change;
    }

    #if UNITY_EDITOR
    static void CaptureNonReadableMesh(Mesh mesh, Surface surface)
    {
        using (var meshDataArray = MeshUtility.AcquireReadOnlyMeshData(mesh))
        {
            var meshData = meshDataArray[0];
            using (var vertices = new NativeArray<Vector3>(meshData.vertexCount, Allocator.Temp))
            {
                meshData.GetVertices(vertices);
                surface.sourceVertices.Clear();
                for (var i = 0; i < vertices.Length; i++)
                    surface.sourceVertices.Add(vertices[i]);
            }

            var indexCount = 0;
            for (var submesh = 0; submesh < meshData.subMeshCount; submesh++)
                indexCount += meshData.GetSubMesh(submesh).indexCount;
            using (var indices = new NativeArray<int>(indexCount, Allocator.Temp))
            {
                surface.triangles.Clear();
                var offset = 0;
                for (var submesh = 0; submesh < meshData.subMeshCount; submesh++)
                {
                    var count = meshData.GetSubMesh(submesh).indexCount;
                    meshData.GetIndices(indices.GetSubArray(offset, count), submesh, true);
                    for (var i = 0; i < count; i++)
                        surface.triangles.Add(indices[offset + i]);
                    offset += count;
                }
            }
        }
        surface.mesh = mesh;
        surface.meshCaptured = true;
    }
    #endif

    // ROS conversion reflects the Unity basis, so reverse the box's Unity face winding.
    static readonly int[] BoxTriangles = { 0, 3, 2, 0, 1, 3, 4, 7, 5, 4, 6, 7, 0, 5, 1, 0, 4, 5, 2, 7, 6, 2, 3, 7, 0, 6, 4, 0, 2, 6, 1, 7, 3, 1, 5, 7 };
    static readonly int[] PrismFaces = { 0, 1, 2, 3, 5, 4, 0, 1, 4, 0, 4, 3, 1, 2, 5, 1, 5, 4, 2, 0, 3, 2, 3, 5 };

    static bool SameVertices(List<Vector3> a, List<Vector3> b, float epsilonSquared = GeometryEpsilonSquared)
    {
        if (a.Count != b.Count) return false;
        for (var i = 0; i < a.Count; i++)
            if ((a[i] - b[i]).sqrMagnitude > epsilonSquared) return false;
        return true;
    }

    static bool SameTriangles(List<int> a, List<int> b)
    {
        if (a.Count != b.Count) return false;
        for (var i = 0; i < a.Count; i++) if (a[i] != b[i]) return false;
        return true;
    }

    public string BuildSceneXml(string robotXml)
    {
        if (disposed)
            throw new ObjectDisposedException(nameof(MicroDuckSurfaces));
        if (robotXml == null)
            throw new ArgumentNullException(nameof(robotXml));
        var assets = new StringBuilder();
        var geoms = new StringBuilder();
        var id = 0;
        foreach (var surface in surfaces)
        {
            if (surface.collider == null)
                continue;
            geoms.Append("    <body name=\"").Append(surface.bodyName).Append("\" mocap=\"true\" pos=\"");
            var position = ToNative(surface.position);
            AppendFloat(geoms, position.x); geoms.Append(' ');
            AppendFloat(geoms, position.y); geoms.Append(' ');
            AppendFloat(geoms, position.z);
            geoms.Append("\" quat=\"");
            var rotation = surface.rotation;
            AppendFloat(geoms, rotation.w); geoms.Append(' ');
            AppendFloat(geoms, -rotation.z); geoms.Append(' ');
            AppendFloat(geoms, rotation.x); geoms.Append(' ');
            AppendFloat(geoms, -rotation.y);
            geoms.Append("\">\n");
            if (surface.box)
                AppendBox(assets, geoms, surface, ref id);
            else
                AppendMeshPrisms(assets, geoms, surface, ref id);
            geoms.Append("    </body>\n");
        }
        var xml = new StringBuilder(robotXml);
        InsertBeforeClosing(xml, "</asset>", assets);
        InsertBeforeClosing(xml, "</worldbody>", geoms);
        return xml.ToString();
    }

    static void AppendBox(StringBuilder assets, StringBuilder geoms, Surface surface, ref int id)
    {
        var center = Vector3.zero;
        for (var i = 0; i < surface.vertices.Count; i++)
            center += ToNative(surface.vertices[i]);
        center /= surface.vertices.Count;
        var name = "microduck_surface_" + id++;
        AppendMeshAsset(assets, name, surface.vertices, surface.triangles, center);
        AppendGeom(geoms, name, center);
    }

    void AppendMeshPrisms(StringBuilder assets, StringBuilder geoms, Surface surface, ref int id)
    {
        var v = surface.vertices;
        var t = surface.triangles;
        for (var i = 0; i + 2 < t.Count; i += 3)
        {
            var ia = t[i]; var ib = t[i + 1]; var ic = t[i + 2];
            if (ia < 0 || ib < 0 || ic < 0 || ia >= v.Count || ib >= v.Count || ic >= v.Count)
                continue;
            var a = ToNative(v[ia]); var b = ToNative(v[ib]); var c = ToNative(v[ic]);
            var normal = Vector3.Cross(b - a, c - a);
            if (normal.sqrMagnitude < TriangleNormalEpsilonSquared) continue;
            if (Vector3.Dot(normal, surface.nativeUp) < 0f)
            {
                (b, c) = (c, b);
                normal = -normal;
            }
            var down = -normal.normalized * SurfaceThickness;
            vertexScratch.Clear();
            vertexScratch.Add(a); vertexScratch.Add(b); vertexScratch.Add(c);
            vertexScratch.Add(a + down); vertexScratch.Add(b + down); vertexScratch.Add(c + down);
            var center = (a + b + c) / 3f + down * 0.5f;
            var name = "microduck_surface_" + id++;
            AppendNativeMeshAsset(assets, name, vertexScratch, PrismFaces, center);
            AppendGeom(geoms, name, center);
        }
    }

    static Vector3 ToNative(Vector3 p) => new(p.z, -p.x, p.y);

    public unsafe void BindNativeModel(MujocoLib.mjModel_* model)
    {
        foreach (var surface in surfaces)
        {
            var bodyId = MujocoLib.mj_name2id(model, (int)MujocoLib.mjtObj.mjOBJ_BODY, surface.bodyName);
            if (bodyId < 0 || model->body_mocapid[bodyId] < 0)
                throw new InvalidOperationException("Missing native mocap surface: " + surface.bodyName);
            surface.mocapId = model->body_mocapid[bodyId];
        }
    }

    public unsafe int ApplyNativePoses(MujocoLib.mjData_* data, bool all)
    {
        var updated = 0;
        foreach (var surface in surfaces)
        {
            if (!all && !surface.poseDirty) continue;
            if (surface.mocapId < 0)
                throw new InvalidOperationException("Bind the native surface model before updating poses.");
            var position = ToNative(surface.position);
            var rotation = surface.rotation;
            var p = surface.mocapId * 3;
            var q = surface.mocapId * 4;
            data->mocap_pos[p] = position.x;
            data->mocap_pos[p + 1] = position.y;
            data->mocap_pos[p + 2] = position.z;
            data->mocap_quat[q] = rotation.w;
            data->mocap_quat[q + 1] = -rotation.z;
            data->mocap_quat[q + 2] = rotation.x;
            data->mocap_quat[q + 3] = -rotation.y;
            surface.poseDirty = false;
            updated++;
        }
        return updated;
    }

    static void AppendMeshAsset(StringBuilder output, string name, IReadOnlyList<Vector3> vertices,
        IReadOnlyList<int> triangles, Vector3 center)
    {
        output.Append("    <mesh name=\"").Append(name).Append("\" vertex=\"");
        for (var i = 0; i < vertices.Count; i++)
        {
            if (i != 0) output.Append(' ');
            var p = ToNative(vertices[i]) - center;
            AppendFloat(output, p.x); output.Append(' '); AppendFloat(output, p.y); output.Append(' '); AppendFloat(output, p.z);
        }
        AppendMeshFaces(output, triangles);
    }

    static void AppendNativeMeshAsset(StringBuilder output, string name, IReadOnlyList<Vector3> vertices,
        IReadOnlyList<int> triangles, Vector3 center)
    {
        output.Append("    <mesh name=\"").Append(name).Append("\" vertex=\"");
        for (var i = 0; i < vertices.Count; i++)
        {
            if (i != 0) output.Append(' ');
            var p = vertices[i] - center;
            AppendFloat(output, p.x); output.Append(' '); AppendFloat(output, p.y); output.Append(' '); AppendFloat(output, p.z);
        }
        AppendMeshFaces(output, triangles);
    }

    static void AppendMeshFaces(StringBuilder output, IReadOnlyList<int> triangles)
    {
        output.Append("\" face=\"");
        for (var i = 0; i < triangles.Count; i++)
        {
            if (i != 0) output.Append(' ');
            output.Append(triangles[i]);
        }
        output.Append("\"/>\n");
    }

    static void AppendGeom(StringBuilder output, string name, Vector3 center)
    {
        output.Append("    <geom name=\"").Append(name).Append("\" type=\"mesh\" mesh=\"").Append(name)
            .Append("\" pos=\"");
        AppendFloat(output, center.x); output.Append(' '); AppendFloat(output, center.y); output.Append(' '); AppendFloat(output, center.z);
        output.Append("\" contype=\"1\" conaffinity=\"1\" density=\"0\" friction=\"1 0.005 0.0001\"/>\n");
    }

    static void AppendFloat(StringBuilder output, float value) => output.Append(value.ToString("R", CultureInfo.InvariantCulture));
    internal void GetNavigationBuildSources(List<NavMeshBuildSource> output, out Bounds bounds)
    {
        if (disposed)
            throw new ObjectDisposedException(nameof(MicroDuckSurfaces));
        output.Clear();
        var hasBounds = false;
        bounds = default;
        for (var i = 0; i < surfaces.Count; i++)
        {
            var surface = surfaces[i];
            var collider = surface.collider;
            if (collider == null || !collider.enabled || !collider.gameObject.activeInHierarchy ||
                collider.isTrigger || (collider.gameObject.layer != ARLayer && collider.gameObject.layer != SimulationLayer) ||
                (collider.attachedRigidbody != null && !collider.attachedRigidbody.isKinematic))
                continue;

            var source = new NavMeshBuildSource { area = 0 };
            if (surface.box && collider is BoxCollider box)
            {
                source.shape = NavMeshBuildSourceShape.Box;
                source.transform = collider.transform.localToWorldMatrix * Matrix4x4.Translate(box.center);
                source.size = box.size;
                var boxBounds = box.bounds;
                if (!hasBounds) { bounds = boxBounds; hasBounds = true; }
                else bounds.Encapsulate(boxBounds);
            }
            else if (collider is MeshCollider meshCollider && meshCollider.sharedMesh != null)
            {
                source.shape = NavMeshBuildSourceShape.Mesh;
                source.sourceObject = meshCollider.sharedMesh;
                source.transform = collider.transform.localToWorldMatrix;
                source.size = Vector3.one;
                var meshBounds = meshCollider.bounds;
                if (!hasBounds) { bounds = meshBounds; hasBounds = true; }
                else bounds.Encapsulate(meshBounds);
            }
            else
                continue;
            output.Add(source);
        }

        if (hasBounds)
        {
            bounds.Expand(1f);
            return;
        }
        bounds = new Bounds(anchor.position, Vector3.one);
    }


    static void InsertBeforeClosing(StringBuilder xml, string closingTag, StringBuilder addition)
    {
        if (addition.Length == 0) return;
        var position = xml.ToString().LastIndexOf(closingTag, StringComparison.Ordinal);
        if (position < 0)
            throw new InvalidOperationException($"MicroDuck scene XML is missing required {closingTag} element.");
        xml.Insert(position, addition);
    }

    public void Dispose()
    {
        disposed = true;
        surfaces.Clear();
        surfacesByCollider.Clear();
        foundSet.Clear();
        roots.Clear();
        found.Clear();
        diagnosed.Clear();
    }
}
