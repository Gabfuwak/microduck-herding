using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.ARSubsystems;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

// Uses XRI's tracked aim pose and select input, not a second gesture recognizer.
[DisallowMultipleComponent]
[RequireComponent(typeof(MicroDuckSpawner))]
public sealed class MicroDuckTargeting : MonoBehaviour
{
    private MicroDuckSpawner m_Spawner;
    private NearFarInteractor[] m_Interactors;
    private NearFarInteractor m_AimingInteractor;
    private LineRenderer m_Preview;
    private Material m_Material;
    private bool m_ReadyForPinch;
    private bool m_Paused;

    private void Awake()
    {
        m_Spawner = GetComponent<MicroDuckSpawner>();
        m_Interactors = transform.root.GetComponentsInChildren<NearFarInteractor>(true);
        var preview = new GameObject("Go there preview");
        preview.transform.SetParent(transform, false);
        m_Preview = preview.AddComponent<LineRenderer>();
        // The same retained shader and ring dimensions as the navigation marker.
        m_Material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        m_Material.SetColor("_BaseColor", new Color(0.2f, 1f, 0.65f));
        m_Preview.sharedMaterial = m_Material;
        m_Preview.useWorldSpace = false;
        m_Preview.loop = true;
        m_Preview.widthMultiplier = 0.012f;
        m_Preview.generateLightingData = true;
        m_Preview.shadowCastingMode = ShadowCastingMode.Off;
        m_Preview.receiveShadows = false;
        m_Preview.positionCount = 33;
        for (var i = 0; i < 33; i++)
        {
            var angle = i * Mathf.PI * 2f / 33f;
            m_Preview.SetPosition(i, new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)) * 0.055f);
        }
        m_Preview.enabled = false;
    }

    private void LateUpdate()
    {
        m_Preview.enabled = false;
        if (!m_Spawner.IsTargeting || m_Paused || !Application.isFocused)
        {
            m_ReadyForPinch = false;
            return;
        }

        var hand = m_Spawner.TargetWithLeftHand ? InteractorHandedness.Left : InteractorHandedness.Right;
        NearFarInteractor interactor = null;
        foreach (var candidate in m_Interactors)
        {
            if (candidate.isActiveAndEnabled && candidate.handedness == hand && candidate.enableFarCasting)
            {
                interactor = candidate;
                break;
            }
        }
        if (interactor != m_AimingInteractor)
        {
            m_AimingInteractor = interactor;
            m_ReadyForPinch = false;
        }
        if (interactor == null || interactor.hasSelection || interactor.hasHover ||
            interactor.IsBlockedByInteractionWithinGroup() || interactor.TryGetCurrentUIRaycastResult(out _))
        {
            m_ReadyForPinch = false;
            return;
        }

        var select = interactor.logicalSelectState;
        if (!select.active)
            m_ReadyForPinch = true;

        // AR planes are intentionally excluded from XRI's interactable mask.
        // Cast from its native aim origin so the floor remains a surface, not a grab target.
        var origin = interactor.curveOrigin;
        if (!Physics.Raycast(origin.position, origin.forward, out var hit, 10f,
                Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore) ||
            !hit.collider.TryGetComponent<ARPlane>(out var plane) ||
            plane.alignment != PlaneAlignment.HorizontalUp || plane.subsumedBy != null ||
            plane.trackingState == TrackingState.None)
        {
            if (select.active)
                m_ReadyForPinch = false;
            return;
        }

        m_Preview.transform.SetPositionAndRotation(hit.point + plane.normal * 0.006f,
            Quaternion.FromToRotation(Vector3.up, plane.normal));
        m_Preview.enabled = true;
        if (m_ReadyForPinch && select.wasPerformedThisFrame)
        {
            m_ReadyForPinch = false;
            if (m_Spawner.TryPlaceOrNavigate(hit.point, plane.normal))
                m_Preview.enabled = false;
        }
    }

    private void OnApplicationPause(bool paused)
    {
        m_Paused = paused;
        m_ReadyForPinch = false;
    }

    private void OnDisable()
    {
        m_ReadyForPinch = false;
        if (m_Preview != null)
            m_Preview.enabled = false;
    }

    private void OnDestroy()
    {
        if (m_Material != null)
            Destroy(m_Material);
    }
}
