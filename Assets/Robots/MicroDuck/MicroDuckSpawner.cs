using UnityEngine;
using UnityEngine.SceneManagement;

[DisallowMultipleComponent]
public sealed class MicroDuckSpawner : MonoBehaviour
{
    [SerializeField] private Camera m_CameraToFace;
    [SerializeField] private GameObject m_ObjectPrefab;

    private GameObject m_LiveDuck;
    private GameObject m_PreparedRoot;
    private GameObject m_PreparedDuck;
    private MicroDuckPolicyController m_PreparedController;
    private bool m_PreparationFailed;
    private float m_NextSurfaceRefresh;

    public Camera cameraToFace
    {
        get => m_CameraToFace != null ? m_CameraToFace : Camera.main;
        set => m_CameraToFace = value;
    }

    private void OnEnable()
    {
        m_PreparationFailed = false;
        if (Application.isPlaying)
            PrepareSpawn();
    }

    private void Update()
    {
        if (m_LiveDuck != null || m_PreparationFailed)
            return;

        if (m_PreparedDuck == null)
        {
            PrepareSpawn();
            return;
        }

        if (Time.unscaledTime < m_NextSurfaceRefresh)
            return;

        m_NextSurfaceRefresh = Time.unscaledTime + 0.25f;
        try
        {
            m_PreparedController.RefreshSurfaces();
        }
        catch (System.Exception exception)
        {
            FailPreparation(exception);
        }
    }

    private void OnDisable()
    {
        ReleasePreparedSpawn();
    }

    private void OnDestroy()
    {
        ReleasePreparedSpawn();
    }

    private void PrepareSpawn()
    {
        if (m_PreparationFailed || m_LiveDuck != null || m_PreparedDuck != null)
            return;

        try
        {
            if (m_ObjectPrefab == null)
                throw new System.InvalidOperationException("MicroDuckSpawner requires a MicroDuck prefab.");

            m_PreparedRoot = new GameObject("MicroDuck Prepared Root");
            m_PreparedRoot.SetActive(false);
            m_PreparedRoot.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            m_PreparedRoot.transform.localScale = Vector3.one;
            SceneManager.MoveGameObjectToScene(m_PreparedRoot, gameObject.scene);

            m_PreparedDuck = Instantiate(m_ObjectPrefab, m_PreparedRoot.transform, false);
            m_PreparedDuck.SetActive(false);
            m_PreparedController = m_PreparedDuck.GetComponent<MicroDuckPolicyController>();
            if (m_PreparedController == null)
                throw new System.InvalidOperationException("MicroDuck prefab has no MicroDuckPolicyController.");

            m_PreparedController.InitializeSimulation();
            m_NextSurfaceRefresh = Time.unscaledTime + 0.25f;
        }
        catch (System.Exception exception)
        {
            FailPreparation(exception);
        }
    }

    public bool TryPlaceOrNavigate(Vector3 spawnPoint, Vector3 spawnNormal)
    {
        if (m_LiveDuck != null)
            return m_LiveDuck.GetComponent<MicroDuckNavigator>().SetDestination(spawnPoint);
        if (m_PreparationFailed)
            return false;
        if (m_PreparedDuck == null)
            PrepareSpawn();
        if (m_PreparationFailed || m_PreparedDuck == null)
            return false;

        Camera facingCamera = cameraToFace;
        Vector3 projectedForward = facingCamera != null
            ? Vector3.ProjectOnPlane(facingCamera.transform.position - spawnPoint, spawnNormal)
            : Vector3.ProjectOnPlane(Vector3.forward, spawnNormal);
        if (projectedForward.sqrMagnitude < 0.000001f)
            projectedForward = Vector3.ProjectOnPlane(Vector3.right, spawnNormal);
        Quaternion rotation = Quaternion.LookRotation(projectedForward, spawnNormal);

        GameObject duck = m_PreparedDuck;
        try
        {
            duck.transform.SetPositionAndRotation(spawnPoint, rotation);
            m_PreparedController.PlacePreparedSimulation();
            var navigator = duck.GetComponent<MicroDuckNavigator>() ?? duck.AddComponent<MicroDuckNavigator>();
            navigator.Initialize(m_PreparedController);

            Transform root = m_PreparedRoot.transform;
            duck.transform.SetParent(null, true);
            m_LiveDuck = duck;
            m_PreparedDuck = null;
            m_PreparedController = null;
            m_PreparedRoot = null;
            DestroyOwnedObject(root.gameObject);
            duck.SetActive(true);
            return true;
        }
        catch (System.Exception exception)
        {
            FailPreparation(exception);
            return false;
        }
    }

    public void PlaceOrNavigate(Vector3 spawnPoint, Vector3 spawnNormal)
    {
        if (!TryPlaceOrNavigate(spawnPoint, spawnNormal))
            Debug.LogWarning("Could not place MicroDuck or set its destination.", this);
    }

    public void ResetRobot()
    {
        if (m_LiveDuck == null)
            return;

        m_LiveDuck.SetActive(false);
        DestroyOwnedObject(m_LiveDuck);
        m_LiveDuck = null;
        // Update prepares the next duck; placement still requires the user's gesture.
    }

    private void FailPreparation(System.Exception exception)
    {
        m_PreparationFailed = true;
        Debug.LogException(exception, this);
        ReleasePreparedSpawn();
    }

    private void ReleasePreparedSpawn()
    {
        if (m_PreparedController != null)
            m_PreparedController.ReleaseSimulation();

        if (m_PreparedRoot != null)
            DestroyOwnedObject(m_PreparedRoot);
        else if (m_PreparedDuck != null)
            DestroyOwnedObject(m_PreparedDuck);

        m_PreparedRoot = null;
        m_PreparedDuck = null;
        m_PreparedController = null;
    }

    private static void DestroyOwnedObject(GameObject ownedObject)
    {
        if (ownedObject == null)
            return;
        if (Application.isPlaying)
            Destroy(ownedObject);
        else
            DestroyImmediate(ownedObject);
    }
}
