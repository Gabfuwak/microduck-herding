using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.Interaction.Toolkit.UI.BodyUI;

[DisallowMultipleComponent]
public sealed class MicroDuckHandMenuControls : MonoBehaviour
{
    [SerializeField] private MicroDuckSpawner m_Spawner;
    [SerializeField] private ARPlaneManager m_PlaneManager;
    [SerializeField] private Button m_GoThereButton;
    [SerializeField] private Button m_ResetButton;
    [SerializeField] private Toggle m_PlanesToggle;
    [SerializeField] private Toggle m_FollowToggle;
    [SerializeField] private Toggle m_RightHandToggle;
    [SerializeField] private HandMenu m_SettingsFollower;
    [SerializeField] private HandMenu m_MicrophoneFollower;

    [SerializeField] private GameObject m_SettingsWindow;
    [SerializeField] private GameObject m_CameraWindow;
    [SerializeField] private GameObject m_DetailsWindow;
    [SerializeField] private Toggle m_SettingsWindowToggle;
    [SerializeField] private Toggle m_CameraWindowToggle;
    [SerializeField] private Toggle m_DetailsWindowToggle;
    [SerializeField] private Button m_CloseSettingsButton;
    [SerializeField] private Button m_CloseCameraButton;
    [SerializeField] private Button m_CloseDetailsButton;
    [SerializeField] private TMP_Text m_StateText;

    private readonly List<Renderer> m_PlaneRenderers = new();

    private void OnEnable()
    {
        m_GoThereButton.onClick.AddListener(m_Spawner.BeginTargeting);
        m_ResetButton.onClick.AddListener(m_Spawner.ResetRobot);
        m_PlanesToggle.onValueChanged.AddListener(SetPlanesVisible);
        m_RightHandToggle.onValueChanged.AddListener(SetRightHand);
        m_PlaneManager.trackablesChanged.AddListener(OnPlanesChanged);
        SetPlanesVisible(m_PlanesToggle.isOn);
        SetRightHand(m_RightHandToggle.isOn);
        RefreshFollowToggle();
        m_FollowToggle.onValueChanged.AddListener(SetFollowing);
        m_SettingsWindowToggle.onValueChanged.AddListener(SetSettingsVisible);
        m_CameraWindowToggle.onValueChanged.AddListener(SetCameraVisible);
        m_DetailsWindowToggle.onValueChanged.AddListener(SetDetailsVisible);
        m_CloseSettingsButton.onClick.AddListener(CloseSettings);
        m_CloseCameraButton.onClick.AddListener(CloseCamera);
        m_CloseDetailsButton.onClick.AddListener(CloseDetails);
        SetSettingsVisible(m_SettingsWindowToggle.isOn);
        SetCameraVisible(m_CameraWindowToggle.isOn);
        SetDetailsVisible(m_DetailsWindowToggle.isOn);
    }

    private void OnDisable()
    {
        m_ResetButton.onClick.RemoveListener(m_Spawner.ResetRobot);
        m_GoThereButton.onClick.RemoveListener(m_Spawner.BeginTargeting);
        m_PlanesToggle.onValueChanged.RemoveListener(SetPlanesVisible);
        m_RightHandToggle.onValueChanged.RemoveListener(SetRightHand);
        m_FollowToggle.onValueChanged.RemoveListener(SetFollowing);
        m_PlaneManager.trackablesChanged.RemoveListener(OnPlanesChanged);
        m_SettingsWindowToggle.onValueChanged.RemoveListener(SetSettingsVisible);
        m_CameraWindowToggle.onValueChanged.RemoveListener(SetCameraVisible);
        m_DetailsWindowToggle.onValueChanged.RemoveListener(SetDetailsVisible);
        m_CloseSettingsButton.onClick.RemoveListener(CloseSettings);
        m_CloseCameraButton.onClick.RemoveListener(CloseCamera);
        m_CloseDetailsButton.onClick.RemoveListener(CloseDetails);
    }

    private void LateUpdate()
    {
        RefreshFollowToggle();
        if (m_DetailsWindow.activeInHierarchy)
            RefreshState();
    }

    private void SetSettingsVisible(bool visible) => m_SettingsWindow.SetActive(visible);
    private void SetCameraVisible(bool visible) => m_CameraWindow.SetActive(visible);

    private void SetDetailsVisible(bool visible)
    {
        m_DetailsWindow.SetActive(visible);
        if (visible)
            RefreshState();
    }

    private void CloseSettings() => m_SettingsWindowToggle.isOn = false;
    private void CloseCamera() => m_CameraWindowToggle.isOn = false;
    private void CloseDetails() => m_DetailsWindowToggle.isOn = false;

    private void RefreshState()
    {
        var state = m_Spawner.RobotState;
        if (m_StateText.text != state)
            m_StateText.text = state;
    }

    private void RefreshFollowToggle()
    {
        m_FollowToggle.interactable = m_Spawner.HasRobot;
        // Keep the switch visuals in sync too; SetFollowing ignores reflected state.
        m_FollowToggle.isOn = m_Spawner.IsFollowing;
    }

    private void SetFollowing(bool follow)
    {
        if (follow != m_Spawner.IsFollowing)
            m_Spawner.SetFollowing(follow);
    }

    private void SetRightHand(bool useRightHand)
    {
        var hand = useRightHand ? HandMenu.MenuHandedness.Right : HandMenu.MenuHandedness.Left;
        m_SettingsFollower.menuHandedness = hand;
        m_MicrophoneFollower.menuHandedness = hand;
        m_Spawner.TargetWithLeftHand = useRightHand;
    }

    private void SetPlanesVisible(bool visible)
    {
        foreach (var plane in m_PlaneManager.trackables)
            SetPlaneVisible(plane, visible);
    }

    private void OnPlanesChanged(ARTrackablesChangedEventArgs<ARPlane> changes)
    {
        for (var i = 0; i < changes.added.Count; i++)
            SetPlaneVisible(changes.added[i], m_PlanesToggle.isOn);
    }

    private void SetPlaneVisible(ARPlane plane, bool visible)
    {
        plane.GetComponentsInChildren(true, m_PlaneRenderers);
        // ARPlaneMeshVisualizer owns Renderer.enabled; leave its tracking and collider updates alone.
        for (var i = 0; i < m_PlaneRenderers.Count; i++)
            m_PlaneRenderers[i].forceRenderingOff = !visible;
    }
}
