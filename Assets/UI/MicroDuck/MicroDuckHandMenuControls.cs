using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.Interaction.Toolkit.UI.BodyUI;

[DisallowMultipleComponent]
public sealed class MicroDuckHandMenuControls : MonoBehaviour
{
    [SerializeField] private MicroDuckSpawner m_Spawner;
    [SerializeField] private ARPlaneManager m_PlaneManager;
    [SerializeField] private Button m_ResetButton;
    [SerializeField] private Toggle m_PlanesToggle;
    [SerializeField] private Toggle m_RightHandToggle;
    [SerializeField] private HandMenu m_SettingsFollower;
    [SerializeField] private HandMenu m_MicrophoneFollower;

    private readonly List<Renderer> m_PlaneRenderers = new();

    private void OnEnable()
    {
        m_ResetButton.onClick.AddListener(m_Spawner.ResetRobot);
        m_PlanesToggle.onValueChanged.AddListener(SetPlanesVisible);
        m_RightHandToggle.onValueChanged.AddListener(SetRightHand);
        m_PlaneManager.trackablesChanged.AddListener(OnPlanesChanged);
        SetPlanesVisible(m_PlanesToggle.isOn);
        SetRightHand(m_RightHandToggle.isOn);
    }

    private void OnDisable()
    {
        m_ResetButton.onClick.RemoveListener(m_Spawner.ResetRobot);
        m_PlanesToggle.onValueChanged.RemoveListener(SetPlanesVisible);
        m_RightHandToggle.onValueChanged.RemoveListener(SetRightHand);
        m_PlaneManager.trackablesChanged.RemoveListener(OnPlanesChanged);
    }

    private void SetRightHand(bool useRightHand)
    {
        var hand = useRightHand ? HandMenu.MenuHandedness.Right : HandMenu.MenuHandedness.Left;
        m_SettingsFollower.menuHandedness = hand;
        m_MicrophoneFollower.menuHandedness = hand;
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
