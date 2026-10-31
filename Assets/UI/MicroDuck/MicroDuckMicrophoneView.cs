using UnityEngine;
using UnityEngine.UI;

[DisallowMultipleComponent]
[RequireComponent(typeof(Toggle))]
public sealed class MicroDuckMicrophoneView : MonoBehaviour
{
    [SerializeField] private Image m_Icon;
    [SerializeField] private Sprite m_MicrophoneIcon;
    [SerializeField] private Sprite m_MutedMicrophoneIcon;

    private Toggle m_Toggle;

    private void Awake()
    {
        m_Toggle = GetComponent<Toggle>();
    }

    private void OnEnable()
    {
        m_Toggle.onValueChanged.AddListener(Refresh);
        Refresh(m_Toggle.isOn);
    }

    private void OnDisable()
    {
        m_Toggle.onValueChanged.RemoveListener(Refresh);
    }

    // Presentation only: the toggle does not enable or mute microphone capture.
    private void Refresh(bool microphoneEnabled)
    {
        m_Icon.sprite = microphoneEnabled ? m_MicrophoneIcon : m_MutedMicrophoneIcon;
        m_Icon.color = microphoneEnabled
            ? new Color32(255, 255, 255, 255)
            : new Color32(239, 100, 107, 255);
    }
}
