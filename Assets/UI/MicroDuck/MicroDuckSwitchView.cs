using UnityEngine;
using UnityEngine.UI;

[DisallowMultipleComponent]
[RequireComponent(typeof(Toggle))]
public sealed class MicroDuckSwitchView : MonoBehaviour
{
    [SerializeField] private Image m_Track;
    [SerializeField] private RectTransform m_Thumb;

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

    // Presentation only; the Toggle owns the value, not an application setting.
    private void Refresh(bool isOn)
    {
        m_Track.color = isOn ? new Color32(106, 176, 255, 255) : new Color32(42, 45, 49, 255);
        var position = m_Thumb.anchoredPosition;
        var track = m_Track.rectTransform.rect;
        position.x = (isOn ? 1f : -1f) * (track.width - track.height) * 0.5f;
        m_Thumb.anchoredPosition = position;
    }
}
