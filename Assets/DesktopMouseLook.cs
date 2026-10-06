using UnityEngine;
using UnityEngine.InputSystem;

public sealed class DesktopMouseLook : MonoBehaviour
{
    [SerializeField] private Transform cameraTransform;
    [SerializeField] private float sensitivity = 0.1f;

    private float pitch;

    private void Start()
    {
        pitch = Mathf.DeltaAngle(0f, cameraTransform.localEulerAngles.x);
        Cursor.lockState = CursorLockMode.Locked;
    }

    private void Update()
    {
        Mouse mouse = Mouse.current;
        if (mouse == null)
            return;

        Vector2 delta = mouse.delta.ReadValue();
        transform.Rotate(0f, delta.x * sensitivity, 0f);
        pitch = Mathf.Clamp(pitch - delta.y * sensitivity, -85f, 85f);
        cameraTransform.localRotation = Quaternion.Euler(pitch, 0f, 0f);
    }

    private void OnDisable()
    {
        Cursor.lockState = CursorLockMode.None;
    }
}
