using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(CharacterController))]
public sealed class DesktopMovement : MonoBehaviour
{
    [SerializeField, Min(0f)] private float moveSpeed = 2f;

    private CharacterController characterController;

    private void Awake()
    {
        characterController = GetComponent<CharacterController>();
    }

    private void Update()
    {
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
            return;

        Vector3 input = new Vector3(
            (keyboard.rightArrowKey.isPressed || keyboard.dKey.isPressed ? 1f : 0f) -
            (keyboard.leftArrowKey.isPressed || keyboard.aKey.isPressed ? 1f : 0f),
            (keyboard.spaceKey.isPressed ? 1f : 0f) - (keyboard.shiftKey.isPressed ? 1f : 0f),
            (keyboard.upArrowKey.isPressed || keyboard.wKey.isPressed ? 1f : 0f) -
            (keyboard.downArrowKey.isPressed || keyboard.sKey.isPressed ? 1f : 0f));
        if (input == Vector3.zero)
            return;

        Vector3 direction = transform.TransformDirection(Vector3.ClampMagnitude(input, 1f));
        characterController.Move(direction * (moveSpeed * Time.deltaTime));
    }
}
