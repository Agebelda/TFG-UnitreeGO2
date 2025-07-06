using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using UnityEngine.InputSystem;

public class CmdVelPublisherVR : MonoBehaviour
{
    ROSConnection ros;

    [Header("Topic config")]
    public string topicName = "/cmd_vel";
    public float maxLinear = 0.4f;
    public float maxAngular = 0.4f;

    [Header("Input Actions")]
    public InputActionProperty moveLinear;    // joystick izquierdo
    public InputActionProperty rotateAngular; // joystick derecho

    [Header("Head rotation config")]
    public float headYawSensitivity = 0.02f;   // factor de conversión de yaw a angular.z
    public float headYawThreshold = 1.0f;      // umbral mínimo para detectar movimiento de cabeza (grados)

    private float previousYaw = 0.0f;
    private float timeSinceStart = 0f;
    private float delayBeforePublishing = 1.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);

        if (Camera.main != null)
        {
            previousYaw = Camera.main.transform.eulerAngles.y;
        }
    }

    void Update()
    {
        timeSinceStart += Time.deltaTime;
        if (timeSinceStart < delayBeforePublishing)
            return;

        // Leer inputs de movimiento
        Vector2 linearInput = moveLinear.action.ReadValue<Vector2>();
        Vector2 angularInput = rotateAngular.action.ReadValue<Vector2>();

        float linearX = linearInput.y * maxLinear;
        float linearY = linearInput.x * maxLinear;

        // Obtener yaw de la cabeza
        float currentYaw = Camera.main.transform.eulerAngles.y;
        float deltaYaw = Mathf.DeltaAngle(previousYaw, currentYaw);
        previousYaw = currentYaw;

        // Seleccionar fuente de angular.z
        float angularZ = 0.0f;
        float joystickThreshold = 0.05f;

        if (Mathf.Abs(angularInput.x) > joystickThreshold)
        {
            angularZ = -angularInput.x * maxAngular;
        }
        else if (Mathf.Abs(deltaYaw) > headYawThreshold)
        {
            angularZ = deltaYaw * headYawSensitivity;
        }
        else
        {
            angularZ = 0.0f;
        }

        // Publicar Twist
        TwistMsg twist = new TwistMsg
        {
            linear = new Vector3Msg(linearX, linearY, 0),
            angular = new Vector3Msg(0, 0, angularZ)
        };

        ros.Publish(topicName, twist);
    }
}