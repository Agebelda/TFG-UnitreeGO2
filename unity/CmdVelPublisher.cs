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
    public float maxAngular = 1.2f;

    [Header("Input Actions")]
    public InputActionProperty moveLinear;    // joystick izquierdo
    public InputActionProperty rotateAngular; // joystick derecho

    [Header("Head rotation config")]
    public float headYawSensitivity = 0.04f;   // sensibilidad del giro de cabeza
    public float headYawThreshold = 0.1f;      // umbral mínimo de giro de cabeza (grados)

    private float previousYaw = 0.0f;
    private float timeSinceStart = 0f;
    private float delayBeforePublishing = 1.0f;

    private float[] angularZBuffer = new float[8];  // buffer de persistencia
    private int bufferIndex = 0;
    private bool wasMoving = false;

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
        float linearY = -linearInput.x * maxLinear;

        // Aplicar umbral para evitar movimientos laterales indeseados
        float lateralDeadZone = 0.1f;
        if (Mathf.Abs(linearY) < lateralDeadZone)
            linearY = 0f;

        // Obtener yaw de la cabeza
        float currentYaw = Camera.main.transform.eulerAngles.y;
        float deltaYaw = Mathf.DeltaAngle(previousYaw, currentYaw);
        previousYaw = currentYaw;

        // Seleccionar fuente de angular.z
        float angularZ = 0.0f;
        float joystickThreshold = 0.05f;

        if (Mathf.Abs(angularInput.x) > joystickThreshold)
        {
            float amplified = Mathf.Clamp(angularInput.x * 1.5f, -1f, 1f);
            angularZ = -amplified * maxAngular;
        }
        else if (Mathf.Abs(deltaYaw) > headYawThreshold)
        {
            angularZ = -deltaYaw * headYawSensitivity;
        }

        // Guardar angularZ en el buffer circular
        angularZBuffer[bufferIndex] = angularZ;
        bufferIndex = (bufferIndex + 1) % angularZBuffer.Length;

        // Usar el valor más fuerte reciente
        float smoothedAngularZ = 0f;
        foreach (float val in angularZBuffer)
        {
            if (Mathf.Abs(val) > Mathf.Abs(smoothedAngularZ))
                smoothedAngularZ = val;
        }

        // Determinar si hay movimiento significativo
        bool isCurrentlyMoving =
            Mathf.Abs(linearX) > 0.01f ||
            Mathf.Abs(linearY) > 0.01f ||
            Mathf.Abs(smoothedAngularZ) > 0.01f;

        if (isCurrentlyMoving)
        {
            TwistMsg twist = new TwistMsg
            {
                linear = new Vector3Msg(linearX, linearY, 0),
                angular = new Vector3Msg(0, 0, smoothedAngularZ)
            };

            ros.Publish(topicName, twist);
            wasMoving = true;
        }
        else if (wasMoving)
        {
            TwistMsg twist = new TwistMsg
            {
                linear = new Vector3Msg(0, 0, 0),
                angular = new Vector3Msg(0, 0, 0)
            };

            ros.Publish(topicName, twist);
            wasMoving = false;
        }
    }
}