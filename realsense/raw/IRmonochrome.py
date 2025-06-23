import pyrealsense2 as rs
import numpy as np
import cv2

# Inicializar el pipeline
pipeline = rs.pipeline()
config = rs.config()

# Activar los streams de las cámaras infrarrojas (izquierda y derecha)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)  # IR Izquierda
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)  # IR Derecha

# Iniciar el pipeline
pipeline.start(config)

# Obtener el dispositivo y desactivar el emisor láser
device = pipeline.get_active_profile().get_device()
depth_sensor = device.first_depth_sensor()
depth_sensor.set_option(rs.option.emitter_enabled, 0)  # Apagar el proyector IR

print("🔹 Proyector láser desactivado. Mostrando imágenes IR...")

try:
    while True:
        frames = pipeline.wait_for_frames()
        ir_left = frames.get_infrared_frame(1)  # Cámara infrarroja izquierda
        ir_right = frames.get_infrared_frame(2)  # Cámara infrarroja derecha

        if not ir_left or not ir_right:
            print("⚠ Error al obtener las imágenes IR.")
            continue

        # Convertir las imágenes en matrices NumPy
        ir_left_image = np.asanyarray(ir_left.get_data())
        ir_right_image = np.asanyarray(ir_right.get_data())

        # Mostrar las imágenes
        cv2.imshow("Infrared Left", ir_left_image)
        cv2.imshow("Infrared Right", ir_right_image)

        # Salir con la tecla ESC
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("🛑 ESC presionado, cerrando...")
            break

except KeyboardInterrupt:
    print("\n🛑 Detenido por el usuario.")

finally:
    cv2.destroyAllWindows()
    pipeline.stop()
