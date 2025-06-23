import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import open3d.visualization.rendering as rendering  # type: ignore
import cv2

# Parámetros
image_width, image_height = 640, 480

# ==================== PIPELINE DE REALSENSE ====================

# Inicializar pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, image_width, image_height, rs.format.z16, 30)
config.enable_stream(rs.stream.color, image_width, image_height, rs.format.bgr8, 30)
profile = pipeline.start(config)

# Alinear profundidad al color
align = rs.align(rs.stream.color)

# Activar emisor infrarrojo si es posible
device = profile.get_device()
depth_sensor = device.first_depth_sensor()
if depth_sensor.supports(rs.option.emitter_enabled):
    depth_sensor.set_option(rs.option.emitter_enabled, 1)

# ===================== VISUALIZADOR OPEN3D ======================

# Visualizador de Open3D
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Nube de Puntos en Tiempo Real")
pcd = o3d.geometry.PointCloud()
vis.add_geometry(pcd)
first_frame = True

# Renderizador sin ventana para cámaras estéreo virtuales
render = rendering.OffscreenRenderer(image_width, image_height)
render.scene.set_background([0, 0, 0, 1])
material = rendering.MaterialRecord()
material.shader = "defaultUnlit"

# Objeto pointcloud de librealsense
pc = rs.pointcloud()

print("🚀 Visualización en tiempo real iniciada. Presiona ESC para salir.")

# ================= BUCLE PROCESAMIENTO DE DATOS =================

try:
    while True:
        
        # Esperar por frames y procesarlos
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Calcular nube de puntos
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

        # Obtener color
        color_image = np.asanyarray(color_frame.get_data())
        color_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        colors = color_rgb.reshape(-1, 3) / 255.0

        # Asegurar misma longitud
        min_len = min(len(verts), len(colors))
        verts = verts[:min_len]
        colors = colors[:min_len]

        # Rotación de nube para que se vea correctamente
        R = np.array([
            [1, 0,  0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        verts = verts @ R.T

        # Asignar a la PointCloud
        pcd.points = o3d.utility.Vector3dVector(verts)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        # Bloquear la vista de la nube de puntos
        if first_frame:
            vis.reset_view_point(True)
            first_frame = False

        # Mostrar imagen RGB original
        cv2.imshow("RGB Frame", color_image)

        # ================== Cámaras virtuales ====================

        depth_stream = depth_frame.profile.as_video_stream_profile()
        color_stream = color_frame.profile.as_video_stream_profile()
        extr = depth_stream.get_extrinsics_to(color_stream)

        # Matriz de extrínsecos (rotación + traslación)
        extr_matrix = np.eye(4)
        extr_matrix[:3, :3] = np.array(extr.rotation).reshape(3, 3)
        extr_matrix[:3, 3] = np.array(extr.translation)

        # Posición y orientación de la cámara
        camera_position = extr_matrix[:3, 3]
        forward = extr_matrix[:3, 2]  # Z hacia delante
        up = extr_matrix[:3, 1]       # Y hacia arriba
        right = np.cross(forward, up)

        # Punto objetivo (lo que las cámaras deben mirar originalmente)
        lookat = camera_position + forward

        # Distancia entre cámaras (6.5 cm)
        eye_distance = 0.265

        # Colocar las cámaras justo detrás del lookat, desplazadas lateralmente
        eye_left = camera_position - forward + (right * (eye_distance / 2))
        eye_right = camera_position - forward - (right * (eye_distance / 2))

        # Nuevo lookat que mantiene la dirección de mirada recta hacia adelante
        lookat_left = eye_left + forward
        lookat_right = eye_right + forward

        # Actualizar geometría en el renderizador
        render.scene.clear_geometry()
        render.scene.add_geometry("pcd", pcd, material)

        # Renderizar cámara izquierda (visión estéreo paralela)
        render.setup_camera(45.0, eye_left, lookat_left, up)
        img_left = render.render_to_image()
        img_left_np = np.asarray(img_left)

        # Renderizar cámara derecha (visión estéreo paralela)
        render.setup_camera(45.0, eye_right, lookat_right, up)
        img_right = render.render_to_image()
        img_right_np = np.asarray(img_right)

        # Mostrar imágenes estéreo
        cv2.imshow("Camara Izquierda", cv2.cvtColor(img_left_np, cv2.COLOR_RGB2BGR))
        cv2.imshow("Camara Derecha", cv2.cvtColor(img_right_np, cv2.COLOR_RGB2BGR))

        # Salir con ESC
        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("\n🛑 Interrupción del usuario.")

finally:
    vis.destroy_window()
    cv2.destroyAllWindows()
    pipeline.stop()
