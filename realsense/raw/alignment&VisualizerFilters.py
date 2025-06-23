import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import cv2

# Inicializar pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# Alinear profundidad al color
align = rs.align(rs.stream.color)

# Activar proyector láser si es posible
device = profile.get_device()
depth_sensor = device.first_depth_sensor()
if depth_sensor.supports(rs.option.emitter_enabled):
    depth_sensor.set_option(rs.option.emitter_enabled, 1)

# Visualizador de Open3D
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Nube de Puntos en Tiempo Real")
pcd = o3d.geometry.PointCloud()
vis.add_geometry(pcd)
first_frame = True

# Objeto pointcloud de librealsense
pc = rs.pointcloud()

print("🚀 Visualización en tiempo real iniciada. Presiona ESC para salir.")

try:
    while True:
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

        # Obtener color y ajustar
        color_image = np.asanyarray(color_frame.get_data())
        color_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        colors = color_rgb.reshape(-1, 3) / 255.0

        # Asegurar misma longitud
        min_len = min(len(verts), len(colors))
        verts = verts[:min_len]
        colors = colors[:min_len]

        # 🔄 Rotar nube en eje Y (ej: 180° alrededor de Y)
        R = np.array([
            [1, 0,  0],
            [ 0, -1,  0],
            [ 0, 0, -1]
        ])
        verts = verts @ R.T

        # Asignar a PointCloud
        pcd.points = o3d.utility.Vector3dVector(verts)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        if first_frame:
            vis.reset_view_point(True)
            first_frame = False

        # Mostrar imagen RGB opcionalmente
        cv2.imshow("RGB Frame", color_image)
        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("\n🛑 Interrupción del usuario.")

finally:
    vis.destroy_window()
    cv2.destroyAllWindows()
    pipeline.stop()
