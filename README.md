# TFG - Teleoperación en Realidad Virtual del Unitree GO2

Este repositorio contiene el desarrollo del Trabajo de Fin de Grado enfocado en la teleoperación del robot cuadrúpedo **Unitree GO2** mediante un sistema de realidad virtual, empleando ROS 2, Unity y visión estéreo con cámaras Intel RealSense.

---

## Descripción general

El objetivo del proyecto es crear un sistema de realidad virtual para el control inmersivo de un perro robótico, combinando:

- **Visión estéreo en tiempo real** mediante una cámara Intel RealSense D435i.
- **Renderizado en Unity** para visualizar la escena en unas gafas Meta Quest 2.
- **Control remoto del robot** desde Unity a través de ROS 2 mediante el paquete ROS-TCP-Connector.
- **Teleoperación mediante mandos VR** y sincronización con el movimiento de la cabeza del operador.

---

## Tecnologías utilizadas

- **Ubuntu 22.04 + ROS 2 Humble**
- **Unitree GO2 con ROS 2 Foxy**
- **Unity + XR Toolkit + ROS-TCP-Connector**
- **Intel RealSense D435i**
- **Python + OpenCV + Open3D**

---

## Cómo ejecutar

### 1. En el robot:
ros2 launch go2_vr robot.launch.py

### 2. En el pc:
ros2 launch go2_vr pv-launch.py

> Es necesario clonar el repositorio [`ROS-TCP-Endpoint`](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) en la carpeta `src/` del workspace y compilarlo con `colcon build`.

### 3. En las gafas Meta Quest 2:

- Transfiere e instala el archivo `vr-viewer-builtin.apk`.
- Ejecuta la aplicación desde las gafas.

---

## Autor

**Alejandro Gea Belda**  
Estudiante del Grado en Ingeniería Robótica  
Universidad de Alicante