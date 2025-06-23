# TFG - Teleoperación en Realidad Virtual del Unitree GO2

Este repositorio contiene el desarrollo del Trabajo de Fin de Grado enfocado en la teleoperación del robot cuadrúpedo **Unitree GO2** mediante un sistema de realidad virtual, empleando ROS 2, Unity y visión estéreo con cámaras Intel RealSense.

---

## 🧠 Descripción general

El objetivo del proyecto es crear un entorno inmersivo de control remoto para el robot Unitree GO2, combinando:

- **Visión estéreo en tiempo real** desde la cámara RealSense D435i.
- **Renderizado en Unity** para mostrar la visión del robot en gafas de realidad virtual.
- **Control remoto del robot** desde Unity a través de ROS 2 usando ROS-TCP-Connector.
- **Transmisión de comandos básicos** (marcha, rotación, dirección) y posible integración de seguimiento con el movimiento de cabeza.

---

## 📁 Estructura del repositorio
<pre> ## 📁 Estructura del repositorio ``` TFG-UnitreeGO2/ ├── realsense/ │ ├── raw/ # Scripts para capturar y procesar imágenes (OpenCV, filtros) │ └── ros/ # Nodos ROS 2 que publican imágenes estéreo y compresión │ ├── unity-vr/ # Proyecto Unity para VR y comunicación con ROS (subido por Plastic SCM) │ ├── scripts_utils/ # Scripts auxiliares y pruebas de visualización │ ├── doc/ # Documentación, diseño de arquitectura y esquemas │ ├── .gitignore └── README.md ``` </pre>

## 🛠️ Tecnologías utilizadas

- 🐧 **Ubuntu 22.04 + ROS 2 Humble**
- 🤖 **Unitree GO2 con ROS 2 Foxy**
- 🎮 **Unity + XR Toolkit + ROS-TCP-Connector**
- 🎥 **Intel RealSense D435i**
- 🧠 **Python + OpenCV + Open3D**
- 🌐 **GitHub para código y Plastic SCM para el proyecto Unity**

---

## 🚀 Cómo ejecutar

### 1. Nodos ROS 2 en el robot:
ros2 run virtual_stereo_cam stereo_disparity_publisher

### 2. Nodo de compresión en PC:
ros2 run virtual_stereo_cam image_compressor_left
ros2 run virtual_stereo_cam image_compressor_right

### 3. Servidor de comunicación con unity:
ros2 run ros_tcp_endpoint default_server_endpoint

### 4. Proyecto Unity:
Abierto desde Unity Hub (unity-vr/).

Conectado a ROS 2 Humble mediante ROSConnection.

Publica comandos básicos de control.

🙋 Autor
Alejandro Gea
Estudiante de Ingeniería Robótica
Universidad de Alicante