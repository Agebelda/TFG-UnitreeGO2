# go2_vr - Sistema de realidad virtual para el control inmersivo de un perro robótico Unitree Go2

Este paquete forma parte del Trabajo de Fin de Grado de Alejandro Gea Belda (Universidad de Alicante) y proporciona los nodos ROS 2 necesarios para habilitar la teleoperación inmersiva del robot cuadrúpedo Unitree GO2.

---

## Dependencias

Este paquete está diseñado para funcionar dentro de un workspace ROS 2 Humble y requiere que el paquete externo [`ros_tcp_endpoint`](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) esté presente en la carpeta `src/` del workspace.

---

## Compilación

```bash
cd ~/ros2_ws/src
git clone https://github.com/Agebelda/TFG-UnitreeGO2.git
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ..
colcon build
source install/setup.bash
```

---

## Lanzamiento de nodos

### En el robot:

ros2 launch go2_vr robot.launch.py

### En el PC:

ros2 launch go2_vr pc.launch.py