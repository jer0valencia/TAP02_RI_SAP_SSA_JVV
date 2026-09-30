# KUKA KR10 MoveIt Workspace

Este repositorio tiene como propósito servir como guía de ejecución del ecosistema de MoveIt desarrollado para las actividades requeridas en el Taller 2 de Robótica de la Universidad EIA.

En este repositorio se encuentra todo el workspace utilizado durante el desarrollo del proyecto, incluyendo:

- Archivos de configuración de MoveIt.
- Modelos URDF y XACRO del robot KUKA KR10 R1100 sixx.
- Paquetes desarrollados para el cumplimiento de todas las partes del taller.
- Scripts y recursos necesarios para la simulación y planificación de trayectorias.

---

## 1. Compilación del workspace

Antes de comenzar, es necesario compilar todo el workspace para generar los archivos requeridos por ROS 2 y MoveIt.

```bash
cd ws_kuka_kr10
colcon build
source install/setup.bash
```

Una vez finalizada la compilación, todos los paquetes y dependencias estarán disponibles para su ejecución.

---

## 2. Inicialización del entorno

Con el workspace compilado y cargado, ya es posible comenzar a trabajar con el entorno de simulación y planificación.

Como primer paso, es recomendable mover el robot a la configuración **Home**, que será utilizada como posición inicial durante las pruebas y ejercicios del taller.

Para ello, primero se debe iniciar el entorno de MoveIt y graficarlo en Rviz:
```
ros2 launch kuka_kr10_moveit_config demo.launch.py
```
Luego debesde poner el robot en home, eso lo hacemos poniendo en la casilla goal state en Home y luego le damos a plan execute.
Ahora ya todo esta listo para que podeamos cargar el enviroment y hacer todas las tareas relacionadas a la actividad
Lo primero que vamos a hacer es cargar el enviorment, para eso en otra terminal hacemos

```
ros2 run kuka_kr10_tasks create_environment 
```
En el entorno esta cargada cada una de las posciones del robot home, prepick pick preplace place, se pueden probar las trayectorias poniendo en goal state el punto final de interes


Y ahí puedes seguir con el siguiente bloque de comandos que uses para lanzar `move_group`, RViz o el paquete que desarrollaste.

**Tip**ara GitHub:** si después vas a poner capturas de RViz, añade algo así:

```markdown
## Entorno de simulación

![RViz](images/e.png
```

si la imagen está dentro de una carpeta `images` del repositorio.
