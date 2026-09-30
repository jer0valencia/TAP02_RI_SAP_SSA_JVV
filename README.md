# KUKA KR10 MoveIt Workspace

Este repositorio tiene como propósito servir como guía de ejecución del ecosistema de MoveIt desarrollado para las actividades requeridas en el Taller 2 de Robótica de la Universidad EIA.

En este repositorio se encuentra todo el workspace utilizado durante el desarrollo del proyecto, incluyendo:

- Archivos de configuración de MoveIt.
- Modelos URDF y XACRO del robot KUKA KR10 R1100 sixx.
- Paquetes desarrollados para el cumplimiento de todas las partes del taller.
- Scripts de Python para la generación de perfiles cúbicos y quínticos.
- Recursos necesarios para la simulación y planificación de trayectorias.

---

# 1. Compilación del Workspace

Antes de comenzar, es necesario compilar todo el workspace para generar los archivos requeridos por ROS 2 y MoveIt.

```bash
cd ~/ws_kuka_kr10
colcon build
source install/setup.bash
```

Una vez finalizada la compilación, todos los paquetes estarán disponibles para su ejecución.

Cada vez que se abra una terminal nueva, se debe cargar el workspace:

```bash
cd ~/ws_kuka_kr10
source install/setup.bash
```

Si se modifica algún archivo C++, se debe volver a compilar el workspace:

```bash
cd ~/ws_kuka_kr10
colcon build
source install/setup.bash
```

---

# 2. Inicialización del Entorno

Con el workspace compilado y cargado, ya es posible comenzar a trabajar con el entorno de simulación y planificación.

Como primer paso, se recomienda mover el robot a la configuración Home, que será utilizada como posición inicial durante las pruebas y ejercicios del taller.

Para ello, inicia MoveIt y RViz con el siguiente comando:

```bash
cd ~/ws_kuka_kr10
source install/setup.bash

ros2 launch kuka_kr10_moveit_config demo.launch.py
```

A continuación, en RViz:

1. Localiza el panel Motion Planning.
2. Selecciona Home en la casilla Goal State.
3. Presiona Plan & Execute.
4. Espera a que el movimiento termine.

Una vez completado este paso, el robot quedará en su posición inicial.

---

# 3. Carga del Entorno de Trabajo

Una vez que el robot se encuentra en Home, se puede cargar el entorno de trabajo utilizado durante el taller.

En una nueva terminal:

```bash
cd ~/ws_kuka_kr10
source install/setup.bash

ros2 run kuka_kr10_tasks create_environment
```

Este nodo añade los siguientes objetos a la PlanningScene:

- `pick_table`: mesa de recogida.
- `part`: pieza manipulada.
- `central_post`: obstáculo central.
- `place_table`: mesa de depósito.

Además, se encuentran definidas las configuraciones:

- Home.
- Pre-Pick.
- Pick.
- Pre-Place.
- Place.

Estas posiciones pueden probarse directamente desde RViz utilizando la opción Goal State y ejecutando la planificación correspondiente.

El entorno debe cargarse antes de evaluar los planeadores o ejecutar el ciclo completo, ya que los movimientos libres deben considerar las mesas, la pieza y el obstáculo central.

---

# 4. Consultar el Estado Actual del Robot

Durante el desarrollo del taller es útil conocer el estado actual del manipulador para obtener valores articulares y transformaciones entre marcos de referencia.

## Obtener los ángulos articulares

```bash
ros2 topic echo /joint_states --once
```

Este comando muestra los valores actuales de todas las articulaciones del robot.

Los valores del campo `position` corresponden a:

```text
joint_a1
joint_a2
joint_a3
joint_a4
joint_a5
joint_a6
```

## Obtener la transformación Base a Herramienta

```bash
ros2 run tf2_ros tf2_echo base_link tool0
```

Este comando permite consultar la transformación entre la base del robot y el efector final.

La salida contiene:

- Traslación.
- Cuaternión.
- Ángulos RPY.
- Matriz de transformación.

Para detener el comando utiliza:

```text
Ctrl + C
```

---

# 5. Evaluación de Planeadores

Se dispone de un nodo encargado de generar trayectorias para el tramo:

```text
Home → Pre-Pick
```

El nodo calcula diferentes métricas de desempeño:

- Tiempo de planificación.
- Tiempo total de ejecución del planeador.
- Duración de la trayectoria.
- Longitud del camino articular.
- Suavidad geométrica.
- Número de puntos generados.
- Éxito o fallo de planificación.

Antes de ejecutar la evaluación:

1. Inicia MoveIt y RViz.
2. Carga el entorno de trabajo.
3. Lleva el robot a la configuración Home.

## Evaluar RRTConnect

```bash
cd ~/ws_kuka_kr10
source install/setup.bash

ros2 run kuka_kr10_tasks evaluate \
  --ros-args \
  -p planner_id:=RRTConnectkConfigDefault
```

## Evaluar RRT*

```bash
cd ~/ws_kuka_kr10
source install/setup.bash

ros2 run kuka_kr10_tasks evaluate \
  --ros-args \
  -p planner_id:=RRTstarkConfigDefault
```

Al ejecutar el nodo se obtiene una salida similar a la siguiente:

```text
[INFO] Planning successful
[INFO] Planning time: 0.055963 s
[INFO] Clock time: 0.122857 s
[INFO] Number of trajectory points: 183
[INFO] Planned duration: 18.198975 s
[INFO] Joint path length: 7.315865 rad
[INFO] Geometric smoothness: 0.001144484919
[INFO] Trajectory was not executed
```

Para realizar una comparación adecuada, se deben conservar las mismas condiciones durante las pruebas:

- Mismo estado inicial.
- Mismo estado final.
- Misma escena de colisión.
- Mismo tiempo máximo de planificación.
- Mismos factores de velocidad y aceleración.
- Misma cantidad de intentos para cada planeador.

Se recomienda ejecutar varias pruebas con cada planeador debido a que los algoritmos utilizados son probabilísticos.

---

# 6. Validación de Trayectorias Cartesianas

Para comparar la interpolación cúbica y quíntica en MoveIt se dispone del nodo `cartesian_path`.

La comparación se realiza sobre el tramo:

```text
Pre-Pick → Pick
```

Antes de ejecutar el nodo:

1. Inicia MoveIt y RViz.
2. Carga el entorno de trabajo.
3. Lleva el robot a la configuración Pre-Pick.
4. Espera a que el robot termine completamente el movimiento.

Después ejecuta:

```bash
cd ~/ws_kuka_kr10
source install/setup.bash

ros2 run kuka_kr10_tasks cartesian_path \
  pre_pick \
  pick \
  0.200 \
  0.300
```

Los últimos dos argumentos representan:

```text
0.200 → velocidad cartesiana máxima en m/s
0.300 → aceleración cartesiana máxima en m/s²
```

El nodo calcula automáticamente para los perfiles cúbico y quíntico:

- Distancia cartesiana entre Pre-Pick y Pick.
- Duración del perfil.
- Waypoints cartesianos.
- Fracción calculada por `computeCartesianPath()`.
- Cantidad de puntos articulares.
- Velocidad articular máxima.
- Aceleración articular máxima.
- Coste acumulado de aceleración.
- Coste de jerk.
- Cumplimiento de límites articulares de velocidad.

La salida presenta las métricas de ambos perfiles de forma independiente.

Un resultado típico tiene la siguiente estructura:

```text
Métricas del perfil cúbico

Duración: ...
Velocidad máxima: ...
Aceleración máxima: ...
Coste de aceleración: ...
Coste de jerk: ...

Métricas del perfil quíntico

Duración: ...
Velocidad máxima: ...
Aceleración máxima: ...
Coste de aceleración: ...
Coste de jerk: ...
```


En este proyecto, el perfil quíntico presentó una menor aceleración articular máxima. Sin embargo, el perfil cúbico presentó un menor coste acumulado de aceleración y un menor coste de jerk.

Debido a que el acercamiento fino requiere un movimiento articular globalmente suave, se seleccionó el perfil cúbico para el tramo Pre-Pick → Pick.

El mismo tipo de perfil se reutiliza en el tramo Pre-Place → Place. La duración se calcula nuevamente de forma automática según la distancia cartesiana del segundo tramo.

---

# 7. Ciclo Completo Pick and Place

Una vez evaluados los planeadores y verificadas las trayectorias cartesianas, es posible ejecutar la secuencia completa de manipulación.

Antes de comenzar, asegúrate de que:

- MoveIt y RViz estén activos.
- El entorno de trabajo esté cargado.
- El robot se encuentre en la configuración Home.
- La pieza `part` se encuentre sobre la mesa de Pick.

Ejecuta:

```bash
cd ~/ws_kuka_kr10
source install/setup.bash

ros2 run kuka_kr10_tasks pick_place_cycle
```

Este nodo ejecuta automáticamente la secuencia:

```text
Home
  ↓
Pre-Pick
  ↓
Pick
  ↓
Pre-Place
  ↓
Place
```

Los tramos del ciclo son:

```text
4A: Home → Pre-Pick
    Movimiento libre con RRTConnect.

4B: Pre-Pick → Pick
    Movimiento cartesiano recto con perfil cúbico.

Toma de la pieza
    La pieza se adjunta a tool0.

4C: Pick → Pre-Place
    Movimiento libre con RRTConnect y la pieza adjunta.

4D: Pre-Place → Place
    Movimiento cartesiano recto con perfil cúbico.

Liberación de la pieza
    La pieza se desadjunta en Place.
```

Los movimientos libres utilizan RRTConnect. Debido a que se trata de un planeador probabilístico, el nodo realiza hasta diez intentos independientes y ejecuta el primer plan válido.

Los movimientos cartesianos calculan automáticamente:

- Distancia del tramo.
- Duración del perfil cúbico.
- Waypoints.
- Trayectoria articular.
- Velocidades articulares.
- Aceleraciones articulares.

---

# Estructura del Proyecto

```text
ws_kuka_kr10/
├── src/
│   ├── kuka_kr10_support/
│   ├── kuka_kr10_moveit_config/
│   └── kuka_kr10_tasks/
├── .gitignore
└── README.md
```

Los directorios:

```text
build/
install/
log/
```

se generan localmente al ejecutar:

```bash
colcon build
```

y no se almacenan en el repositorio.

