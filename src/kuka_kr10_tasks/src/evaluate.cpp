/*
 * ============================================================================
 * NODO: evaluate
 * ============================================================================
 *
 * OBJETIVO
 * --------
 *
 * Evaluar una planificación correspondiente al tramo 4A:
 *
 * Home -> pre_pick
 *
 * El nodo utiliza:
 *
 * Grupo de planificación:
 *   manipulator
 *
 * Pipeline:
 *   OMPL
 *
 * Planeador:
 *   RRTConnectkConfigDefault
 *
 * Estado inicial:
 *   Home
 *
 * Estado objetivo:
 *   pre_pick
 *
 * IMPORTANTE
 * ----------
 *
 * Los nombres "Home" y "pre_pick" deben coincidir exactamente con los nombres
 * almacenados en el SRDF.
 *
 * Los nombres son sensibles a mayúsculas y minúsculas:
 *
 * Home != HOME
 *
 * Este nodo:
 *
 * - Construye explícitamente el estado inicial Home.
 * - Establece pre_pick como objetivo.
 * - Solicita una sola planificación.
 * - NO ejecuta la trayectoria.
 * - NO mueve el robot.
 * - Calcula métricas básicas de la trayectoria.
 *
 * Métricas calculadas:
 *
 * - Éxito o fallo.
 * - Tiempo informado por MoveIt.
 * - Tiempo total de reloj.
 * - Número de puntos de la trayectoria.
 * - Duración temporal de la trayectoria.
 * - Longitud del camino en espacio articular.
 * - Suavidad geométrica mediante segunda diferencia.
 *
 * Flujo:
 *
 * SRDF
 *   |
 *   | estados Home y pre_pick
 *   v
 * nodo evaluate
 *   |
 *   | MotionPlanRequest
 *   v
 * move_group
 *   |
 *   | OMPL + RRTConnect
 *   v
 * trayectoria articular
 *   |
 *   v
 * cálculo de métricas
 *
 * ============================================================================
 */


/*
 * ============================================================================
 * LIBRERÍAS ESTÁNDAR DE C++
 * ============================================================================
 */

/*
 * <chrono>
 *
 * Permite medir intervalos de tiempo.
 *
 * Se utiliza para medir el tiempo total alrededor de la llamada plan().
 */
#include <chrono>

/*
 * <cmath>
 *
 * Proporciona funciones matemáticas.
 *
 * Se utiliza para calcular la raíz cuadrada en la longitud articular.
 */
#include <cmath>

/*
 * <memory>
 *
 * Permite utilizar punteros inteligentes.
 *
 * Se utiliza para crear el nodo con std::make_shared.
 */
#include <memory>

/*
 * <string>
 *
 * Permite manejar cadenas de caracteres.
 */
#include <string>

/*
 * <thread>
 *
 * Permite crear el hilo utilizado por el executor.
 */
#include <thread>

/*
 * <vector>
 *
 * Permite trabajar con vectores y colecciones dinámicas.
 */
#include <vector>


/*
 * ============================================================================
 * LIBRERÍAS DE ROS 2
 * ============================================================================
 */

/*
 * Biblioteca principal de ROS 2 para C++.
 */
#include <rclcpp/rclcpp.hpp>

/*
 * Tipo de duración utilizado por time_from_start.
 */
#include <builtin_interfaces/msg/duration.hpp>

/*
 * Tipo de trayectoria articular utilizado por MoveIt.
 */
#include <trajectory_msgs/msg/joint_trajectory.hpp>


/*
 * ============================================================================
 * LIBRERÍAS DE MOVEIT 2
 * ============================================================================
 */

/*
 * MoveGroupInterface proporciona la interfaz para:
 *
 * - Seleccionar un grupo de planificación.
 * - Establecer el estado inicial.
 * - Establecer el estado objetivo.
 * - Seleccionar el pipeline.
 * - Seleccionar el planeador.
 * - Solicitar una planificación.
 */
#include <moveit/move_group_interface/move_group_interface.hpp>

/*
 * RobotState representa una configuración completa del robot.
 *
 * Se utiliza para crear explícitamente el estado inicial Home.
 */
#include <moveit/robot_state/robot_state.hpp>


/*
 * ============================================================================
 * FUNCIÓN: calculateJointPathLength
 * ============================================================================
 *
 * Calcula la longitud total de la trayectoria en espacio articular.
 *
 * Para cada par de puntos consecutivos:
 *
 * delta_q_k = q_(k+1) - q_k
 *
 * La longitud de cada segmento se calcula mediante:
 *
 * ||delta_q_k|| =
 *
 * sqrt(
 *   delta_q_1^2 +
 *   delta_q_2^2 +
 *   ...
 *   delta_q_6^2
 * )
 *
 * La longitud total es:
 *
 * L_q = suma de todas las longitudes de segmento
 *
 * Esta métrica indica cuánto se desplazan conjuntamente las articulaciones.
 *
 * RETORNO
 * -------
 *
 * Longitud total en el espacio articular.
 */
double calculateJointPathLength(
  const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  /*
   * Se necesitan al menos dos puntos para formar un segmento.
   */
  if (trajectory.points.size() < 2)
  {
    return 0.0;
  }


  /*
   * Acumulador de la longitud total.
   */
  double total_length = 0.0;


  /*
   * Recorremos todos los pares de puntos consecutivos.
   */
  for (
    std::size_t point_index = 0;
    point_index + 1 < trajectory.points.size();
    ++point_index)
  {
    /*
     * Punto actual q_k.
     */
    const auto& current_point =
      trajectory.points[point_index];


    /*
     * Punto siguiente q_(k+1).
     */
    const auto& next_point =
      trajectory.points[point_index + 1];


    /*
     * Acumulador de la distancia al cuadrado del segmento.
     */
    double squared_segment_length = 0.0;


    /*
     * Se comprueba que ambos puntos tengan la misma cantidad
     * de posiciones articulares.
     */
    if (
      current_point.positions.size()
      != next_point.positions.size())
    {
      /*
       * Si las dimensiones no coinciden, no se puede calcular
       * correctamente la longitud.
       */
      return -1.0;
    }


    /*
     * Se calcula la diferencia para cada articulación.
     */
    for (
      std::size_t joint_index = 0;
      joint_index < current_point.positions.size();
      ++joint_index)
    {
      /*
       * Cambio de la articulación j entre dos puntos.
       */
      const double delta_q =
        next_point.positions[joint_index]
        - current_point.positions[joint_index];


      /*
       * Se suma el cuadrado de la diferencia.
       */
      squared_segment_length +=
        delta_q * delta_q;
    }


    /*
     * La raíz cuadrada entrega la norma euclidiana
     * del segmento en espacio articular.
     */
    const double segment_length =
      std::sqrt(squared_segment_length);


    /*
     * Se acumula la longitud del segmento.
     */
    total_length += segment_length;
  }


  /*
   * Se devuelve la longitud total.
   */
  return total_length;
}


/*
 * ============================================================================
 * FUNCIÓN: calculateGeometricSmoothness
 * ============================================================================
 *
 * Calcula una medida geométrica de suavidad mediante la segunda diferencia
 * discreta de las posiciones articulares.
 *
 * Para tres puntos consecutivos:
 *
 * q_(k-1), q_k, q_(k+1)
 *
 * se calcula:
 *
 * delta2_q_k =
 *
 * q_(k+1) - 2*q_k + q_(k-1)
 *
 * Si los tres puntos mantienen aproximadamente la misma dirección,
 * la segunda diferencia será pequeña.
 *
 * Si el camino cambia bruscamente de dirección, la segunda diferencia
 * será mayor.
 *
 * La métrica total es:
 *
 * S = suma ||delta2_q_k||^2
 *
 * INTERPRETACIÓN
 * --------------
 *
 * Menor valor:
 *   Geometría articular menos quebrada.
 *
 * Mayor valor:
 *   Cambios de dirección más marcados.
 *
 * LIMITACIÓN
 * ----------
 *
 * Esta medida depende de la cantidad y distribución de los puntos.
 *
 * Por ahora se utiliza como una primera descripción del camino.
 */
double calculateGeometricSmoothness(
  const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  /*
   * Se necesitan al menos tres puntos para calcular
   * una segunda diferencia.
   */
  if (trajectory.points.size() < 3)
  {
    return 0.0;
  }


  /*
   * Acumulador de la medida de suavidad.
   */
  double smoothness = 0.0;


  /*
   * Se recorren únicamente los puntos que tienen:
   *
   * - Un punto anterior.
   * - Un punto posterior.
   */
  for (
    std::size_t point_index = 1;
    point_index + 1 < trajectory.points.size();
    ++point_index)
  {
    /*
     * Punto anterior.
     */
    const auto& previous_point =
      trajectory.points[point_index - 1];


    /*
     * Punto actual.
     */
    const auto& current_point =
      trajectory.points[point_index];


    /*
     * Punto siguiente.
     */
    const auto& next_point =
      trajectory.points[point_index + 1];


    /*
     * Verificamos que los tres puntos tengan la misma dimensión.
     */
    if (
      previous_point.positions.size()
        != current_point.positions.size()
      ||
      current_point.positions.size()
        != next_point.positions.size())
    {
      return -1.0;
    }


    /*
     * Acumulador de la norma cuadrada de la segunda diferencia.
     */
    double squared_second_difference = 0.0;


    /*
     * Se calcula la segunda diferencia para cada articulación.
     */
    for (
      std::size_t joint_index = 0;
      joint_index < current_point.positions.size();
      ++joint_index)
    {
      /*
       * Segunda diferencia discreta:
       *
       * q_(k+1) - 2*q_k + q_(k-1)
       */
      const double second_difference =
        next_point.positions[joint_index]
        - 2.0 * current_point.positions[joint_index]
        + previous_point.positions[joint_index];


      /*
       * Se acumula el cuadrado de la segunda diferencia.
       */
      squared_second_difference +=
        second_difference * second_difference;
    }


    /*
     * Se suma la contribución de este punto.
     */
    smoothness += squared_second_difference;
  }


  /*
   * Se retorna la medida total.
   */
  return smoothness;
}


/*
 * ============================================================================
 * FUNCIÓN: durationToSeconds
 * ============================================================================
 *
 * Convierte una duración ROS 2:
 *
 * builtin_interfaces::msg::Duration
 *
 * a un número decimal de segundos.
 *
 * Una duración ROS contiene:
 *
 * sec:
 *   Parte entera en segundos.
 *
 * nanosec:
 *   Parte fraccionaria en nanosegundos.
 *
 * Conversión:
 *
 * segundos = sec + nanosec * 10^-9
 */
double durationToSeconds(
  const builtin_interfaces::msg::Duration& duration)
{
  return
    static_cast<double>(duration.sec)
    +
    static_cast<double>(duration.nanosec)
    * 1e-9;
}


/*
 * ============================================================================
 * FUNCIÓN PRINCIPAL
 * ============================================================================
 */

int main(int argc, char* argv[])
{
  /*
   * --------------------------------------------------------------------------
   * INICIALIZACIÓN DE ROS 2
   * --------------------------------------------------------------------------
   */
  rclcpp::init(argc, argv);


  /*
   * --------------------------------------------------------------------------
   * CREACIÓN DEL NODO
   * --------------------------------------------------------------------------
   *
   * El nodo se llama:
   *
   * /evaluate
   *
   * automatically_declare_parameters_from_overrides permite aceptar
   * parámetros proporcionados externamente.
   */
  auto node = std::make_shared<rclcpp::Node>(
    "evaluate",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
  );


  /*
   * Se obtiene el logger del nodo.
   */
  auto logger = node->get_logger();


  /*
   * --------------------------------------------------------------------------
   * EXECUTOR
   * --------------------------------------------------------------------------
   *
   * MoveGroupInterface utiliza:
   *
   * - Acciones.
   * - Servicios.
   * - Suscripciones.
   *
   * El executor procesa las respuestas que llegan desde move_group.
   */
  rclcpp::executors::SingleThreadedExecutor executor;


  /*
   * Se añade el nodo al executor.
   */
  executor.add_node(node);


  /*
   * Se crea un hilo independiente para mantener activo el executor.
   *
   * La función principal puede continuar solicitando la planificación
   * mientras el executor procesa respuestas de ROS 2.
   */
  std::thread executor_thread(
    [&executor]()
    {
      executor.spin();
    }
  );


  /*
   * --------------------------------------------------------------------------
   * CREACIÓN DE MOVE GROUP INTERFACE
   * --------------------------------------------------------------------------
   *
   * Se crea la interfaz para el grupo:
   *
   * manipulator
   *
   * Este grupo debe existir en el SRDF.
   */
  moveit::planning_interface::MoveGroupInterface move_group(
    node,
    "manipulator"
  );


  /*
   * --------------------------------------------------------------------------
   * INFORMACIÓN DEL MODELO
   * --------------------------------------------------------------------------
   */

  RCLCPP_INFO(
    logger,
    "Grupo de planificación: %s",
    move_group.getName().c_str()
  );


  RCLCPP_INFO(
    logger,
    "Frame de planificación: %s",
    move_group.getPlanningFrame().c_str()
  );


  RCLCPP_INFO(
    logger,
    "Efector final: %s",
    move_group.getEndEffectorLink().c_str()
  );


  /*
   * --------------------------------------------------------------------------
   * OBTENER LOS ESTADOS NOMBRADOS
   * --------------------------------------------------------------------------
   *
   * Los estados nombrados vienen del SRDF.
   */
  const auto named_targets =
    move_group.getNamedTargets();


  /*
   * Variables que indican si encontramos los estados requeridos.
   */
  bool home_exists = false;
  bool pre_pick_exists = false;


  RCLCPP_INFO(
    logger,
    "Estados nombrados disponibles:"
  );


  /*
   * Se recorren todos los estados cargados.
   */
  for (const auto& target_name : named_targets)
  {
    RCLCPP_INFO(
      logger,
      "  - %s",
      target_name.c_str()
    );


    /*
     * Se busca exactamente "Home".
     *
     * La comparación distingue mayúsculas y minúsculas.
     */
    if (target_name == "Home")
    {
      home_exists = true;
    }


    /*
     * Se busca exactamente "pre_pick".
     */
    if (target_name == "pre_pick")
    {
      pre_pick_exists = true;
    }
  }


  /*
   * Si falta alguno de los dos estados, el experimento no puede continuar.
   */
  if (!home_exists || !pre_pick_exists)
  {
    RCLCPP_ERROR(
      logger,
      "No existen los estados Home y pre_pick en el SRDF"
    );


    /*
     * Se detiene el executor.
     */
    executor.cancel();


    /*
     * Se espera a que termine el hilo.
     */
    executor_thread.join();


    /*
     * Se cierra ROS 2.
     */
    rclcpp::shutdown();


    /*
     * Código 1:
     *
     * Error de configuración.
     */
    return 1;
  }


  /*
   * --------------------------------------------------------------------------
   * OBTENER LOS VALORES DE Home
   * --------------------------------------------------------------------------
   *
   * getNamedTargetValues devuelve un mapa:
   *
   * nombre_articulación -> valor
   *
   * Ejemplo:
   *
   * joint_a1 -> valor_a1
   * joint_a2 -> valor_a2
   */
  const auto home_values =
    move_group.getNamedTargetValues("Home");


  /*
   * --------------------------------------------------------------------------
   * CREAR EL ESTADO INICIAL
   * --------------------------------------------------------------------------
   *
   * RobotState necesita el modelo completo del robot.
   */
  moveit::core::RobotState home_state(
    move_group.getRobotModel()
  );


  /*
   * Se inicializan todas las variables articulares con valores por defecto.
   */
  home_state.setToDefaultValues();


  /*
   * Se asignan los valores articulares guardados en el estado Home.
   */
  home_state.setVariablePositions(
    home_values
  );


  /*
   * Se actualizan las transformaciones internas del RobotState.
   */
  home_state.update();


  /*
   * --------------------------------------------------------------------------
   * FIJAR Home COMO ESTADO INICIAL
   * --------------------------------------------------------------------------
   *
   * Esto hace que el problema matemático siempre comience en Home,
   * independientemente de la postura actual mostrada en RViz.
   */
  move_group.setStartState(
    home_state
  );


  /*
   * --------------------------------------------------------------------------
   * FIJAR pre_pick COMO OBJETIVO
   * --------------------------------------------------------------------------
   *
   * Se utiliza directamente el estado articular guardado en el SRDF.
   *
   * Como pre_pick ya contiene valores articulares, no es necesario resolver
   * cinemática inversa en este ensayo.
   */
  const bool target_success =
    move_group.setNamedTarget("pre_pick");


  /*
   * Si el objetivo no pudo establecerse, se termina el programa.
   */
  if (!target_success)
  {
    RCLCPP_ERROR(
      logger,
      "No fue posible establecer pre_pick como objetivo"
    );

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();

    return 1;
  }


  /*
   * --------------------------------------------------------------------------
   * SELECCIONAR EL PIPELINE
   * --------------------------------------------------------------------------
   *
   * Se selecciona explícitamente OMPL.
   */
  move_group.setPlanningPipelineId(
    "ompl"
  );


  /*
   * --------------------------------------------------------------------------
   * SELECCIONAR EL PLANEADOR
   * --------------------------------------------------------------------------
   *
   * Se selecciona explícitamente RRTConnect.
   *
   * El identificador debe existir en la configuración de OMPL.
   */
  move_group.setPlannerId(
    "RRTsta*kConfigDefault"
  );


  /*
   * --------------------------------------------------------------------------
   * TIEMPO MÁXIMO DE PLANIFICACIÓN
   * --------------------------------------------------------------------------
   *
   * RRTConnect dispone de un máximo de cinco segundos para encontrar
   * una trayectoria.
   */
  move_group.setPlanningTime(
    5.0
  );


  /*
   * --------------------------------------------------------------------------
   * NÚMERO DE INTENTOS INTERNOS
   * --------------------------------------------------------------------------
   *
   * Se configura un solo intento.
   *
   * Así:
   *
   * una llamada a plan() = un ensayo experimental
   */
  move_group.setNumPlanningAttempts(
    1
  );


  /*
   * --------------------------------------------------------------------------
   * ESCALADOS DE VELOCIDAD Y ACELERACIÓN
   * --------------------------------------------------------------------------
   *
   * Estos factores no determinan la geometría principal del camino de OMPL.
   *
   * Se utilizan durante la parametrización temporal de la trayectoria.
   */
  move_group.setMaxVelocityScalingFactor(
    0.20
  );


  move_group.setMaxAccelerationScalingFactor(
    0.20
  );


  /*
   * --------------------------------------------------------------------------
   * MOSTRAR LA CONFIGURACIÓN DEL ENSAYO
   * --------------------------------------------------------------------------
   */

  RCLCPP_INFO(
    logger,
    "Inicio del ensayo"
  );


  RCLCPP_INFO(
    logger,
    "Estado inicial: Home"
  );


  RCLCPP_INFO(
    logger,
    "Estado objetivo: pre_pick"
  );


  RCLCPP_INFO(
    logger,
    "Pipeline: %s",
    move_group.getPlanningPipelineId().c_str()
  );


  RCLCPP_INFO(
    logger,
    "Planeador: %s",
    move_group.getPlannerId().c_str()
  );


  RCLCPP_INFO(
    logger,
    "Tiempo máximo de planificación: %.3f s",
    move_group.getPlanningTime()
  );


  /*
   * --------------------------------------------------------------------------
   * CREAR EL CONTENEDOR DEL PLAN
   * --------------------------------------------------------------------------
   *
   * La estructura Plan almacenará:
   *
   * - La trayectoria.
   * - El estado inicial.
   * - El tiempo informado por MoveIt.
   */
  moveit::planning_interface::MoveGroupInterface::Plan plan;


  /*
   * --------------------------------------------------------------------------
   * INICIAR MEDICIÓN DEL TIEMPO DE RELOJ
   * --------------------------------------------------------------------------
   */
  const auto wall_start =
    std::chrono::steady_clock::now();


  /*
   * --------------------------------------------------------------------------
   * SOLICITAR LA PLANIFICACIÓN
   * --------------------------------------------------------------------------
   *
   * Esta llamada envía la solicitud a move_group.
   *
   * IMPORTANTE:
   *
   * plan() no ejecuta la trayectoria.
   */
  const auto planning_result =
    move_group.plan(plan);


  /*
   * --------------------------------------------------------------------------
   * FINALIZAR MEDICIÓN DEL TIEMPO DE RELOJ
   * --------------------------------------------------------------------------
   */
  const auto wall_end =
    std::chrono::steady_clock::now();


  /*
   * Se convierte el intervalo medido a segundos.
   */
  const double wall_time =
    std::chrono::duration<double>(
      wall_end - wall_start
    ).count();


  /*
   * El código de resultado de MoveIt se convierte a bool.
   */
  const bool planning_success =
    static_cast<bool>(planning_result);


  /*
   * --------------------------------------------------------------------------
   * PROCESAR UN FALLO
   * --------------------------------------------------------------------------
   */
  if (!planning_success)
  {
    RCLCPP_ERROR(
      logger,
      "RTstar no encontró una trayectoria"
    );


    RCLCPP_INFO(
      logger,
      "Tiempo total de reloj: %.6f s",
      wall_time
    );


    RCLCPP_INFO(
      logger,
      "La trayectoria NO fue ejecutada"
    );


    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();


    /*
     * Código 2:
     *
     * Configuración correcta, pero planificación fallida.
     */
    return 2;
  }


  /*
   * --------------------------------------------------------------------------
   * EXTRAER LA TRAYECTORIA ARTICULAR
   * --------------------------------------------------------------------------
   *
   * En MoveIt 2 Jazzy, el campo correcto es:
   *
   * plan.trajectory
   *
   * No se utiliza:
   *
   * plan.trajectory_
   */
  const auto& joint_trajectory =
    plan.trajectory.joint_trajectory;


  /*
   * Cantidad de configuraciones articulares contenidas en el plan.
   */
  const std::size_t number_of_points =
    joint_trajectory.points.size();


  /*
   * Calcular la longitud en espacio articular.
   */
  const double joint_path_length =
    calculateJointPathLength(
      joint_trajectory
    );


  /*
   * Calcular la suavidad geométrica.
   */
  const double geometric_smoothness =
    calculateGeometricSmoothness(
      joint_trajectory
    );


  /*
   * --------------------------------------------------------------------------
   * DURACIÓN PLANIFICADA DE LA TRAYECTORIA
   * --------------------------------------------------------------------------
   *
   * Los puntos contienen un campo time_from_start.
   *
   * La duración total corresponde al time_from_start del último punto.
   */
  double planned_duration = 0.0;


  if (!joint_trajectory.points.empty())
  {
    planned_duration =
      durationToSeconds(
        joint_trajectory.points.back().time_from_start
      );
  }


  /*
   * --------------------------------------------------------------------------
   * MOSTRAR LOS RESULTADOS
   * --------------------------------------------------------------------------
   */

  RCLCPP_INFO(
    logger,
    "Planificación exitosa"
  );


  /*
   * En MoveIt 2 Jazzy, el campo correcto es:
   *
   * plan.planning_time
   *
   * No se utiliza:
   *
   * plan.planning_time_
   */
  RCLCPP_INFO(
    logger,
    "Tiempo informado por MoveIt: %.6f s",
    plan.planning_time
  );


  /*
   * Tiempo total alrededor de la llamada plan().
   */
  RCLCPP_INFO(
    logger,
    "Tiempo total de reloj: %.6f s",
    wall_time
  );


  /*
   * Cantidad de puntos articulares.
   */
  RCLCPP_INFO(
    logger,
    "Número de puntos: %zu",
    number_of_points
  );


  /*
   * Duración que tendría la trayectoria si fuera ejecutada.
   */
  RCLCPP_INFO(
    logger,
    "Duración planificada: %.6f s",
    planned_duration
  );


  /*
   * Longitud acumulada del camino articular.
   */
  RCLCPP_INFO(
    logger,
    "Longitud articular: %.6f rad",
    joint_path_length
  );


  /*
   * Medida inicial de suavidad geométrica.
   */
  RCLCPP_INFO(
    logger,
    "Suavidad geométrica: %.12f",
    geometric_smoothness
  );


  /*
   * Confirmación de que solo se realizó planificación.
   */
  RCLCPP_INFO(
    logger,
    "La trayectoria NO fue ejecutada"
  );


  /*
   * --------------------------------------------------------------------------
   * CERRAR EL EXECUTOR
   * --------------------------------------------------------------------------
   */
  executor.cancel();


  /*
   * Se espera a que termine el hilo.
   */
  executor_thread.join();


  /*
   * Se cierra ROS 2 correctamente.
   */
  rclcpp::shutdown();


  /*
   * Código 0:
   *
   * Planificación exitosa.
   */
  return 0;
}
