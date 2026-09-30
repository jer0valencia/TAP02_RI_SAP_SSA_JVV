/*
 * ============================================================================
 * NODO: cartesian_path
 * ============================================================================
 *
 * Uso:
 *
 * ros2 run kuka_kr10_tasks cartesian_path \
 *   <estado_inicial> \
 *   <estado_final> \
 *   <velocidad_maxima_m_s> \
 *   <aceleracion_maxima_m_s2>
 *
 * Ejemplo 4B:
 *
 * ros2 run kuka_kr10_tasks cartesian_path \
 *   pre_pick pick 0.200 0.300
 *
 * Ejemplo 4D:
 *
 * ros2 run kuka_kr10_tasks cartesian_path \
 *   pre_place place 0.200 0.300
 *
 * El nodo:
 *
 * 1. Lee los estados nombrados del SRDF.
 * 2. Calcula las poses cartesianas mediante cinemática directa.
 * 3. Calcula automáticamente la distancia cartesiana.
 * 4. Genera internamente los perfiles cúbico y quíntico.
 * 5. Genera cuatro puntos intermedios.
 * 6. Calcula los caminos mediante computeCartesianPath().
 * 7. Comprueba que ambos caminos estén completos.
 * 8. Temporiza todos los puntos articulares.
 * 9. Calcula velocidades y aceleraciones articulares.
 * 10. Calcula aceleración máxima, coste de aceleración y jerk.
 * 11. Comprueba los límites articulares disponibles.
 * 12. Verifica que el robot esté en el estado inicial.
 * 13. Ejecuta el perfil cúbico.
 *
 * Esta versión no utiliza archivos CSV para ejecutar.
 *
 * El script Python sigue utilizándose para generar las tablas
 * y gráficas exigidas por el taller.
 *
 * ============================================================================
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_state/conversions.hpp>

#include <moveit_msgs/msg/robot_trajectory.hpp>


/*
 * ============================================================================
 * ESTRUCTURAS
 * ============================================================================
 */

/*
 * Un punto del perfil temporal cartesiano.
 */
struct ProfilePoint
{
  /*
   * Tiempo real, en segundos.
   */
  double time;

  /*
   * Tiempo normalizado entre cero y uno.
   */
  double tau;

  /*
   * Avance normalizado.
   *
   * s = 0 corresponde al inicio.
   * s = 1 corresponde al final.
   */
  double s;

  /*
   * Distancia cartesiana recorrida, en metros.
   */
  double distance;

  /*
   * Velocidad cartesiana, en metros por segundo.
   */
  double velocity;

  /*
   * Aceleración cartesiana con signo, en metros por segundo cuadrado.
   */
  double acceleration;
};


/*
 * Métricas articulares de una trayectoria.
 */
struct TrajectoryMetrics
{
  /*
   * Duración total.
   */
  double duration;

  /*
   * Mayor velocidad articular absoluta.
   */
  double maximum_velocity;

  /*
   * Mayor aceleración articular absoluta.
   */
  double maximum_acceleration;

  /*
   * Aproximación numérica de:
   *
   * integral ||qddot(t)||² dt
   */
  double acceleration_cost;

  /*
   * Aproximación numérica de:
   *
   * integral ||jerk(t)||² dt
   */
  double jerk_cost;
};


/*
 * Resultado de la comprobación de límites.
 */
struct JointLimitCheck
{
  bool valid;

  std::size_t velocity_violations;

  std::size_t acceleration_violations;

  double maximum_velocity_ratio;

  double maximum_acceleration_ratio;
};


/*
 * ============================================================================
 * FUNCIONES DE LOS PERFILES
 * ============================================================================
 */

/*
 * Posición normalizada del perfil cúbico:
 *
 * s(tau) = 3 tau² - 2 tau³
 */
double cubicPosition(
  const double tau)
{
  return
    3.0 * tau * tau
    - 2.0 * tau * tau * tau;
}


/*
 * Derivada normalizada del perfil cúbico:
 *
 * ds/dtau = 6 tau - 6 tau²
 */
double cubicVelocity(
  const double tau)
{
  return
    6.0 * tau
    - 6.0 * tau * tau;
}


/*
 * Segunda derivada normalizada del perfil cúbico:
 *
 * d²s/dtau² = 6 - 12 tau
 */
double cubicAcceleration(
  const double tau)
{
  return
    6.0
    - 12.0 * tau;
}


/*
 * Posición normalizada del perfil quíntico:
 *
 * s(tau) = 10 tau³ - 15 tau⁴ + 6 tau⁵
 */
double quinticPosition(
  const double tau)
{
  return
    10.0 * std::pow(tau, 3)
    - 15.0 * std::pow(tau, 4)
    + 6.0 * std::pow(tau, 5);
}


/*
 * Derivada normalizada del perfil quíntico:
 *
 * ds/dtau = 30 tau² - 60 tau³ + 30 tau⁴
 */
double quinticVelocity(
  const double tau)
{
  return
    30.0 * std::pow(tau, 2)
    - 60.0 * std::pow(tau, 3)
    + 30.0 * std::pow(tau, 4);
}


/*
 * Segunda derivada normalizada del perfil quíntico:
 *
 * d²s/dtau² =
 *
 * 60 tau - 180 tau² + 120 tau³
 */
double quinticAcceleration(
  const double tau)
{
  return
    60.0 * tau
    - 180.0 * std::pow(tau, 2)
    + 120.0 * std::pow(tau, 3);
}


/*
 * ============================================================================
 * GENERACIÓN AUTOMÁTICA DEL PERFIL
 * ============================================================================
 */

std::vector<ProfilePoint> generateProfile(
  const double length,
  const double maximum_velocity,
  const double maximum_acceleration,
  const std::size_t intermediate_points,
  const bool quintic)
{
  if (length <= 0.0)
  {
    throw std::runtime_error(
      "La longitud cartesiana debe ser positiva"
    );
  }

  if (
    maximum_velocity <= 0.0
    ||
    maximum_acceleration <= 0.0)
  {
    throw std::runtime_error(
      "Los límites cartesianos deben ser positivos"
    );
  }


  /*
   * Calcular el tiempo mínimo que cumple simultáneamente:
   *
   * velocidad <= maximum_velocity
   * aceleración <= maximum_acceleration
   */
  double velocity_time = 0.0;

  double acceleration_time = 0.0;


  if (quintic)
  {
    /*
     * Para el perfil quíntico:
     *
     * max(ds/dtau) = 1.875
     *
     * max(|d²s/dtau²|) = 10/sqrt(3)
     */
    velocity_time =
      1.875
      * length
      / maximum_velocity;

    acceleration_time =
      std::sqrt(
        (
          10.0
          / std::sqrt(3.0)
        )
        * length
        / maximum_acceleration
      );
  }
  else
  {
    /*
     * Para el perfil cúbico:
     *
     * max(ds/dtau) = 1.5
     *
     * max(|d²s/dtau²|) = 6
     */
    velocity_time =
      1.5
      * length
      / maximum_velocity;

    acceleration_time =
      std::sqrt(
        6.0
        * length
        / maximum_acceleration
      );
  }


  const double duration =
    std::max(
      velocity_time,
      acceleration_time
    );


  /*
   * Un punto inicial, cuatro intermedios y uno final.
   */
  const std::size_t total_points =
    intermediate_points + 2;


  std::vector<ProfilePoint> profile;

  profile.reserve(total_points);


  for (std::size_t index = 0;
       index < total_points;
       ++index)
  {
    const double tau =
      static_cast<double>(index)
      /
      static_cast<double>(
        total_points - 1
      );


    double s = 0.0;

    double ds_dtau = 0.0;

    double d2s_dtau2 = 0.0;


    if (quintic)
    {
      s =
        quinticPosition(tau);

      ds_dtau =
        quinticVelocity(tau);

      d2s_dtau2 =
        quinticAcceleration(tau);
    }
    else
    {
      s =
        cubicPosition(tau);

      ds_dtau =
        cubicVelocity(tau);

      d2s_dtau2 =
        cubicAcceleration(tau);
    }


    ProfilePoint point;


    point.time =
      tau * duration;


    point.tau =
      tau;


    point.s =
      s;


    point.distance =
      length * s;


    /*
     * ds/dt =
     *
     * ds/dtau / duration
     */
    point.velocity =
      length
      * ds_dtau
      / duration;


    /*
     * d²s/dt² =
     *
     * d²s/dtau² / duration²
     */
    point.acceleration =
      length
      * d2s_dtau2
      / (
          duration
          * duration
        );


    profile.push_back(point);
  }


  /*
   * Forzar los extremos para evitar errores numéricos pequeños.
   */
  profile.front().time =
    0.0;

  profile.front().tau =
    0.0;

  profile.front().s =
    0.0;

  profile.front().distance =
    0.0;

  profile.front().velocity =
    0.0;


  profile.back().time =
    duration;

  profile.back().tau =
    1.0;

  profile.back().s =
    1.0;

  profile.back().distance =
    length;

  profile.back().velocity =
    0.0;


  return profile;
}


/*
 * ============================================================================
 * CINEMÁTICA DIRECTA
 * ============================================================================
 */

geometry_msgs::msg::Pose poseFromRobotState(
  const moveit::core::RobotState& state,
  const std::string& link_name)
{
  const Eigen::Isometry3d& transform =
    state.getGlobalLinkTransform(
      link_name
    );


  geometry_msgs::msg::Pose pose;


  pose.position.x =
    transform.translation().x();

  pose.position.y =
    transform.translation().y();

  pose.position.z =
    transform.translation().z();


  const Eigen::Quaterniond quaternion(
    transform.rotation()
  );


  pose.orientation.x =
    quaternion.x();

  pose.orientation.y =
    quaternion.y();

  pose.orientation.z =
    quaternion.z();

  pose.orientation.w =
    quaternion.w();


  return pose;
}


/*
 * Diferencia entre dos cuaterniones.
 *
 * q y -q representan la misma orientación.
 */
double quaternionDifference(
  const geometry_msgs::msg::Quaternion& first,
  const geometry_msgs::msg::Quaternion& second)
{
  const double direct_difference =
    std::sqrt(
      std::pow(first.x - second.x, 2)
      +
      std::pow(first.y - second.y, 2)
      +
      std::pow(first.z - second.z, 2)
      +
      std::pow(first.w - second.w, 2)
    );


  const double opposite_difference =
    std::sqrt(
      std::pow(first.x + second.x, 2)
      +
      std::pow(first.y + second.y, 2)
      +
      std::pow(first.z + second.z, 2)
      +
      std::pow(first.w + second.w, 2)
    );


  return
    std::min(
      direct_difference,
      opposite_difference
    );
}


/*
 * ============================================================================
 * CONSTRUCCIÓN DE WAYPOINTS
 * ============================================================================
 */

std::vector<geometry_msgs::msg::Pose> buildWaypoints(
  const geometry_msgs::msg::Pose& start_pose,
  const geometry_msgs::msg::Pose& goal_pose,
  const std::vector<ProfilePoint>& profile)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;


  const double delta_x =
    goal_pose.position.x
    - start_pose.position.x;

  const double delta_y =
    goal_pose.position.y
    - start_pose.position.y;

  const double delta_z =
    goal_pose.position.z
    - start_pose.position.z;


  for (const auto& profile_point :
       profile)
  {
    geometry_msgs::msg::Pose waypoint;


    waypoint.position.x =
      start_pose.position.x
      + profile_point.s
      * delta_x;


    waypoint.position.y =
      start_pose.position.y
      + profile_point.s
      * delta_y;


    waypoint.position.z =
      start_pose.position.z
      + profile_point.s
      * delta_z;


    /*
     * Mantener orientación constante.
     */
    waypoint.orientation =
      start_pose.orientation;


    waypoints.push_back(
      waypoint
    );
  }


  return waypoints;
}


/*
 * Mostrar los waypoints.
 */
void printWaypoints(
  const std::string& profile_name,
  const std::vector<ProfilePoint>& profile,
  const std::vector<geometry_msgs::msg::Pose>& waypoints,
  const rclcpp::Logger& logger)
{
  RCLCPP_INFO(
    logger,
    "Waypoints del perfil %s:",
    profile_name.c_str()
  );


  for (std::size_t index = 0;
       index < waypoints.size();
       ++index)
  {
    RCLCPP_INFO(
      logger,
      "  %zu: t=%.6f s, s=%.6f, "
      "p=[%.6f, %.6f, %.6f]",
      index,
      profile[index].time,
      profile[index].s,
      waypoints[index].position.x,
      waypoints[index].position.y,
      waypoints[index].position.z
    );
  }
}


/*
 * ============================================================================
 * CONVERSIONES DE TIEMPO
 * ============================================================================
 */

builtin_interfaces::msg::Duration secondsToDuration(
  const double seconds)
{
  builtin_interfaces::msg::Duration duration;


  const double safe_seconds =
    std::max(
      0.0,
      seconds
    );


  duration.sec =
    static_cast<int32_t>(
      std::floor(
        safe_seconds
      )
    );


  duration.nanosec =
    static_cast<uint32_t>(
      std::round(
        (
          safe_seconds
          - static_cast<double>(
              duration.sec
            )
        )
        * 1e9
      )
    );


  if (
    duration.nanosec
    >= 1000000000u)
  {
    duration.sec += 1;

    duration.nanosec -=
      1000000000u;
  }


  return duration;
}


double durationToSeconds(
  const builtin_interfaces::msg::Duration& duration)
{
  return
    static_cast<double>(
      duration.sec
    )
    +
    static_cast<double>(
      duration.nanosec
    )
    * 1e-9;
}


/*
 * ============================================================================
 * INVERSIÓN DEL PERFIL
 * ============================================================================
 *
 * Obtiene tau para un avance s conocido.
 */

double invertProfile(
  const double target_s,
  const bool quintic)
{
  const double clamped_s =
    std::clamp(
      target_s,
      0.0,
      1.0
    );


  if (clamped_s <= 0.0)
  {
    return 0.0;
  }


  if (clamped_s >= 1.0)
  {
    return 1.0;
  }


  double lower_tau =
    0.0;

  double upper_tau =
    1.0;


  for (int iteration = 0;
       iteration < 60;
       ++iteration)
  {
    const double middle_tau =
      0.5
      * (
          lower_tau
          + upper_tau
        );


    const double middle_s =
      quintic
      ? quinticPosition(middle_tau)
      : cubicPosition(middle_tau);


    if (middle_s < clamped_s)
    {
      lower_tau =
        middle_tau;
    }
    else
    {
      upper_tau =
        middle_tau;
    }
  }


  return
    0.5
    * (
        lower_tau
        + upper_tau
      );
}


/*
 * ============================================================================
 * COMPUTE CARTESIAN PATH
 * ============================================================================
 */

double processProfile(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::RobotState& start_state,
  const std::string& profile_name,
  const std::vector<ProfilePoint>& profile,
  const geometry_msgs::msg::Pose& start_pose,
  const geometry_msgs::msg::Pose& goal_pose,
  moveit_msgs::msg::RobotTrajectory& trajectory,
  const rclcpp::Logger& logger)
{
  /*
   * Fijar explícitamente el estado inicial.
   */
  move_group.setStartState(
    start_state
  );


  const auto waypoints =
    buildWaypoints(
      start_pose,
      goal_pose,
      profile
    );


  printWaypoints(
    profile_name,
    profile,
    waypoints,
    logger
  );


  /*
   * Resolución cartesiana:
   *
   * 5 mm.
   */
  const double eef_step =
    0.005;


  /*
   * true activa evitación de colisiones.
   */
  const double fraction =
    move_group.computeCartesianPath(
      waypoints,
      eef_step,
      trajectory,
      true
    );


  RCLCPP_INFO(
    logger,
    "Perfil %s: fracción = %.2f %%",
    profile_name.c_str(),
    fraction * 100.0
  );


  RCLCPP_INFO(
    logger,
    "Perfil %s: puntos articulares = %zu",
    profile_name.c_str(),
    trajectory
      .joint_trajectory
      .points
      .size()
  );


  return fraction;
}


/*
 * ============================================================================
 * AVANCE CARTESIANO DE CADA PUNTO ARTICULAR
 * ============================================================================
 */

std::vector<double> calculateCartesianProgress(
  const moveit_msgs::msg::RobotTrajectory& trajectory,
  const moveit::core::RobotModelConstPtr& robot_model,
  const geometry_msgs::msg::Pose& start_pose,
  const geometry_msgs::msg::Pose& goal_pose,
  const std::string& end_effector_link)
{
  std::vector<double> progress_values;


  const auto& joint_trajectory =
    trajectory.joint_trajectory;


  const double delta_x =
    goal_pose.position.x
    - start_pose.position.x;

  const double delta_y =
    goal_pose.position.y
    - start_pose.position.y;

  const double delta_z =
    goal_pose.position.z
    - start_pose.position.z;


  const double squared_length =
    delta_x * delta_x
    +
    delta_y * delta_y
    +
    delta_z * delta_z;


  if (squared_length < 1e-12)
  {
    throw std::runtime_error(
      "La distancia cartesiana es cero"
    );
  }


  moveit::core::RobotState state(
    robot_model
  );


  state.setToDefaultValues();


  for (const auto& trajectory_point :
       joint_trajectory.points)
  {
    state.setVariablePositions(
      joint_trajectory.joint_names,
      trajectory_point.positions
    );


    state.update();


    const Eigen::Isometry3d& transform =
      state.getGlobalLinkTransform(
        end_effector_link
      );


    const double point_delta_x =
      transform.translation().x()
      - start_pose.position.x;

    const double point_delta_y =
      transform.translation().y()
      - start_pose.position.y;

    const double point_delta_z =
      transform.translation().z()
      - start_pose.position.z;


    double s =
      (
        point_delta_x * delta_x
        +
        point_delta_y * delta_y
        +
        point_delta_z * delta_z
      )
      / squared_length;


    s =
      std::clamp(
        s,
        0.0,
        1.0
      );


    /*
     * Evitar pequeños retrocesos causados por precisión numérica.
     */
    if (
      !progress_values.empty()
      &&
      s < progress_values.back())
    {
      s =
        progress_values.back();
    }


    progress_values.push_back(
      s
    );
  }


  if (!progress_values.empty())
  {
    progress_values.front() =
      0.0;

    progress_values.back() =
      1.0;
  }


  return progress_values;
}


/*
 * ============================================================================
 * TEMPORIZACIÓN ARTICULAR
 * ============================================================================
 */

void temporalizeTrajectory(
  moveit_msgs::msg::RobotTrajectory& trajectory,
  const moveit::core::RobotModelConstPtr& robot_model,
  const geometry_msgs::msg::Pose& start_pose,
  const geometry_msgs::msg::Pose& goal_pose,
  const std::string& end_effector_link,
  const double profile_duration,
  const bool quintic)
{
  auto& joint_trajectory =
    trajectory.joint_trajectory;


  const std::size_t number_of_points =
    joint_trajectory.points.size();


  if (number_of_points < 3)
  {
    throw std::runtime_error(
      "La trayectoria tiene menos de tres puntos"
    );
  }


  const auto progress_values =
    calculateCartesianProgress(
      trajectory,
      robot_model,
      start_pose,
      goal_pose,
      end_effector_link
    );


  std::vector<double> times(
    number_of_points,
    0.0
  );


  /*
   * Convertir:
   *
   * s -> tau -> tiempo
   */
  for (std::size_t point_index = 0;
       point_index < number_of_points;
       ++point_index)
  {
    const double tau =
      invertProfile(
        progress_values[point_index],
        quintic
      );


    times[point_index] =
      tau * profile_duration;
  }


  times.front() =
    0.0;

  times.back() =
    profile_duration;


  /*
   * Tiempos estrictamente crecientes.
   */
  for (std::size_t point_index = 1;
       point_index < number_of_points;
       ++point_index)
  {
    if (
      times[point_index]
      <= times[point_index - 1])
    {
      times[point_index] =
        times[point_index - 1]
        + 1e-6;
    }
  }


  if (
    times[number_of_points - 2]
    >= profile_duration)
  {
    throw std::runtime_error(
      "No fue posible generar tiempos crecientes"
    );
  }


  times.back() =
    profile_duration;


  /*
   * Asignar time_from_start.
   */
  for (std::size_t point_index = 0;
       point_index < number_of_points;
       ++point_index)
  {
    joint_trajectory
      .points[point_index]
      .time_from_start =
        secondsToDuration(
          times[point_index]
        );
  }


  const std::size_t number_of_joints =
    joint_trajectory
      .points
      .front()
      .positions
      .size();


  /*
   * Inicializar velocidades y aceleraciones.
   */
  for (auto& point :
       joint_trajectory.points)
  {
    point.velocities.assign(
      number_of_joints,
      0.0
    );


    point.accelerations.assign(
      number_of_joints,
      0.0
    );
  }


  /*
   * Velocidades interiores por diferencia central.
   */
  for (std::size_t point_index = 1;
       point_index + 1 < number_of_points;
       ++point_index)
  {
    const double delta_time =
      times[point_index + 1]
      - times[point_index - 1];


    if (delta_time <= 0.0)
    {
      throw std::runtime_error(
        "Intervalo temporal no positivo al calcular velocidad"
      );
    }


    for (std::size_t joint_index = 0;
         joint_index < number_of_joints;
         ++joint_index)
    {
      const double delta_position =
        joint_trajectory
          .points[point_index + 1]
          .positions[joint_index]
        -
        joint_trajectory
          .points[point_index - 1]
          .positions[joint_index];


      joint_trajectory
        .points[point_index]
        .velocities[joint_index] =
          delta_position
          / delta_time;
    }
  }


  /*
   * Reposo al inicio y al final.
   */
  std::fill(
    joint_trajectory
      .points
      .front()
      .velocities
      .begin(),
    joint_trajectory
      .points
      .front()
      .velocities
      .end(),
    0.0
  );


  std::fill(
    joint_trajectory
      .points
      .back()
      .velocities
      .begin(),
    joint_trajectory
      .points
      .back()
      .velocities
      .end(),
    0.0
  );


  /*
   * Aceleraciones interiores.
   */
  for (std::size_t point_index = 1;
       point_index + 1 < number_of_points;
       ++point_index)
  {
    const double delta_time =
      times[point_index + 1]
      - times[point_index - 1];


    if (delta_time <= 0.0)
    {
      throw std::runtime_error(
        "Intervalo temporal no positivo al calcular aceleración"
      );
    }


    for (std::size_t joint_index = 0;
         joint_index < number_of_joints;
         ++joint_index)
    {
      const double delta_velocity =
        joint_trajectory
          .points[point_index + 1]
          .velocities[joint_index]
        -
        joint_trajectory
          .points[point_index - 1]
          .velocities[joint_index];


      joint_trajectory
        .points[point_index]
        .accelerations[joint_index] =
          delta_velocity
          / delta_time;
    }
  }


  /*
   * Aceleración inicial.
   */
  {
    const double delta_time =
      times[1]
      - times[0];


    if (delta_time <= 0.0)
    {
      throw std::runtime_error(
        "Intervalo inicial no positivo"
      );
    }


    for (std::size_t joint_index = 0;
         joint_index < number_of_joints;
         ++joint_index)
    {
      joint_trajectory
        .points[0]
        .accelerations[joint_index] =
          (
            joint_trajectory
              .points[1]
              .velocities[joint_index]
            -
            joint_trajectory
              .points[0]
              .velocities[joint_index]
          )
          / delta_time;
    }
  }


  /*
   * Aceleración final.
   */
  {
    const std::size_t last_index =
      number_of_points - 1;


    const double delta_time =
      times[last_index]
      - times[last_index - 1];


    if (delta_time <= 0.0)
    {
      throw std::runtime_error(
        "Intervalo final no positivo"
      );
    }


    for (std::size_t joint_index = 0;
         joint_index < number_of_joints;
         ++joint_index)
    {
      joint_trajectory
        .points[last_index]
        .accelerations[joint_index] =
          (
            joint_trajectory
              .points[last_index]
              .velocities[joint_index]
            -
            joint_trajectory
              .points[last_index - 1]
              .velocities[joint_index]
          )
          / delta_time;
    }
  }
}


/*
 * ============================================================================
 * MÉTRICAS ARTICULARES
 * ============================================================================
 */

TrajectoryMetrics calculateTrajectoryMetrics(
  const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  const auto& points =
    trajectory.joint_trajectory.points;


  if (points.size() < 2)
  {
    throw std::runtime_error(
      "No hay suficientes puntos para calcular métricas"
    );
  }


  TrajectoryMetrics metrics;


  metrics.duration =
    durationToSeconds(
      points.back().time_from_start
    );


  metrics.maximum_velocity =
    0.0;


  metrics.maximum_acceleration =
    0.0;


  metrics.acceleration_cost =
    0.0;


  metrics.jerk_cost =
    0.0;


  /*
   * Velocidad máxima.
   */
  for (const auto& point :
       points)
  {
    for (const double velocity :
         point.velocities)
    {
      metrics.maximum_velocity =
        std::max(
          metrics.maximum_velocity,
          std::abs(
            velocity
          )
        );
    }
  }


  /*
   * Aceleración y jerk.
   */
  for (std::size_t point_index = 0;
       point_index + 1 < points.size();
       ++point_index)
  {
    const double initial_time =
      durationToSeconds(
        points[point_index]
          .time_from_start
      );


    const double final_time =
      durationToSeconds(
        points[point_index + 1]
          .time_from_start
      );


    const double delta_time =
      final_time
      - initial_time;


    if (delta_time <= 0.0)
    {
      continue;
    }


    double acceleration_norm_squared =
      0.0;


    double jerk_norm_squared =
      0.0;


    const std::size_t number_of_joints =
      points[point_index]
        .accelerations
        .size();


    for (std::size_t joint_index = 0;
         joint_index < number_of_joints;
         ++joint_index)
    {
      const double initial_acceleration =
        points[point_index]
          .accelerations[joint_index];


      const double final_acceleration =
        points[point_index + 1]
          .accelerations[joint_index];


      metrics.maximum_acceleration =
        std::max(
          metrics.maximum_acceleration,
          std::abs(
            initial_acceleration
          )
        );


      metrics.maximum_acceleration =
        std::max(
          metrics.maximum_acceleration,
          std::abs(
            final_acceleration
          )
        );


      acceleration_norm_squared +=
        0.5
        * (
            initial_acceleration
            * initial_acceleration
            +
            final_acceleration
            * final_acceleration
          );


      const double jerk =
        (
          final_acceleration
          - initial_acceleration
        )
        / delta_time;


      jerk_norm_squared +=
        jerk * jerk;
    }


    metrics.acceleration_cost +=
      acceleration_norm_squared
      * delta_time;


    metrics.jerk_cost +=
      jerk_norm_squared
      * delta_time;
  }


  return metrics;
}


/*
 * ============================================================================
 * VERIFICACIÓN BÁSICA DE LÍMITES ARTICULARES
 * ============================================================================
 */

bool checkVelocityLimits(
  const moveit_msgs::msg::RobotTrajectory& trajectory,
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::string& profile_name,
  const rclcpp::Logger& logger)
{
  const auto& joint_trajectory =
    trajectory.joint_trajectory;


  bool valid =
    true;


  RCLCPP_INFO(
    logger,
    "Verificación de velocidades: perfil %s",
    profile_name.c_str()
  );


  for (std::size_t joint_index = 0;
       joint_index < joint_trajectory.joint_names.size();
       ++joint_index)
  {
    const std::string& joint_name =
      joint_trajectory
        .joint_names[joint_index];


    const auto& bounds =
      robot_model->getVariableBounds(
        joint_name
      );


    double observed_maximum =
      0.0;


    for (const auto& point :
         joint_trajectory.points)
    {
      if (
        joint_index
        < point.velocities.size())
      {
        observed_maximum =
          std::max(
            observed_maximum,
            std::abs(
              point.velocities[joint_index]
            )
          );
      }
    }


    if (
      bounds.velocity_bounded_
      &&
      bounds.max_velocity_ > 0.0)
    {
      const double limit =
        std::max(
          std::abs(
            bounds.min_velocity_
          ),
          std::abs(
            bounds.max_velocity_
          )
        );


      const double percentage =
        100.0
        * observed_maximum
        / limit;


      RCLCPP_INFO(
        logger,
        "%s: %.6f / %.6f rad/s (%.2f %%)",
        joint_name.c_str(),
        observed_maximum,
        limit,
        percentage
      );


      if (observed_maximum > 1.001 * limit)
      {
        valid =
          false;


        RCLCPP_ERROR(
          logger,
          "%s supera el límite de velocidad",
          joint_name.c_str()
        );
      }
    }
    else
    {
      RCLCPP_WARN(
        logger,
        "%s no tiene límite de velocidad activo",
        joint_name.c_str()
      );
    }
  }


  return valid;
}


/*
 * ============================================================================
 * VERIFICAR ESTADO ACTUAL
 * ============================================================================
 */

bool currentStateMatchesStart(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::RobotState& expected_start_state,
  const double tolerance,
  const rclcpp::Logger& logger)
{
  const auto current_state =
    move_group.getCurrentState(
      2.0
    );


  if (!current_state)
  {
    RCLCPP_ERROR(
      logger,
      "No fue posible obtener el estado actual"
    );


    return false;
  }


  const auto joint_names =
    move_group.getJointNames();


  bool matches =
    true;


  RCLCPP_INFO(
    logger,
    "Comprobación del estado inicial:"
  );


  for (const auto& joint_name :
       joint_names)
  {
    const double current_value =
      current_state
        ->getVariablePosition(
          joint_name
        );


    const double expected_value =
      expected_start_state
        .getVariablePosition(
          joint_name
        );


    const double difference =
      std::abs(
        current_value
        - expected_value
      );


    RCLCPP_INFO(
      logger,
      "%s: actual=%.6f, esperado=%.6f, "
      "diferencia=%.6f rad",
      joint_name.c_str(),
      current_value,
      expected_value,
      difference
    );


    if (difference > tolerance)
    {
      matches =
        false;
    }
  }


  return matches;
}


/*
 * ============================================================================
 * MAIN
 * ============================================================================
 */

int main(int argc, char* argv[])
{
  rclcpp::init(
    argc,
    argv
  );


  /*
   * Argumentos esperados:
   *
   * argv[1] = estado inicial
   * argv[2] = estado final
   * argv[3] = velocidad máxima cartesiana
   * argv[4] = aceleración máxima cartesiana
   */
  if (argc < 5)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "cartesian_path"
      ),
      "Uso: cartesian_path "
      "<estado_inicial> "
      "<estado_final> "
      "<velocidad_maxima_m_s> "
      "<aceleracion_maxima_m_s2>"
    );


    rclcpp::shutdown();


    return 1;
  }


  const std::string start_state_name =
    argv[1];


  const std::string goal_state_name =
    argv[2];


  double maximum_cartesian_velocity =
    0.0;


  double maximum_cartesian_acceleration =
    0.0;


  try
  {
    maximum_cartesian_velocity =
      std::stod(
        argv[3]
      );


    maximum_cartesian_acceleration =
      std::stod(
        argv[4]
      );
  }
  catch (const std::exception&)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "cartesian_path"
      ),
      "La velocidad y la aceleración deben ser números"
    );


    rclcpp::shutdown();


    return 1;
  }


  if (
    maximum_cartesian_velocity <= 0.0
    ||
    maximum_cartesian_acceleration <= 0.0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "cartesian_path"
      ),
      "La velocidad y la aceleración deben ser positivas"
    );


    rclcpp::shutdown();


    return 1;
  }


  auto node =
    std::make_shared<rclcpp::Node>(
      "cartesian_path",
      rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(
          true
        )
    );


  const auto logger =
    node->get_logger();


  rclcpp::executors::SingleThreadedExecutor
    executor;


  executor.add_node(
    node
  );


  std::thread executor_thread(
    [&executor]()
    {
      executor.spin();
    }
  );


  moveit::planning_interface::MoveGroupInterface
    move_group(
      node,
      "manipulator"
    );


  /*
   * Verificar estados nombrados.
   */
  const auto named_targets =
    move_group.getNamedTargets();


  bool start_exists =
    false;


  bool goal_exists =
    false;


  for (const auto& target_name :
       named_targets)
  {
    if (target_name == start_state_name)
    {
      start_exists =
        true;
    }


    if (target_name == goal_state_name)
    {
      goal_exists =
        true;
    }
  }


  if (!start_exists || !goal_exists)
  {
    RCLCPP_ERROR(
      logger,
      "No existen los estados %s y %s en el SRDF",
      start_state_name.c_str(),
      goal_state_name.c_str()
    );


    executor.cancel();

    executor_thread.join();

    rclcpp::shutdown();


    return 1;
  }


  /*
   * Leer estados articulares desde el SRDF.
   */
  const auto start_values =
    move_group.getNamedTargetValues(
      start_state_name
    );


  const auto goal_values =
    move_group.getNamedTargetValues(
      goal_state_name
    );


  moveit::core::RobotState start_state(
    move_group.getRobotModel()
  );


  moveit::core::RobotState goal_state(
    move_group.getRobotModel()
  );


  start_state.setToDefaultValues();

  goal_state.setToDefaultValues();


  start_state.setVariablePositions(
    start_values
  );


  goal_state.setVariablePositions(
    goal_values
  );


  start_state.update();

  goal_state.update();


  const std::string end_effector_link =
    "tool0";


  const auto start_pose =
    poseFromRobotState(
      start_state,
      end_effector_link
    );


  const auto goal_pose =
    poseFromRobotState(
      goal_state,
      end_effector_link
    );


  /*
   * Calcular automáticamente la distancia cartesiana.
   */
  const double delta_x =
    goal_pose.position.x
    - start_pose.position.x;


  const double delta_y =
    goal_pose.position.y
    - start_pose.position.y;


  const double delta_z =
    goal_pose.position.z
    - start_pose.position.z;


  const double cartesian_length =
    std::sqrt(
      delta_x * delta_x
      +
      delta_y * delta_y
      +
      delta_z * delta_z
    );


  RCLCPP_INFO(
    logger,
    "Movimiento: %s -> %s",
    start_state_name.c_str(),
    goal_state_name.c_str()
  );


  RCLCPP_INFO(
    logger,
    "Pose inicial: [%.6f, %.6f, %.6f]",
    start_pose.position.x,
    start_pose.position.y,
    start_pose.position.z
  );


  RCLCPP_INFO(
    logger,
    "Pose final: [%.6f, %.6f, %.6f]",
    goal_pose.position.x,
    goal_pose.position.y,
    goal_pose.position.z
  );


  RCLCPP_INFO(
    logger,
    "Longitud cartesiana: %.9f m",
    cartesian_length
  );


  RCLCPP_INFO(
    logger,
    "Velocidad máxima cartesiana: %.6f m/s",
    maximum_cartesian_velocity
  );


  RCLCPP_INFO(
    logger,
    "Aceleración máxima cartesiana: %.6f m/s²",
    maximum_cartesian_acceleration
  );


  const double orientation_difference =
    quaternionDifference(
      start_pose.orientation,
      goal_pose.orientation
    );


  RCLCPP_INFO(
    logger,
    "Diferencia de orientación: %.12f",
    orientation_difference
  );


  if (orientation_difference > 1e-3)
  {
    RCLCPP_WARN(
      logger,
      "Las orientaciones no coinciden. "
      "Se conservará la orientación inicial."
    );
  }


  try
  {
    /*
     * Generar automáticamente cuatro puntos intermedios.
     */
    constexpr std::size_t intermediate_points =
      4;


    const auto cubic_profile =
      generateProfile(
        cartesian_length,
        maximum_cartesian_velocity,
        maximum_cartesian_acceleration,
        intermediate_points,
        false
      );


    const auto quintic_profile =
      generateProfile(
        cartesian_length,
        maximum_cartesian_velocity,
        maximum_cartesian_acceleration,
        intermediate_points,
        true
      );


    RCLCPP_INFO(
      logger,
      "Duración cúbica automática: %.6f s",
      cubic_profile.back().time
    );


    RCLCPP_INFO(
      logger,
      "Duración quíntica automática: %.6f s",
      quintic_profile.back().time
    );


    moveit_msgs::msg::RobotTrajectory
      cubic_trajectory;


    moveit_msgs::msg::RobotTrajectory
      quintic_trajectory;


    const double cubic_fraction =
      processProfile(
        move_group,
        start_state,
        "cúbico",
        cubic_profile,
        start_pose,
        goal_pose,
        cubic_trajectory,
        logger
      );


    const double quintic_fraction =
      processProfile(
        move_group,
        start_state,
        "quíntico",
        quintic_profile,
        start_pose,
        goal_pose,
        quintic_trajectory,
        logger
      );


    if (
      cubic_fraction < 0.99
      ||
      quintic_fraction < 0.99)
    {
      RCLCPP_ERROR(
        logger,
        "Al menos uno de los perfiles "
        "no completó el camino cartesiano"
      );


      executor.cancel();

      executor_thread.join();

      rclcpp::shutdown();


      return 2;
    }


    temporalizeTrajectory(
      cubic_trajectory,
      move_group.getRobotModel(),
      start_pose,
      goal_pose,
      end_effector_link,
      cubic_profile.back().time,
      false
    );


    temporalizeTrajectory(
      quintic_trajectory,
      move_group.getRobotModel(),
      start_pose,
      goal_pose,
      end_effector_link,
      quintic_profile.back().time,
      true
    );


    const auto cubic_metrics =
      calculateTrajectoryMetrics(
        cubic_trajectory
      );


    const auto quintic_metrics =
      calculateTrajectoryMetrics(
        quintic_trajectory
      );


    RCLCPP_INFO(
      logger,
      "========================================"
    );


    RCLCPP_INFO(
      logger,
      "Métricas del perfil cúbico"
    );


    RCLCPP_INFO(
      logger,
      "Duración: %.6f s",
      cubic_metrics.duration
    );


    RCLCPP_INFO(
      logger,
      "Velocidad máxima: %.6f rad/s",
      cubic_metrics.maximum_velocity
    );


    RCLCPP_INFO(
      logger,
      "Aceleración máxima: %.6f rad/s²",
      cubic_metrics.maximum_acceleration
    );


    RCLCPP_INFO(
      logger,
      "Coste de aceleración: %.9f",
      cubic_metrics.acceleration_cost
    );


    RCLCPP_INFO(
      logger,
      "Coste de jerk: %.9f",
      cubic_metrics.jerk_cost
    );


    RCLCPP_INFO(
      logger,
      "========================================"
    );


    RCLCPP_INFO(
      logger,
      "Métricas del perfil quíntico"
    );


    RCLCPP_INFO(
      logger,
      "Duración: %.6f s",
      quintic_metrics.duration
    );


    RCLCPP_INFO(
      logger,
      "Velocidad máxima: %.6f rad/s",
      quintic_metrics.maximum_velocity
    );


    RCLCPP_INFO(
      logger,
      "Aceleración máxima: %.6f rad/s²",
      quintic_metrics.maximum_acceleration
    );


    RCLCPP_INFO(
      logger,
      "Coste de aceleración: %.9f",
      quintic_metrics.acceleration_cost
    );


    RCLCPP_INFO(
      logger,
      "Coste de jerk: %.9f",
      quintic_metrics.jerk_cost
    );


    /*
     * Verificar límites de velocidad articular.
     */
    const bool cubic_limits_valid =
      checkVelocityLimits(
        cubic_trajectory,
        move_group.getRobotModel(),
        "cúbico",
        logger
      );


    const bool quintic_limits_valid =
      checkVelocityLimits(
        quintic_trajectory,
        move_group.getRobotModel(),
        "quíntico",
        logger
      );


    if (
      !cubic_limits_valid
      ||
      !quintic_limits_valid)
    {
      RCLCPP_ERROR(
        logger,
        "Al menos un perfil supera límites articulares"
      );


      executor.cancel();

      executor_thread.join();

      rclcpp::shutdown();


      return 3;
    }


    /*
     * Perfil seleccionado según el análisis previo:
     *
     * menor coste de aceleración y menor coste de jerk.
     */
    RCLCPP_INFO(
      logger,
      "Perfil seleccionado: cúbico"
    );


    /*
     * Verificar que el robot esté realmente en el estado inicial.
     */
    const double start_state_tolerance =
      0.02;


    const bool start_state_is_correct =
      currentStateMatchesStart(
        move_group,
        start_state,
        start_state_tolerance,
        logger
      );


    if (!start_state_is_correct)
    {
      RCLCPP_ERROR(
        logger,
        "El robot no se encuentra en %s",
        start_state_name.c_str()
      );


      RCLCPP_ERROR(
        logger,
        "Ejecución cancelada"
      );


      executor.cancel();

      executor_thread.join();

      rclcpp::shutdown();


      return 4;
    }


    /*
     * Construir el Plan ejecutable.
     */
    moveit::planning_interface::
      MoveGroupInterface::Plan selected_plan;


    selected_plan.trajectory =
      cubic_trajectory;


    selected_plan.planning_time =
      0.0;


    moveit::core::robotStateToRobotStateMsg(
      start_state,
      selected_plan.start_state
    );


    RCLCPP_INFO(
      logger,
      "========================================"
    );


    RCLCPP_INFO(
      logger,
      "Ejecutando trayectoria cúbica"
    );


    RCLCPP_INFO(
      logger,
      "Movimiento: %s -> %s",
      start_state_name.c_str(),
      goal_state_name.c_str()
    );


    RCLCPP_INFO(
      logger,
      "Duración programada: %.6f s",
      cubic_metrics.duration
    );


    const auto execution_result =
      move_group.execute(
        selected_plan
      );


    const bool execution_success =
      static_cast<bool>(
        execution_result
      );


    if (!execution_success)
    {
      RCLCPP_ERROR(
        logger,
        "El controlador no pudo ejecutar la trayectoria"
      );


      executor.cancel();

      executor_thread.join();

      rclcpp::shutdown();


      return 5;
    }


    RCLCPP_INFO(
      logger,
      "Trayectoria ejecutada correctamente"
    );


    RCLCPP_INFO(
      logger,
      "El robot terminó en el estado %s",
      goal_state_name.c_str()
    );
  }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(
      logger,
      "%s",
      exception.what()
    );


    executor.cancel();

    executor_thread.join();

    rclcpp::shutdown();


    return 1;
  }


  executor.cancel();

  executor_thread.join();

  rclcpp::shutdown();


  return 0;
}