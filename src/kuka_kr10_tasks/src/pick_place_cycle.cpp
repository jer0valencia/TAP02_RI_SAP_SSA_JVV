#include <algorithm>
#include <chrono>
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
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <moveit_msgs/msg/robot_trajectory.hpp>


// ============================================================================
// DATOS DE UN PUNTO DEL PERFIL CÚBICO
// ============================================================================

struct ProfilePoint
{
  double time;
  double tau;
  double s;
  double distance;
  double velocity;
  double acceleration;
};


// ============================================================================
// PERFIL CÚBICO
// ============================================================================

double cubicPosition(const double tau)
{
  return
    3.0 * tau * tau
    - 2.0 * tau * tau * tau;
}


double cubicVelocity(const double tau)
{
  return
    6.0 * tau
    - 6.0 * tau * tau;
}


double cubicAcceleration(const double tau)
{
  return
    6.0
    - 12.0 * tau;
}


// ============================================================================
// GENERAR PERFIL CÚBICO AUTOMÁTICAMENTE
// ============================================================================

std::vector<ProfilePoint> generateCubicProfile(
  const double length,
  const double maximum_velocity,
  const double maximum_acceleration,
  const std::size_t intermediate_points)
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
   * Condición de velocidad:
   *
   * T >= 1.5 L / vmax
   */
  const double velocity_time =
    1.5
    * length
    / maximum_velocity;

  /*
   * Condición de aceleración:
   *
   * T >= sqrt(6 L / amax)
   */
  const double acceleration_time =
    std::sqrt(
      6.0
      * length
      / maximum_acceleration
    );

  const double duration =
    std::max(
      velocity_time,
      acceleration_time
    );

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

    const double s =
      cubicPosition(tau);

    const double ds_dtau =
      cubicVelocity(tau);

    const double d2s_dtau2 =
      cubicAcceleration(tau);

    ProfilePoint point;

    point.time =
      tau * duration;

    point.tau =
      tau;

    point.s =
      s;

    point.distance =
      length * s;

    point.velocity =
      length
      * ds_dtau
      / duration;

    point.acceleration =
      length
      * d2s_dtau2
      / (
          duration
          * duration
        );

    profile.push_back(point);
  }

  profile.front().time = 0.0;
  profile.front().tau = 0.0;
  profile.front().s = 0.0;
  profile.front().distance = 0.0;
  profile.front().velocity = 0.0;

  profile.back().time = duration;
  profile.back().tau = 1.0;
  profile.back().s = 1.0;
  profile.back().distance = length;
  profile.back().velocity = 0.0;

  return profile;
}


// ============================================================================
// INVERSIÓN DEL PERFIL CÚBICO
// ============================================================================

double invertCubicProfile(const double target_s)
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

  double lower_tau = 0.0;
  double upper_tau = 1.0;

  /*
   * Búsqueda binaria porque el perfil cúbico es monótono.
   */
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
      cubicPosition(middle_tau);

    if (middle_s < clamped_s)
    {
      lower_tau = middle_tau;
    }
    else
    {
      upper_tau = middle_tau;
    }
  }

  return
    0.5
    * (
        lower_tau
        + upper_tau
      );
}


// ============================================================================
// CONVERSIÓN DE TIEMPO
// ============================================================================

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
      std::floor(safe_seconds)
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

  if (duration.nanosec >= 1000000000u)
  {
    duration.sec += 1;
    duration.nanosec -= 1000000000u;
  }

  return duration;
}


// ============================================================================
// OBTENER ESTADO NOMBRADO DEL SRDF
// ============================================================================

moveit::core::RobotState getNamedRobotState(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const std::string& state_name)
{
  const auto named_targets =
    move_group.getNamedTargets();

  const bool state_exists =
    std::find(
      named_targets.begin(),
      named_targets.end(),
      state_name
    )
    != named_targets.end();

  if (!state_exists)
  {
    throw std::runtime_error(
      "No existe el estado nombrado: "
      + state_name
    );
  }

  const auto values =
    move_group.getNamedTargetValues(
      state_name
    );

  moveit::core::RobotState state(
    move_group.getRobotModel()
  );

  state.setToDefaultValues();

  state.setVariablePositions(values);

  state.update();

  return state;
}


// ============================================================================
// CINEMÁTICA DIRECTA
// ============================================================================

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


// ============================================================================
// COMPROBAR ESTADO ACTUAL
// ============================================================================

bool currentStateMatches(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::RobotState& expected_state,
  const double tolerance,
  const rclcpp::Logger& logger)
{
  const auto current_state =
    move_group.getCurrentState(2.0);

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

  bool matches = true;

  for (const auto& joint_name :
       joint_names)
  {
    const double current_value =
      current_state->getVariablePosition(
        joint_name
      );

    const double expected_value =
      expected_state.getVariablePosition(
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
      matches = false;
    }
  }

  return matches;
}


// ============================================================================
// MOVIMIENTO LIBRE CON OMPL Y REINTENTOS
// ============================================================================

bool moveNamed(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const std::string& goal_state_name,
  const std::string& planner_id,
  const rclcpp::Logger& logger)
{
  RCLCPP_INFO(
    logger,
    "========================================"
  );

  RCLCPP_INFO(
    logger,
    "Movimiento libre hacia: %s",
    goal_state_name.c_str()
  );

  move_group.setPlanningPipelineId(
    "ompl"
  );

  move_group.setPlannerId(
    planner_id
  );

  /*
   * Cinco segundos por intento.
   */
  move_group.setPlanningTime(
    5.0
  );

  /*
   * Cada llamada a plan() representa un intento.
   */
  move_group.setNumPlanningAttempts(
    1
  );

  move_group.setMaxVelocityScalingFactor(
    0.20
  );

  move_group.setMaxAccelerationScalingFactor(
    0.20
  );

  /*
   * Hasta 10 intentos independientes.
   */
  constexpr std::size_t maximum_attempts =
    10;

  moveit::planning_interface::
    MoveGroupInterface::Plan successful_plan;

  bool plan_found = false;

  std::size_t successful_attempt = 0;

  for (std::size_t attempt = 1;
       attempt <= maximum_attempts;
       ++attempt)
  {
    RCLCPP_INFO(
      logger,
      "Intento OMPL %zu de %zu hacia %s",
      attempt,
      maximum_attempts,
      goal_state_name.c_str()
    );

    /*
     * Actualizar el estado inicial antes de cada intento.
     */
    move_group.setStartStateToCurrentState();

    /*
     * Limpiar objetivos cartesianos anteriores.
     */
    move_group.clearPoseTargets();

    /*
     * Establecer de nuevo el objetivo nombrado.
     */
    const bool target_set =
      move_group.setNamedTarget(
        goal_state_name
      );

    if (!target_set)
    {
      RCLCPP_ERROR(
        logger,
        "No fue posible establecer el objetivo %s",
        goal_state_name.c_str()
      );

      return false;
    }

    moveit::planning_interface::
      MoveGroupInterface::Plan candidate_plan;

    const auto planning_result =
      move_group.plan(
        candidate_plan
      );

    if (static_cast<bool>(planning_result))
    {
      successful_plan =
        candidate_plan;

      successful_attempt =
        attempt;

      plan_found =
        true;

      break;
    }

    RCLCPP_WARN(
      logger,
      "RRTConnect no encontró trayectoria "
      "en el intento %zu hacia %s",
      attempt,
      goal_state_name.c_str()
    );

    std::this_thread::sleep_for(
      std::chrono::milliseconds(100)
    );
  }

  if (!plan_found)
  {
    RCLCPP_ERROR(
      logger,
      "No se encontró trayectoria hacia %s "
      "después de %zu intentos",
      goal_state_name.c_str(),
      maximum_attempts
    );

    return false;
  }

  RCLCPP_INFO(
    logger,
    "Planificación exitosa hacia %s "
    "en el intento %zu",
    goal_state_name.c_str(),
    successful_attempt
  );

  RCLCPP_INFO(
    logger,
    "Tiempo de planificación: %.6f s",
    successful_plan.planning_time
  );

  RCLCPP_INFO(
    logger,
    "Puntos de trayectoria: %zu",
    successful_plan
      .trajectory
      .joint_trajectory
      .points
      .size()
  );

  const auto execution_result =
    move_group.execute(
      successful_plan
    );

  if (!static_cast<bool>(execution_result))
  {
    RCLCPP_ERROR(
      logger,
      "No fue posible ejecutar el movimiento hacia %s",
      goal_state_name.c_str()
    );

    return false;
  }

  RCLCPP_INFO(
    logger,
    "Robot en estado %s",
    goal_state_name.c_str()
  );

  return true;
}


// ============================================================================
// CONSTRUIR WAYPOINTS CARTESIANOS
// ============================================================================

std::vector<geometry_msgs::msg::Pose> buildCubicWaypoints(
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
     * Orientación constante.
     */
    waypoint.orientation =
      start_pose.orientation;

    waypoints.push_back(waypoint);
  }

  return waypoints;
}


// ============================================================================
// CALCULAR AVANCE CARTESIANO DE LA TRAYECTORIA
// ============================================================================

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

    double progress =
      (
        point_delta_x * delta_x
        +
        point_delta_y * delta_y
        +
        point_delta_z * delta_z
      )
      / squared_length;

    progress =
      std::clamp(
        progress,
        0.0,
        1.0
      );

    /*
     * Evitar pequeños retrocesos numéricos.
     */
    if (
      !progress_values.empty()
      &&
      progress < progress_values.back())
    {
      progress =
        progress_values.back();
    }

    progress_values.push_back(
      progress
    );
  }

  if (!progress_values.empty())
  {
    progress_values.front() = 0.0;
    progress_values.back() = 1.0;
  }

  return progress_values;
}


// ============================================================================
// TEMPORIZAR TRAYECTORIA CÚBICA
// ============================================================================

void temporalizeCubicTrajectory(
  moveit_msgs::msg::RobotTrajectory& trajectory,
  const moveit::core::RobotModelConstPtr& robot_model,
  const geometry_msgs::msg::Pose& start_pose,
  const geometry_msgs::msg::Pose& goal_pose,
  const std::string& end_effector_link,
  const double duration)
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
   * s -> tau -> tiempo.
   */
  for (std::size_t point_index = 0;
       point_index < number_of_points;
       ++point_index)
  {
    const double tau =
      invertCubicProfile(
        progress_values[point_index]
      );

    times[point_index] =
      tau * duration;
  }

  times.front() = 0.0;
  times.back() = duration;

  /*
   * Garantizar tiempos crecientes.
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
    >= duration)
  {
    throw std::runtime_error(
      "No fue posible crear tiempos crecientes"
    );
  }

  times.back() = duration;

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
   * Velocidades interiores.
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
        "Intervalo temporal inválido al calcular velocidad"
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
        "Intervalo temporal inválido al calcular aceleración"
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
        "Intervalo temporal inicial inválido"
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
        "Intervalo temporal final inválido"
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


// ============================================================================
// MOVIMIENTO CARTESIANO CÚBICO
// ============================================================================

bool moveCartesianCubic(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const std::string& start_state_name,
  const std::string& goal_state_name,
  const double maximum_cartesian_velocity,
  const double maximum_cartesian_acceleration,
  const std::string& end_effector_link,
  const rclcpp::Logger& logger)
{
  RCLCPP_INFO(
    logger,
    "========================================"
  );

  RCLCPP_INFO(
    logger,
    "Movimiento cartesiano cúbico: %s -> %s",
    start_state_name.c_str(),
    goal_state_name.c_str()
  );

  const auto start_state =
    getNamedRobotState(
      move_group,
      start_state_name
    );

  const auto goal_state =
    getNamedRobotState(
      move_group,
      goal_state_name
    );

  /*
   * Confirmar que el robot está en el inicio.
   */
  constexpr double state_tolerance =
    0.02;

  if (
    !currentStateMatches(
      move_group,
      start_state,
      state_tolerance,
      logger
    ))
  {
    RCLCPP_ERROR(
      logger,
      "El robot no está en %s",
      start_state_name.c_str()
    );

    return false;
  }

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
    "Longitud cartesiana: %.9f m",
    cartesian_length
  );

  /*
   * Cuatro puntos intermedios.
   */
  constexpr std::size_t intermediate_points =
    4;

  const auto profile =
    generateCubicProfile(
      cartesian_length,
      maximum_cartesian_velocity,
      maximum_cartesian_acceleration,
      intermediate_points
    );

  const auto waypoints =
    buildCubicWaypoints(
      start_pose,
      goal_pose,
      profile
    );

  move_group.setStartState(
    start_state
  );

  moveit_msgs::msg::RobotTrajectory
    trajectory;

  /*
   * Resolución cartesiana de 5 mm.
   */
  constexpr double eef_step =
    0.005;

  const double fraction =
    move_group.computeCartesianPath(
      waypoints,
      eef_step,
      trajectory,
      true
    );

  RCLCPP_INFO(
    logger,
    "Fracción cartesiana: %.2f %%",
    fraction * 100.0
  );

  if (fraction < 0.99)
  {
    RCLCPP_ERROR(
      logger,
      "El camino cartesiano no fue completado"
    );

    return false;
  }

  temporalizeCubicTrajectory(
    trajectory,
    move_group.getRobotModel(),
    start_pose,
    goal_pose,
    end_effector_link,
    profile.back().time
  );

  moveit::planning_interface::
    MoveGroupInterface::Plan plan;

  plan.trajectory =
    trajectory;

  plan.planning_time =
    0.0;

  moveit::core::robotStateToRobotStateMsg(
    start_state,
    plan.start_state
  );

  RCLCPP_INFO(
    logger,
    "Duración cúbica: %.6f s",
    profile.back().time
  );

  const auto execution_result =
    move_group.execute(
      plan
    );

  if (!static_cast<bool>(execution_result))
  {
    RCLCPP_ERROR(
      logger,
      "No fue posible ejecutar %s -> %s",
      start_state_name.c_str(),
      goal_state_name.c_str()
    );

    return false;
  }

  RCLCPP_INFO(
    logger,
    "Robot en estado %s",
    goal_state_name.c_str()
  );

  return true;
}


// ============================================================================
// ADJUNTAR PIEZA
// ============================================================================

bool attachPart(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const rclcpp::Logger& logger)
{
  const std::string object_id =
    "part";

  const std::string attach_link =
    "tool0";

  const std::vector<std::string> touch_links =
  {
    "tool0"
  };

  const bool attached =
    move_group.attachObject(
      object_id,
      attach_link,
      touch_links
    );

  if (!attached)
  {
    RCLCPP_ERROR(
      logger,
      "No fue posible adjuntar la pieza"
    );

    return false;
  }

  /*
   * Permitir que la PlanningScene reciba la actualización.
   */
  std::this_thread::sleep_for(
    std::chrono::milliseconds(500)
  );

  RCLCPP_INFO(
    logger,
    "Pieza adjuntada a tool0"
  );

  return true;
}


// ============================================================================
// DESADJUNTAR PIEZA
// ============================================================================

bool detachPart(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const rclcpp::Logger& logger)
{
  const bool detached =
    move_group.detachObject(
      "part"
    );

  if (!detached)
  {
    RCLCPP_ERROR(
      logger,
      "No fue posible desadjuntar la pieza"
    );

    return false;
  }

  std::this_thread::sleep_for(
    std::chrono::milliseconds(500)
  );

  RCLCPP_INFO(
    logger,
    "Pieza desadjuntada en place"
  );

  return true;
}


// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char* argv[])
{
  rclcpp::init(
    argc,
    argv
  );

  auto node =
    std::make_shared<rclcpp::Node>(
      "pick_place_cycle",
      rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(
          true
        )
    );

  const auto logger =
    node->get_logger();

  rclcpp::executors::SingleThreadedExecutor
    executor;

  executor.add_node(node);

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
   * Configuración seleccionada en 4A.
   */
  const std::string planner_id =
    "RRTConnectkConfigDefault";

  const std::string end_effector_link =
    "tool0";

  /*
   * Límites cartesianos del tramo rojo.
   */
  constexpr double maximum_cartesian_velocity =
    0.200;

  constexpr double maximum_cartesian_acceleration =
    0.300;

  try
  {
    /*
     * Verificar estado inicial Home.
     */
    const auto home_state =
      getNamedRobotState(
        move_group,
        "Home"
      );

    constexpr double home_tolerance =
      0.02;

    RCLCPP_INFO(
      logger,
      "Comprobando estado inicial Home"
    );

    if (
      !currentStateMatches(
        move_group,
        home_state,
        home_tolerance,
        logger
      ))
    {
      throw std::runtime_error(
        "El robot no está en Home"
      );
    }

    /*
     * 4A: Home -> pre_pick.
     */
    if (
      !moveNamed(
        move_group,
        "pre_pick",
        planner_id,
        logger
      ))
    {
      throw std::runtime_error(
        "Falló el tramo 4A: Home -> pre_pick"
      );
    }

    /*
     * 4B: pre_pick -> pick.
     */
    if (
      !moveCartesianCubic(
        move_group,
        "pre_pick",
        "pick",
        maximum_cartesian_velocity,
        maximum_cartesian_acceleration,
        end_effector_link,
        logger
      ))
    {
      throw std::runtime_error(
        "Falló el tramo 4B: pre_pick -> pick"
      );
    }

    /*
     * Adjuntar la pieza.
     */
    if (
      !attachPart(
        move_group,
        logger
      ))
    {
      throw std::runtime_error(
        "No fue posible tomar la pieza"
      );
    }

    /*
     * 4C: pick -> pre_place.
     */
    if (
      !moveNamed(
        move_group,
        "pre_place",
        planner_id,
        logger
      ))
    {
      throw std::runtime_error(
        "Falló el tramo 4C: pick -> pre_place"
      );
    }

    /*
     * 4D: pre_place -> place.
     */
    if (
      !moveCartesianCubic(
        move_group,
        "pre_place",
        "place",
        maximum_cartesian_velocity,
        maximum_cartesian_acceleration,
        end_effector_link,
        logger
      ))
    {
      throw std::runtime_error(
        "Falló el tramo 4D: pre_place -> place"
      );
    }

    /*
     * Soltar la pieza.
     */
    if (
      !detachPart(
        move_group,
        logger
      ))
    {
      throw std::runtime_error(
        "No fue posible soltar la pieza"
      );
    }

    RCLCPP_INFO(
      logger,
      "========================================"
    );

    RCLCPP_INFO(
      logger,
      "CICLO PICK-AND-PLACE COMPLETADO"
    );

    RCLCPP_INFO(
      logger,
      "Home -> pre_pick -> pick -> "
      "pre_place -> place"
    );
  }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(
      logger,
      "%s",
      exception.what()
    );

    RCLCPP_ERROR(
      logger,
      "Ciclo detenido"
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