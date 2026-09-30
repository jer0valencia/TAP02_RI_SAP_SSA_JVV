cmake_minimum_required(VERSION 3.8)

project(kuka_kr10_tasks)


# ============================================================================
# OPCIONES DEL COMPILADOR
# ============================================================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )

endif()


# ============================================================================
# ESTÁNDAR DE C++
# ============================================================================

if(NOT CMAKE_CXX_STANDARD)

  set(CMAKE_CXX_STANDARD 17)

endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ============================================================================
# DEPENDENCIAS
# ============================================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(builtin_interfaces REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(shape_msgs REQUIRED)

find_package(trajectory_msgs REQUIRED)

find_package(moveit_msgs REQUIRED)

find_package(moveit_core REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)


# ============================================================================
# EJECUTABLE: create_environment
# ============================================================================
#
# Crea:
#
# - Mesa Pick.
# - Pieza.
# - Poste central.
# - Mesa Place.
# - Colores de los objetos.
#
# Ejecución:
#
# ros2 run kuka_kr10_tasks create_environment
#

add_executable(
  create_environment
  src/create_environment.cpp
)


ament_target_dependencies(
  create_environment

  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_ros_planning_interface
)


# ============================================================================
# EJECUTABLE: evaluate
# ============================================================================
#
# Evalúa los planeadores de OMPL para:
#
# Home -> pre_pick
#
# Ejecución:
#
# ros2 run kuka_kr10_tasks evaluate
#

add_executable(
  evaluate
  src/evaluate.cpp
)


ament_target_dependencies(
  evaluate

  rclcpp
  builtin_interfaces
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)


# ============================================================================
# EJECUTABLE: cartesian_path
# ============================================================================
#
# Calcula, analiza y ejecuta movimientos cartesianos:
#
# pre_pick -> pick
#
# y posteriormente:
#
# pre_place -> place
#
# Ejecución:
#
# ros2 run kuka_kr10_tasks cartesian_path \
#   pre_pick \
#   pick \
#   /ruta/al/directorio/profiles
#

add_executable(
  cartesian_path
  src/cartesian_path.cpp
)


ament_target_dependencies(
  cartesian_path

  rclcpp
  builtin_interfaces
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)
# ============================================================================
# EJECUTABLE: pick_place_cycle
# ============================================================================

add_executable(
  pick_place_cycle
  src/pick_place_cycle.cpp
)

ament_target_dependencies(
  pick_place_cycle

  rclcpp
  builtin_interfaces
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)
# ============================================================================
# EJECUTABLE: jacobian_check
# ============================================================================

add_executable(
  jacobian_check
  src/jacobian_check.cpp
)

ament_target_dependencies(
  jacobian_check

  rclcpp
  moveit_core
  moveit_ros_planning_interface
)

# ============================================================================
# INSTALACIÓN DE LOS EJECUTABLES C++
# ============================================================================

install(
  TARGETS
    jacobian_check
    create_environment
    evaluate
    cartesian_path
    pick_place_cycle

  DESTINATION

    lib/${PROJECT_NAME}
)


# ============================================================================
# INSTALACIÓN DEL SCRIPT PYTHON
# ============================================================================
#
# Esto permite utilizar:
#
# ros2 run kuka_kr10_tasks generate_profiles.py
#
# También puede ejecutarse directamente mediante python3.
#

install(
  PROGRAMS

    scripts/generate_profiles.py

  DESTINATION

    lib/${PROJECT_NAME}
)


# ============================================================================
# INSTALACIÓN DE LOS PERFILES
# ============================================================================
#
# Copia cubic.csv y quintic.csv al directorio install.
#
# OPTIONAL evita que la compilación falle si la carpeta profiles
# todavía no existe.
#

install(
  DIRECTORY

    profiles

  DESTINATION

    share/${PROJECT_NAME}

  OPTIONAL
)


# ============================================================================
# PRUEBAS
# ============================================================================

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

endif()


# ============================================================================
# REGISTRO FINAL DEL PAQUETE
# ============================================================================

ament_package()