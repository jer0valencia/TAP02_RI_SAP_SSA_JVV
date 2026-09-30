/*
 * ============================================================================
 * NODO: create_environment
 * ============================================================================
 *
 * PROPÓSITO
 * ---------
 *
 * Crear el entorno de colisión para el ciclo Pick & Place del
 * KUKA KR10 R1100 sixx.
 *
 * El entorno contiene cuatro objetos:
 *
 * 1. pick_table
 *    Mesa donde comienza la pieza.
 *
 * 2. part
 *    Pieza que será recogida por el robot.
 *
 * 3. central_post
 *    Obstáculo fijo que deberá rodear el manipulador.
 *
 * 4. place_table
 *    Mesa donde será depositada la pieza.
 *
 * Las posiciones de las mesas se definieron utilizando las posiciones
 * cartesianas de Pick y Place obtenidas mediante TF:
 *
 * Pick:
 *
 *   x =  0.620 m
 *   y =  0.466 m
 *   z =  0.346 m
 *
 * Place:
 *
 *   x =  0.620 m
 *   y = -0.202 m
 *   z =  0.346 m
 *
 * Este nodo:
 *
 * - NO mueve el robot.
 * - NO calcula cinemática inversa.
 * - NO ejecuta trayectorias.
 * - NO utiliza OMPL.
 *
 * El nodo únicamente describe el entorno y lo añade a la PlanningScene
 * de MoveIt.
 *
 * Flujo:
 *
 * create_environment
 *        |
 *        | CollisionObject + ObjectColor
 *        v
 * PlanningSceneInterface
 *        |
 *        v
 * move_group
 *        |
 *        v
 * PlanningSceneMonitor
 *        |
 *        v
 * /monitored_planning_scene
 *        |
 *        v
 * RViz
 *
 * ============================================================================
 */


/*
 * ============================================================================
 * LIBRERÍAS ESTÁNDAR DE C++
 * ============================================================================
 */

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>


/*
 * ============================================================================
 * LIBRERÍAS DE ROS 2
 * ============================================================================
 */

#include <rclcpp/rclcpp.hpp>


/*
 * ============================================================================
 * MENSAJES E INTERFACES DE MOVEIT 2
 * ============================================================================
 */

/*
 * Mensaje utilizado para representar posición y orientación.
 */
#include <geometry_msgs/msg/pose.hpp>

/*
 * Mensaje que describe un objeto de colisión.
 */
#include <moveit_msgs/msg/collision_object.hpp>

/*
 * Mensaje que asocia un color RGBA a un objeto de la PlanningScene.
 */
#include <moveit_msgs/msg/object_color.hpp>

/*
 * Interfaz de alto nivel para modificar la PlanningScene de MoveIt.
 */
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

/*
 * Mensaje utilizado para crear primitivas como cajas,
 * cilindros, conos y esferas.
 */
#include <shape_msgs/msg/solid_primitive.hpp>


/*
 * ============================================================================
 * FUNCIÓN AUXILIAR: createBox
 * ============================================================================
 *
 * Construye un CollisionObject con forma de caja.
 *
 * PARÁMETROS
 * ----------
 *
 * object_id:
 *   Identificador único utilizado por MoveIt.
 *
 * frame_id:
 *   Frame respecto al que se expresa la pose.
 *
 * size_x:
 *   Dimensión de la caja sobre X local, en metros.
 *
 * size_y:
 *   Dimensión de la caja sobre Y local, en metros.
 *
 * size_z:
 *   Dimensión de la caja sobre Z local, en metros.
 *
 * position_x:
 *   Coordenada X del centro de la caja.
 *
 * position_y:
 *   Coordenada Y del centro de la caja.
 *
 * position_z:
 *   Coordenada Z del centro de la caja.
 *
 * RETORNO
 * -------
 *
 * CollisionObject configurado y listo para añadirse a MoveIt.
 */
moveit_msgs::msg::CollisionObject createBox(
  const std::string& object_id,
  const std::string& frame_id,
  const double size_x,
  const double size_y,
  const double size_z,
  const double position_x,
  const double position_y,
  const double position_z)
{
  /*
   * Creamos el mensaje del objeto.
   */
  moveit_msgs::msg::CollisionObject collision_object;


  /*
   * Indicamos el frame respecto al que están definidas las coordenadas.
   */
  collision_object.header.frame_id = frame_id;


  /*
   * Asignamos el identificador único.
   */
  collision_object.id = object_id;


  /*
   * Creamos la primitiva geométrica.
   */
  shape_msgs::msg::SolidPrimitive primitive;


  /*
   * Indicamos que la geometría será una caja.
   */
  primitive.type =
    shape_msgs::msg::SolidPrimitive::BOX;


  /*
   * Una caja necesita tres dimensiones.
   */
  primitive.dimensions.resize(3);


  /*
   * Dimensión X.
   */
  primitive.dimensions[
    shape_msgs::msg::SolidPrimitive::BOX_X
  ] = size_x;


  /*
   * Dimensión Y.
   */
  primitive.dimensions[
    shape_msgs::msg::SolidPrimitive::BOX_Y
  ] = size_y;


  /*
   * Dimensión Z.
   */
  primitive.dimensions[
    shape_msgs::msg::SolidPrimitive::BOX_Z
  ] = size_z;


  /*
   * Creamos la pose del centro geométrico.
   */
  geometry_msgs::msg::Pose object_pose;


  /*
   * Posición del centro de la caja respecto al frame seleccionado.
   */
  object_pose.position.x = position_x;
  object_pose.position.y = position_y;
  object_pose.position.z = position_z;


  /*
   * Cuaternión identidad:
   *
   * [x, y, z, w] = [0, 0, 0, 1]
   *
   * La caja queda alineada con los ejes de base_link.
   */
  object_pose.orientation.x = 0.0;
  object_pose.orientation.y = 0.0;
  object_pose.orientation.z = 0.0;
  object_pose.orientation.w = 1.0;


  /*
   * Añadimos la geometría al CollisionObject.
   */
  collision_object.primitives.push_back(
    primitive
  );


  /*
   * Añadimos la pose asociada a la geometría.
   *
   * primitives[0] corresponde a primitive_poses[0].
   */
  collision_object.primitive_poses.push_back(
    object_pose
  );


  /*
   * Indicamos que el objeto debe añadirse o actualizarse.
   */
  collision_object.operation =
    moveit_msgs::msg::CollisionObject::ADD;


  /*
   * Retornamos el objeto configurado.
   */
  return collision_object;
}


/*
 * ============================================================================
 * FUNCIÓN AUXILIAR: createColor
 * ============================================================================
 *
 * Crea el color visual asociado a un CollisionObject.
 *
 * El color no modifica:
 *
 * - La geometría.
 * - Las dimensiones.
 * - La detección de colisiones.
 * - La planificación.
 *
 * Únicamente cambia cómo RViz visualiza el objeto.
 *
 * Las componentes RGBA deben estar entre 0 y 1.
 */
moveit_msgs::msg::ObjectColor createColor(
  const std::string& object_id,
  const double red,
  const double green,
  const double blue,
  const double alpha)
{
  /*
   * Creamos el mensaje de color.
   */
  moveit_msgs::msg::ObjectColor object_color;


  /*
   * El identificador debe coincidir exactamente con el
   * CollisionObject correspondiente.
   */
  object_color.id = object_id;


  /*
   * Asignamos las componentes RGBA.
   */
  object_color.color.r = red;
  object_color.color.g = green;
  object_color.color.b = blue;
  object_color.color.a = alpha;


  /*
   * Retornamos el color configurado.
   */
  return object_color;
}


/*
 * ============================================================================
 * FUNCIÓN PRINCIPAL
 * ============================================================================
 */

int main(int argc, char* argv[])
{
  /*
   * Inicializamos ROS 2.
   */
  rclcpp::init(argc, argv);


  /*
   * Creamos el nodo.
   *
   * El nodo se podrá observar como:
   *
   * /create_environment
   */
  auto node = std::make_shared<rclcpp::Node>(
    "create_environment"
  );


  /*
   * Obtenemos el logger.
   */
  auto logger = node->get_logger();


  /*
   * Creamos la interfaz para modificar el mundo de MoveIt.
   */
  moveit::planning_interface::PlanningSceneInterface
    planning_scene_interface;


  /*
   * ==========================================================================
   * FRAME DE REFERENCIA
   * ==========================================================================
   *
   * Todos los objetos estarán definidos respecto a base_link.
   */
  const std::string planning_frame = "base_link";


  /*
   * ==========================================================================
   * POSICIONES CARTESIANAS DE PICK Y PLACE
   * ==========================================================================
   *
   * Estas posiciones corresponden al origen de tool0.
   *
   * Se expresan en metros respecto a base_link.
   */


  /*
   * Posición Pick.
   */
  const double pick_x = 0.620;
  const double pick_y = 0.466;
  const double pick_z = 0.346;


  /*
   * Posición Place.
   */
  const double place_x = 0.620;
  const double place_y = -0.202;
  const double place_z = 0.346;


  /*
   * ==========================================================================
   * DIMENSIONES GENERALES
   * ==========================================================================
   */


  /*
   * Dimensiones del tablero de las mesas.
   */
  const double table_size_x = 0.35;
  const double table_size_y = 0.35;
  const double table_size_z = 0.02;


  /*
   * Dimensiones de la pieza.
   */
  const double part_size_x = 0.05;
  const double part_size_y = 0.05;
  const double part_size_z = 0.005;


  /*
   * La cara superior de la pieza debe coincidir inicialmente con la
   * coordenada Z del TCP en la pose Pick.
   *
   * Centro pieza:
   *
   * z_part = z_pick - altura_pieza/2
   */
  const double part_center_z =
    pick_z - part_size_z-0.005;


  /*
   * La cara inferior de la pieza determina la superficie de la mesa.
   *
   * Superficie mesa:
   *
   * z_surface = z_part - altura_pieza/2
   */
  const double table_surface_z =
    part_center_z - part_size_z / 2.0;


  /*
   * Calculamos el centro del tablero.
   *
   * Centro tablero:
   *
   * z_table = z_surface - espesor_tablero/2
   */
  const double table_center_z =
    table_surface_z- table_size_z;


  /*
   * ==========================================================================
   * OBJETO 1: MESA DE PICK
   * ==========================================================================
   *
   * La mesa queda centrada bajo la coordenada XY de Pick.
   *
   * Su superficie superior queda en:
   *
   * z = 0.266 m
   *
   * Su centro queda en:
   *
   * z = 0.241 m
   */
  const auto pick_table = createBox(
    "pick_table",
    planning_frame,

    table_size_x,
    table_size_y,
    table_size_z,

    pick_x,
    pick_y,
    table_center_z
  );


  /*
   * ==========================================================================
   * OBJETO 2: PIEZA
   * ==========================================================================
   *
   * La pieza está centrada bajo el origen de tool0 en Pick.
   *
   * La cara superior de la pieza queda en:
   *
   * z = 0.346 m
   *
   * Su centro queda en:
   *
   * z = 0.306 m
   *
   * La cara inferior queda en:
   *
   * z = 0.266 m
   *
   * Esta cara inferior coincide con la superficie de la mesa.
   */
  const auto part = createBox(
    "part",
    planning_frame,

    part_size_x,
    part_size_y,
    part_size_z,

    pick_x,
    pick_y,
    part_center_z
  );


  /*
   * ==========================================================================
   * OBJETO 3: POSTE CENTRAL
   * ==========================================================================
   *
   * El poste es el obstáculo que obligará al planeador a buscar una ruta
   * libre de colisión.
   *
   * Estas coordenadas son una propuesta inicial.
   *
   * La posición deberá validarse visualmente en RViz y mediante planificación.
   *
   * El poste mide 0,70 m de alto y su centro está en 0,35 m.
   *
   * Por tanto:
   *
   * Cara inferior = 0,00 m
   * Cara superior = 0,70 m
   */
  const auto central_post = createBox(
    "central_post",
    planning_frame,

    0.12,
    0.12,
    1.80,

    0.54,
    0.13,
    0.90
  );


  /*
   * ==========================================================================
   * OBJETO 4: MESA DE PLACE
   * ==========================================================================
   *
   * La mesa Place se centra debajo de la posición XY de Place.
   *
   * Utilizamos la misma altura que la mesa Pick porque ambas poses tienen:
   *
   * z = 0.346 m
   *
   * y la misma pieza será depositada sobre la mesa Place.
   */
  const auto place_table = createBox(
    "place_table",
    planning_frame,

    table_size_x,
    table_size_y,
    table_size_z,

    place_x,
    place_y,
    table_center_z
  );


  /*
   * ==========================================================================
   * COLORES DE LOS OBJETOS
   * ==========================================================================
   */


  /*
   * Mesa Pick: azul.
   */
  const auto pick_table_color = createColor(
    "pick_table",
    0.10,
    0.35,
    0.90,
    1.00
  );


  /*
   * Pieza: naranja.
   */
  const auto part_color = createColor(
    "part",
    1.00,
    0.55,
    0.00,
    1.00
  );


  /*
   * Poste: rojo.
   */
  const auto central_post_color = createColor(
    "central_post",
    0.85,
    0.10,
    0.10,
    1.00
  );


  /*
   * Mesa Place: verde.
   */
  const auto place_table_color = createColor(
    "place_table",
    0.10,
    0.75,
    0.25,
    1.00
  );


  /*
   * ==========================================================================
   * VECTOR DE OBJETOS DE COLISIÓN
   * ==========================================================================
   */

  std::vector<moveit_msgs::msg::CollisionObject>
    collision_objects;


  /*
   * Introducimos los cuatro objetos.
   */
  collision_objects.push_back(
    pick_table
  );

  collision_objects.push_back(
    part
  );

  collision_objects.push_back(
    central_post
  );

  collision_objects.push_back(
    place_table
  );


  /*
   * ==========================================================================
   * VECTOR DE COLORES
   * ==========================================================================
   */

  std::vector<moveit_msgs::msg::ObjectColor>
    object_colors;


  /*
   * Introducimos los colores.
   *
   * Cada color se asocia a su objeto mediante el campo id.
   */
  object_colors.push_back(
    pick_table_color
  );

  object_colors.push_back(
    part_color
  );

  object_colors.push_back(
    central_post_color
  );

  object_colors.push_back(
    place_table_color
  );


  /*
   * ==========================================================================
   * ELIMINAR LA CAJA DE PRUEBA ANTERIOR
   * ==========================================================================
   *
   * Si test_box todavía existe en la PlanningScene de esta sesión, lo
   * eliminamos para evitar que quede mezclado con el entorno definitivo.
   *
   * Si el objeto no existe, MoveIt simplemente no tendrá nada que eliminar.
   */
  planning_scene_interface.removeCollisionObjects(
    {"test_box"}
  );


  /*
   * ==========================================================================
   * APLICAR OBJETOS Y COLORES
   * ==========================================================================
   *
   * Esta es la instrucción que realmente modifica el mundo de MoveIt.
   *
   * collision_objects:
   *   Define la geometría, dimensiones y poses.
   *
   * object_colors:
   *   Define únicamente la apariencia visual.
   */
  const bool success =
    planning_scene_interface.applyCollisionObjects(
      collision_objects,
      object_colors
    );


  /*
   * Si MoveIt no pudo aplicar el entorno, terminamos con error.
   */
  if (!success)
  {
    RCLCPP_ERROR(
      logger,
      "MoveIt no pudo crear el entorno de colisión"
    );

    rclcpp::shutdown();

    return 1;
  }


  /*
   * ==========================================================================
   * INFORMACIÓN MOSTRADA EN TERMINAL
   * ==========================================================================
   */

  RCLCPP_INFO(
    logger,
    "Entorno añadido correctamente a la PlanningScene"
  );

  RCLCPP_INFO(
    logger,
    "Frame de referencia: %s",
    planning_frame.c_str()
  );

  RCLCPP_INFO(
    logger,
    "Pick: x=%.3f, y=%.3f, z=%.3f m",
    pick_x,
    pick_y,
    pick_z
  );

  RCLCPP_INFO(
    logger,
    "Place: x=%.3f, y=%.3f, z=%.3f m",
    place_x,
    place_y,
    place_z
  );

  RCLCPP_INFO(
    logger,
    "Superficie de las mesas: z=%.3f m",
    table_surface_z
  );

  RCLCPP_INFO(
    logger,
    "Centro de las mesas: z=%.3f m",
    table_center_z
  );

  RCLCPP_INFO(
    logger,
    "Centro de la pieza: z=%.3f m",
    part_center_z
  );

  RCLCPP_INFO(
    logger,
    "Objetos añadidos:"
  );

  RCLCPP_INFO(
    logger,
    "  - pick_table  [azul]"
  );

  RCLCPP_INFO(
    logger,
    "  - part        [naranja]"
  );

  RCLCPP_INFO(
    logger,
    "  - central_post [rojo]"
  );

  RCLCPP_INFO(
    logger,
    "  - place_table [verde]"
  );


  /*
   * ==========================================================================
   * ESPERA BREVE
   * ==========================================================================
   *
   * El nodo permanece vivo dos segundos para facilitar la propagación inicial
   * de la actualización.
   *
   * Los objetos seguirán existiendo en la PlanningScene mientras move_group
   * continúe activo.
   */
  std::this_thread::sleep_for(
    std::chrono::seconds(2)
  );


  /*
   * Cerramos ROS 2 correctamente.
   */
  rclcpp::shutdown();


  /*
   * Código de salida 0: ejecución correcta.
   */
  return 0;
}
