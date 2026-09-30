#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<rclcpp::Node>(
      "jacobian_check",
      rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true)
    );

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node);

  std::thread executor_thread(
    [&executor]()
    {
      executor.spin();
    }
  );

  moveit::planning_interface::MoveGroupInterface move_group(
    node,
    "manipulator"
  );

  const auto robot_model =
    move_group.getRobotModel();

  const auto joint_model_group =
    robot_model->getJointModelGroup(
      "manipulator"
    );

  if (joint_model_group == nullptr)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "No se encontró el grupo manipulator"
    );

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();

    return 1;
  }

  const auto pre_pick_values =
    move_group.getNamedTargetValues(
      "pre_pick"
    );

  moveit::core::RobotState state(
    robot_model
  );

  state.setToDefaultValues();

  state.setVariablePositions(
    pre_pick_values
  );

  state.update();

  const auto link_model =
    state.getLinkModel(
      "tool0"
    );

  if (link_model == nullptr)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "No se encontró el link tool0"
    );

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();

    return 1;
  }

  Eigen::MatrixXd jacobian;

  const bool success =
    state.getJacobian(
      joint_model_group,
      link_model,
      Eigen::Vector3d::Zero(),
      jacobian
    );

  if (!success)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "No fue posible calcular el Jacobiano"
    );

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();

    return 1;
  }

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "Jacobiano de MoveIt en pre_pick:\n"
    << jacobian
  );

  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();

  return 0;
}