/*! @package lifecycle_manager
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "lifecycle_manager/lifecycle_manager.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto lifecycle_node = std::make_shared<LifeCycleManager>(options);

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(lifecycle_node->get_node_base_interface());

    lifecycle_node->trigger_transition(TRANSITION_CONFIGURE);
    lifecycle_node->trigger_transition(TRANSITION_ACTIVATE);

    lifecycle_node->add_activation("speed_controller");
    lifecycle_node->add_activation("wheel_odometry");
    lifecycle_node->add_activation("rpm_converter");
    lifecycle_node->add_activation("custom_gps");

    executor->spin();

    rclcpp::shutdown();

    return 0;
}
