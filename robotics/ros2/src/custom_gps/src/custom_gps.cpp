/*! @package custom_gps
    Code Information:
        Maintainer: Wilmer Garzon
        Mail: garzonwilmer5@gmail.com
        Develop by: Wilmer Garzon
*/
#include "custom_gps/custom_gps.hpp"
#define PI 3.1516
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CustomGPS::on_configure(
    const rclcpp_lifecycle::State &)
{
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    // Publishers
    m_custom_gps_pub = this->create_publisher<usr_msgs::msg::LocationMsg>("/custom_gps", default_qos);

    // Subscribers
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", default_qos,
                                                                 std::bind(&CustomGPS::ImuCb, this, _1));
    m_router_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/wifi_geo/fix", default_qos,
                                                                 std::bind(&CustomGPS::RouterGpsCb, this, _1));
    m_main_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/fix", default_qos,
                                                                 std::bind(&CustomGPS::MainGpsCb, this, _1));

    m_custom_gps_msg.roll = m_imu_roll;
    m_custom_gps_msg.pitch = m_imu_pitch;
    m_custom_gps_msg.yaw = m_imu_yaw;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CustomGPS::on_activate(
    const rclcpp_lifecycle::State &)
{
    m_custom_gps_pub->on_activate();
    m_state = TRANSITION_ACTIVATE;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CustomGPS::on_deactivate(
    const rclcpp_lifecycle::State &)
{  
    m_custom_gps_pub->on_deactivate();
    m_state = TRANSITION_DEACTIVATE;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CustomGPS::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    m_state = TRANSITION_CLEANUP;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CustomGPS::on_shutdown(
    const rclcpp_lifecycle::State &)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Subscribers callbacks

void CustomGPS::ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(m_imu_roll, m_imu_pitch, m_imu_yaw);
    m_imu_omega = msg->angular_velocity.z;

    m_custom_gps_msg.roll = m_imu_roll;
    m_custom_gps_msg.pitch = m_imu_pitch;
    m_custom_gps_msg.yaw = m_imu_yaw;
}

void CustomGPS::RouterGpsCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    m_wifi_time = msg->header.stamp;
    m_custom_gps_msg.latitude = msg->latitude;
    m_custom_gps_msg.longitude = msg->longitude;
    m_custom_gps_msg.sensor = "WIFI";

    m_custom_gps_pub->publish(m_custom_gps_msg);
}

void CustomGPS::MainGpsCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    m_gps_time = msg->header.stamp;
    if (m_gps_time > m_wifi_time){ // Router signal have priority
        m_custom_gps_msg.latitude = msg->latitude;
        m_custom_gps_msg.longitude = msg->longitude;
        m_custom_gps_msg.sensor = "GPS";

        m_custom_gps_pub->publish(m_custom_gps_msg);
    } 
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto custom_gps_node = std::make_shared<CustomGPS>(options);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(custom_gps_node->get_node_base_interface());

    /* Node set to 30 Hz */
    auto period = std::chrono::milliseconds(33);
    rclcpp::Rate r(period);

    custom_gps_node->trigger_transition(TRANSITION_CONFIGURE);
    custom_gps_node->trigger_transition(TRANSITION_ACTIVATE);
    while (rclcpp::ok())
    {
        executor->spin_some();
        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}