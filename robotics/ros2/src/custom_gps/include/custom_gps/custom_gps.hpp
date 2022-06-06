/*! @package custom_gps
    Code Information:
        Maintainer: Wilmer Garzon
        Mail: garzonwilmer5@gmail.com
        Develop by: Wilmer Garzon
*/
#ifndef CUSTOM_GPS_CAN_H_INCLUDED
#define CUSTOM_GPS_CAN_H_INCLUDED

// STD libraries
#include <math.h>
#include <memory>
#include <utility>
#include <vector>

// ROS2 Default
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

// TF2 Transformations
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

// Custom libraries
#include "utils/console.hpp"

// ROS2 Messages
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

// Custom Messages
#include "usr_msgs/msg/location_msg.hpp"

#define TRANSITION_CONFIGURE 1
#define TRANSITION_CLEANUP 2
#define TRANSITION_ACTIVATE 3
#define TRANSITION_DEACTIVATE 4

using std::placeholders::_1;

class CustomGPS : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
    /*!
        Constructor
        @param options: rclcpp::NodeOptions.
    */
    explicit CustomGPS(rclcpp::NodeOptions &options) : CascadeLifecycleNode("custom_gps", options) {}

    /*!
        Destructor
    */
    ~CustomGPS(){};

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &);

    uint8_t m_state = TRANSITION_ACTIVATE;

private:
    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<usr_msgs::msg::LocationMsg>::SharedPtr
        m_custom_gps_pub; /*!< Publish at topic /custom_gps */


    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;           /*!< @sa ImuCb() */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_router_gps_sub; /*!< @sa RouterGpsCb() */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_main_gps_sub;     /*!< @sa MainGpsCb() */

    // Subscribers callbacks
    /*!
        IMU information subscription callback for the topic /imu/data
        @param msg: IMU message
        @return void
    */
    void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg);

    /*!
        Router GPS information subscription callback for the topic /wifi_geo/fix
        @param msg: NavSatFix message
        @return void
    */
    void RouterGpsCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /*!
        Main GPS information subscription callback for the topic /fix
        @param msg: NavSatFix message
        @return void
    */
    void MainGpsCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);


    // Member attributes
    usr_msgs::msg::LocationMsg m_custom_gps_msg;

    rclcpp::Time m_wifi_time;
    rclcpp::Time m_gps_time;

    // Variables
    double m_imu_roll = 0.0f;
    double m_imu_pitch = 0.0f;
    double m_imu_yaw = 0.0f;
};

#endif