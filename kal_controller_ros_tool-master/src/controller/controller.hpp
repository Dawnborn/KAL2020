#pragma once
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "kal_controller_ros_tool/ControllerInterface.h"

namespace kal_controller_ros_tool {

class Controller {
public:
    using Interface = ControllerInterface;
    using Msg = std_msgs::Header;

    explicit Controller(const ros::NodeHandle& nhPrivate);

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void controlLoopCallback(const ros::TimerEvent& timer_event);
    void messageCallback(const Msg::ConstPtr& msg);
    void reconfigureCallback(const Interface::Config& config, uint32_t /*level*/);

    std::vector<Eigen::Affine3d> path_;
    ros::Publisher servo_command_publisher_;
    ros::Subscriber path_subscriber_;
    ros::Timer control_loop_timer_;
    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    double desired_speed_;
};
} // namespace kal_controller_ros_tool
