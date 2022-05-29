#pragma once
#include <Eigen/Eigen>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "kal_trajectory_planner_ros_tool/TrajectoryPlannerInterface.h"
#include "kal_decision_making_ros_tool/Decision.h"

namespace kal_trajectory_planner_ros_tool {

class TrajectoryPlanner {
public:
    using Interface = TrajectoryPlannerInterface;
    using Msg = std_msgs::Header;

    explicit TrajectoryPlanner(const ros::NodeHandle& nhPrivate);

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void callbackTimer(const ros::TimerEvent&);
    void reconfigureCallback(const Interface::Config& config, uint32_t /*level*/);
    void decisionCallback(const kal_decision_making_ros_tool::Decision::ConstPtr& msg);

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;
    ros::Timer trajectory_generation_timer;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    nav_msgs::Path current_path_;

    bool stop_command;
};
} // namespace kal_trajectory_planner_ros_tool