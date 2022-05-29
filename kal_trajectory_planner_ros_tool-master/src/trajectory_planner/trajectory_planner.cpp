#include "trajectory_planner.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/range/algorithm/min_element.hpp>
namespace kal_trajectory_planner_ros_tool {

/**
 * Initialization
 */
TrajectoryPlanner::TrajectoryPlanner(const ros::NodeHandle& nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    interface_.fromParamServer();

    /*
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/TrajectoryPlanner.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&TrajectoryPlanner::reconfigureCallback, this, _1, _2));
    interface_.path_subscriber->registerCallback(&TrajectoryPlanner::pathCallback, this);
    interface_.decision_subscriber->registerCallback(&TrajectoryPlanner::decisionCallback, this);

    trajectory_generation_timer =
        nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &TrajectoryPlanner::callbackTimer, this);

    rosinterface_handler::showNodeInfo();

    /*
     * The preferred way of logging is to call the logging functions of the interface object.
     * These also work for nodelets and are better than using ROS_DEBUG (etc.) macros.
     */
    interface_.logDebug("Node initialized.");
    stop_command = false;
}


void TrajectoryPlanner::pathCallback(const nav_msgs::Path::ConstPtr& path) {


    if (path->poses.size() < 2)
        return;
    // save the current path
    current_path_.poses.emplace_back(path->poses[0]);
    Eigen::Vector2d last_point = Eigen::Vector2d(path->poses[0].pose.position.x, path->poses[0].pose.position.y);
    double distance_between_points = 0;
    for (int i = 1; i < (int)path->poses.size(); i++) {
        double x = path->poses[i].pose.position.x;
        double y = path->poses[i].pose.position.y;
        distance_between_points += (last_point - Eigen::Vector2d(x, y)).norm();
        if (distance_between_points > interface_.trajectory_points_distance) {
            current_path_.poses.emplace_back(path->poses[i]);
            distance_between_points = 0.;
            last_point = Eigen::Vector2d(x, y);
        }
    }
}

void TrajectoryPlanner::callbackTimer(const ros::TimerEvent&) {
    if (current_path_.poses.size() == 0) { // check if the path is received
        ROS_WARN_STREAM("Trajectory Planner:: No Path received yet");
        return;
    }
    // get the vehicle pose from tf
    Eigen::Isometry3d vehicle_pose;
    try {
        const geometry_msgs::TransformStamped tf_ros =
            tfBuffer_.lookupTransform(interface_.map_frame, interface_.vehicle_frame, ros::Time(0));
        vehicle_pose = tf2::transformToEigen(tf_ros);
    } catch (const tf2::TransformException& e) {
        ROS_WARN_STREAM(e.what());
        return;
    }
    // get vehicle 2d position
    const Eigen::Vector2d vehicle_position = vehicle_pose.translation().head<2>();

    // find closest point on the global path
    auto it = boost::range::min_element(current_path_.poses, [&vehicle_position](const auto& le, const auto& re) {
        Eigen::Vector2d left = Eigen::Vector2d(le.pose.position.x, le.pose.position.y);
        Eigen::Vector2d right = Eigen::Vector2d(re.pose.position.x, re.pose.position.y);
        return (left - vehicle_position).squaredNorm() < (right - vehicle_position).squaredNorm();
    });

    // generate trajectory as stamped poses (poses for specific times in the future)
    nav_msgs::Path trajectory;
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = interface_.map_frame;
    // add closest pose at path as the first pose of trajectory which is planned for the current time
    geometry_msgs::PoseStamped first_pose;
    first_pose.pose = (it)->pose;
    first_pose.header.stamp = ros::Time::now();
    first_pose.header.frame_id = interface_.map_frame;
    trajectory.poses.push_back(first_pose);
    
    double final_speed = interface_.desired_speed;
    if(stop_command == true)
        final_speed = 0;

    for (unsigned int i = 1; i < 20 and it + i < current_path_.poses.end(); i++) {
        geometry_msgs::PoseStamped new_pose;
        new_pose.pose = (it + i)->pose;
        // if desired speed from user is close to zero, set pose stamps as current time to stop the vehicle
        if (final_speed < 0.5) {
            new_pose.header.stamp = ros::Time::now();
        } else {
            // calculate distance between two poses and according to this distance and desired speed set the time stamp
            Eigen::Vector2d current_2d = Eigen::Vector2d(new_pose.pose.position.x, new_pose.pose.position.y);
            Eigen::Vector2d last_2d =
                Eigen::Vector2d(trajectory.poses[i - 1].pose.position.x, trajectory.poses[i - 1].pose.position.y);
            double distance_to_last_pose = (current_2d - last_2d).norm();
            double time_diff = distance_to_last_pose / final_speed;
            new_pose.header.stamp = trajectory.poses[i - 1].header.stamp + ros::Duration(time_diff);
        }
        new_pose.header.frame_id = interface_.map_frame;
        trajectory.poses.push_back(new_pose);
    }
    // publish the trajectory for the controller
    interface_.trajectory_publisher.publish(trajectory);
}

/**
 * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window.
 * The parameter "level" is unused here. It is set to the number of changes in the config.
 * At startup, the callback is automatically called with level == std::numeric_limits<uint32_t>::max().
 */
void TrajectoryPlanner::reconfigureCallback(const Interface::Config& config, uint32_t /*level*/) {
    interface_.fromConfig(config);
}

void TrajectoryPlanner::decisionCallback(const kal_decision_making_ros_tool::Decision::ConstPtr& msg)
{
    if(msg->decision==0)
        stop_command = true;
    else
        stop_command = false;
    return;
}

} // namespace kal_trajectory_planner_ros_tool