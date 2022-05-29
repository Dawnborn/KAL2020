#include "controller.hpp"
#include <ackermann_msgs/AckermannDrive.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/range/algorithm/min_element.hpp>
#include "discrete_curvature.h"
#include "safe_iterator_operations.h"
namespace kal_controller_ros_tool {

/**
 * Initialization
 */
Controller::Controller(const ros::NodeHandle& nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, tfListener_{tfBuffer_} {

    interface_.fromParamServer();
    desired_speed_ = 0.;
    /*
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/Controller.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&Controller::reconfigureCallback, this, _1, _2));
    interface_.path_subscriber->registerCallback(&Controller::pathCallback, this);

    rosinterface_handler::showNodeInfo();

    /*
     * The preferred way of logging is to call the logging functions of the interface object.
     * These also work for nodelets and are better than using ROS_DEBUG (etc.) macros.
     */
    interface_.logDebug("Node initialized.");

    //    path_subscriber = nhPrivate.subscribe(interface_.path_topic,
    //                                                interface_.msg_queue_size,
    //                                                &Controller::pathCallback,
    //                                                this,
    //                                                ros::TransportHints().tcpNoDelay());
    control_loop_timer_ =
        nhPrivate.createTimer(ros::Rate(interface_.control_loop_rate), &Controller::controlLoopCallback, this);
}

/**
 * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window.
 * The parameter "level" is unused here. It is set to the number of changes in the config.
 * At startup, the callback is automatically called with level == std::numeric_limits<uint32_t>::max().
 */
void Controller::reconfigureCallback(const Interface::Config& config, uint32_t /*level*/) {
    interface_.fromConfig(config);
}

void Controller::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (msg->poses.size() < 5) {
        ROS_ERROR_STREAM("Controller::Trajectory is very short");
    }
    path_.clear();
    // path_.reserve(msg->poses.size());
    double distance_on_path = 0;
    for (const auto& pose_stamped : msg->poses) {
        Eigen::Affine3d pose;
        tf2::fromMsg(pose_stamped.pose, pose);
        if (path_.size() > 0)
            distance_on_path += (path_[path_.size() - 1].translation() - pose.translation()).norm();
        path_.push_back(pose);
    }
    ros::Duration time_diff = msg->poses[msg->poses.size() - 1].header.stamp - msg->poses[0].header.stamp;
    double trajectory_horizen = time_diff.toSec();
    if (trajectory_horizen < 0.1) {
        ROS_WARN_STREAM("Controller::Trajectory horizen too small, stopping the vehicle.");
        desired_speed_ = 0.0;
    } else
        desired_speed_ = distance_on_path / trajectory_horizen;
}

double signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const double vz = boost::math::sign(a.cross(b).z());
    return vz * std::acos(a.normalized().dot(b.normalized()));
}


void Controller::controlLoopCallback(const ros::TimerEvent& timer_event) {
    if (path_.size() < 5) {
        ROS_INFO_STREAM("Controller::No Trajectory received yet");
        return;
    }

    /*
     * Lookup the latest transform from vehicle to map frame
     */
    Eigen::Affine3d vehicle_pose;
    try {
        const geometry_msgs::TransformStamped tf_ros =
            tfBuffer_.lookupTransform(interface_.map_frame, interface_.vehicle_frame, ros::Time(0));
        vehicle_pose = tf2::transformToEigen(tf_ros);
    } catch (const tf2::TransformException& e) {
        ROS_WARN_STREAM(e.what());
        return;
    }

    const Eigen::Vector3d vehicle_position = vehicle_pose.translation();

    /*
     * Shift Rear-axle in current direction -> kos_shift
     */
    const Eigen::Vector3d vehicle_frame_unit_x = [&vehicle_pose]() {
        Eigen::Vector3d p = vehicle_pose.rotation() * Eigen::Vector3d::UnitX();
        p.z() = 0.0;
        return p.normalized();
    }();

    const Eigen::Vector3d shifted_vehicle_position = vehicle_position + vehicle_frame_unit_x * interface_.kos_shift;


    auto const& it = boost::range::min_element(
        path_, [&shifted_vehicle_position](const Eigen::Affine3d& lhs, const Eigen::Affine3d& rhs) {
            return (lhs.translation() - shifted_vehicle_position).squaredNorm() <
                   (rhs.translation() - shifted_vehicle_position).squaredNorm();
        });

    if (it == std::prev(path_.end())) {
        ROS_ERROR("Reached end of trajectory!");
        return;
    }

    const Eigen::Vector3d closest_trajectory_point = it->translation();
    /*
     * Find look ahead point
     */
    //	interface_.index_shift --- lookaheadpoint
    //	interface_.ii_off ---	point offset for curvature approximation
    auto const& it_lb = safe_next(it, interface_.index_shift, std::prev(path_.end()));
    const Eigen::Vector3d& p_lookahead = it_lb->translation();

    // Determine three points for approximation
    const Eigen::Vector3d& p_prev = safe_prev(it_lb, interface_.ii_off, path_.begin())->translation();
    const Eigen::Vector3d& p_next = safe_next(it_lb, interface_.ii_off, std::prev(path_.end()))->translation();

    const double curv = discreteCurvature(p_prev.head<2>(), p_lookahead.head<2>(), p_next.head<2>());

    const Eigen::Vector3d target_direction = p_next - p_lookahead;

    /*
     * Compute angle difference
     */
    const double delta_angle = signedAngleBetween(target_direction, vehicle_frame_unit_x);
    /*
     * Calculation of the sign for distance control (= determine side of trajectory)
     */
    const double vz_dist =
        boost::math::sign(target_direction.cross(closest_trajectory_point - shifted_vehicle_position).z());
    const double dist = (closest_trajectory_point - shifted_vehicle_position).norm();

    /*
     * Controller law
     */
    double r_ang = -1. * interface_.k_ang * delta_angle;
    double r_dist = interface_.k_dist * vz_dist * dist;
    double steering_angle = std::atan(interface_.wheel_base * (curv + r_ang + r_dist));

    steering_angle =
        boost::algorithm::clamp(steering_angle, -interface_.max_steering_angle, interface_.max_steering_angle);

    ROS_DEBUG_STREAM("Steering_angle: " << steering_angle);

    ackermann_msgs::AckermannDrive ackermanMsg;
    ackermanMsg.steering_angle = steering_angle;
    ackermanMsg.acceleration = 0.;
    ackermanMsg.speed = desired_speed_;

    interface_.ackerman_publisher.publish(ackermanMsg);
}

} // namespace kal_controller_ros_tool
