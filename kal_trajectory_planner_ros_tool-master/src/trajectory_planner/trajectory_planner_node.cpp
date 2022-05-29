#include "trajectory_planner.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "trajectory_planner_node");

    kal_trajectory_planner_ros_tool::TrajectoryPlanner trajectory_planner(ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
