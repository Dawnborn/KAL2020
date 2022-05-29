#include "decision_making.hpp"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "decision_making_node");

  kal_decision_making_ros_tool::DecisionMaking decision_making(
      ros::NodeHandle("~"));

  ros::spin();
  return EXIT_SUCCESS;
}
