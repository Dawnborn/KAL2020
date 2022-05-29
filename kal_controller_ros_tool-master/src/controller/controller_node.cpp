#include "controller.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "controller_node");

    kal_controller_ros_tool::Controller controller(ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
