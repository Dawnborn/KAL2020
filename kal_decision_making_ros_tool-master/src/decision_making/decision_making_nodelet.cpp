#include "decision_making.hpp"
#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace kal_decision_making_ros_tool {

class DecisionMakingNodelet : public nodelet::Nodelet {

  inline void onInit() override {
    impl_ = std::make_unique<DecisionMaking>(getPrivateNodeHandle());
  }
  std::unique_ptr<DecisionMaking> impl_;
};
} // namespace kal_decision_making_ros_tool

PLUGINLIB_EXPORT_CLASS(kal_decision_making_ros_tool::DecisionMakingNodelet,
                       nodelet::Nodelet);
