#pragma once
#include "carla_msgs/CarlaTrafficRules.h"
#include "kal_decision_making_ros_tool/DecisionMakingInterface.h"
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/forwards.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/**
 inference part in modified version from:

 https://github.com/spmallick/learnopencv/tree/master/ObjectDetection-YOLO

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2000-2018, Intel Corporation, all rights reserved.
Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
Copyright (C) 2009-2016, NVIDIA Corporation, all rights reserved.
Copyright (C) 2010-2013, Advanced Micro Devices, Inc., all rights reserved.
Copyright (C) 2015-2016, OpenCV Foundation, all rights reserved.
Copyright (C) 2015-2016, Itseez Inc., all rights reserved.
Third party copyrights are property of their respective owners.
 */

namespace kal_decision_making_ros_tool {

class DecisionMaking {
public:
  using Interface = DecisionMakingInterface;
  using Msg = std_msgs::Header;

  explicit DecisionMaking(const ros::NodeHandle &nhPrivate);

private:
  void messageCallback(const Msg::ConstPtr &msg);
  void reconfigureCallback(const Interface::Config &config, uint32_t /*level*/);
  void trafficrulesCallback(
      const carla_msgs::CarlaTrafficRules::ConstPtr &traffic_rule_msg);
  void cameraImageCallback(const sensor_msgs::ImageConstPtr &image_msg);
  std::vector<cv::String> getOutputsNames(const cv::dnn::Net &net);
  int postprocess(cv::Mat &frame, const std::vector<cv::Mat> &outs);
  void drawPred(int classId, float conf, int left, int top, int right,
                int bottom, cv::Mat &frame);

  Interface interface_;
  dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_{tfBuffer_};
  tf2_ros::TransformBroadcaster tfBroadcaster_;

  bool trafficlights;
  bool crosswalk;
  int counter;
  int flag;

  // YoloV3-tiny parameters and network
  float confThreshold;
  float nmsThreshold;
  int inpWidth;
  int inpHeight;

  std::vector<std::string> classes;
  std::string classesFile;
  std::string line;

  std::string modelConfiguration;
  std::string modelWeights;

  cv::dnn::Net net;

}; // namespace kal_decision_making_ros_tool

}; // namespace kal_decision_making_ros_tool