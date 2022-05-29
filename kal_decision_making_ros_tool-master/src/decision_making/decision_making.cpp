#include "decision_making.hpp"
#include "kal_decision_making_ros_tool/Decision.h"

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

/**
 * Initialization
 */
DecisionMaking::DecisionMaking(const ros::NodeHandle &nhPrivate)
    : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

  interface_.fromParamServer();

  /*
   * Set up callbacks for subscribers and reconfigure.
   *
   * New subscribers can be created with "add_subscriber" in
   * "cfg/DecisionMaking.if file. Don't forget to register your callbacks here!
   */
  reconfigureServer_.setCallback(
      boost::bind(&DecisionMaking::reconfigureCallback, this, _1, _2));

  interface_.traffic_rule_subscriber->registerCallback(
      &DecisionMaking::trafficrulesCallback, this);
  interface_.camera_image_subscriber->registerCallback(
      &DecisionMaking::cameraImageCallback, this);

  rosinterface_handler::showNodeInfo();

  counter = 0;
  flag = -5;
  trafficlights = false;
  crosswalk = false;

  // Setup YoloV3-tiny for inference

  // Initialize the parameters
  confThreshold = 0.5; // Confidence threshold
  nmsThreshold = 0.4;  // Non-maximum suppression threshold
  inpWidth = 416;      // Width of network's input image
  inpHeight = 416;     // Height of network's input image

  // Load names of classes
  classesFile =
      "/home/kalvm/KAL/kal_ws/src/kal_decision_making_ros_tool/res/classes.txt";
  // classesFile =
  // "/home/kal5-1/kal_ws/src/kal_decision_making_ros_tool/res/classes.txt"; //
  // on Jetson
  std::ifstream ifs(classesFile.c_str());
  while (std::getline(ifs, line))
    classes.push_back(line);

  // Configuration and weight files for the model
  modelConfiguration = "/home/kalvm/KAL/kal_ws/src/"
                       "kal_decision_making_ros_tool/res/kal-yolov3-tiny.cfg";
  modelWeights = "/home/kalvm/KAL/kal_ws/src/kal_decision_making_ros_tool/res/"
                 "kal-yolov3-tiny.weights";
  // modelConfiguration =
  // "/home/kal5-1/kal_ws/src/kal_decision_making_ros_tool/res/kal-yolov3-tiny.cfg";
  // // on Jetson modelWeights =
  // "/home/kal5-1/kal_ws/src/kal_decision_making_ros_tool/res/kal-yolov3-tiny.weights";
  // // on Jetson

  // Load the network
  net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
  // net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA); // uncomment on Jetson
  // // cv::dnn::DNN_BACKEND_OPENCV for cpu
  // net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA); // uncomment on Jetson
  // // cv::dnn::DNN_TARGET_CPU for cpu

  /*
   * The preferred way of logging is to call the logging functions of the
   * interface object. These also work for nodelets and are better than using
   * ROS_DEBUG (etc.) macros.
   */
  interface_.logDebug("Node initialized.");
}

void DecisionMaking::reconfigureCallback(const Interface::Config &config,
                                         uint32_t /*level*/) {
  interface_.fromConfig(config);
}

void DecisionMaking::trafficrulesCallback(
    const carla_msgs::CarlaTrafficRules::ConstPtr &traffic_rule_msg) {
  trafficlights = traffic_rule_msg->traffic_light;
  crosswalk = traffic_rule_msg->zebra_crossing;
}

void DecisionMaking::cameraImageCallback(
    const sensor_msgs::ImageConstPtr &image_msg) {

  Decision decision_msg;
  decision_msg.decision = 1; // by default: drive
  int classId = 5;           // 5 = no detections

  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat frame = cv_ptr->image;
  cv::Mat blob;

  // run inference
  if (trafficlights || crosswalk) {

    // Create a 4D blob from a frame.
    cv::dnn::blobFromImage(frame, blob, 1 / 255.0,
                           cv::Size(inpWidth, inpHeight), cv::Scalar(0, 0, 0),
                           true, false);

    // Sets the input to the network
    net.setInput(blob);

    // Runs the forward pass to get output of the output layers
    std::vector<cv::Mat> outs;
    net.forward(outs, DecisionMaking::getOutputsNames(net));

    // Remove the bounding boxes with low confidence
    classId = DecisionMaking::postprocess(frame, outs);

    cv_ptr->image = frame;
  }

  if (classId == 2 || classId == 3)

    decision_msg.decision = 0; // stop

  else if (classId == 0 || classId == 1)

    decision_msg.decision = 1; // drive

  interface_.debug_image_publisher.publish(cv_ptr->toImageMsg());
  interface_.decision_publisher.publish(decision_msg);
}

// Get the names of the output layers
std::vector<cv::String>
DecisionMaking::getOutputsNames(const cv::dnn::Net &net) {
  static std::vector<cv::String> names;
  if (names.empty()) {
    // Get the indices of the output layers, i.e. the layers with unconnected
    // outputs
    std::vector<int> outLayers = net.getUnconnectedOutLayers();

    // get the names of all the layers in the network
    std::vector<cv::String> layersNames = net.getLayerNames();

    // Get the names of the output layers in names
    names.resize(outLayers.size());
    for (size_t i = 0; i < outLayers.size(); ++i)
      names[i] = layersNames[outLayers[i] - 1];
  }
  return names;
}

// Remove the bounding boxes with low confidence using non-maxima suppression
int DecisionMaking::postprocess(cv::Mat &frame,
                                const std::vector<cv::Mat> &outs) {
  std::vector<int> classIds;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  int classId = 5; // 5 = no detection -> car won't update decision

  for (size_t i = 0; i < outs.size(); ++i) {
    // Scan through all the bounding boxes output from the network and keep only
    // the ones with high confidence scores. Assign the box's class label as the
    // class with the highest score for the box.
    float *data = (float *)outs[i].data;
    for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
      cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
      cv::Point classIdPoint;
      double confidence;
      // Get the value and location of the maximum score
      minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
      if (confidence > confThreshold) {
        int centerX = (int)(data[0] * frame.cols);
        int centerY = (int)(data[1] * frame.rows);
        int width = (int)(data[2] * frame.cols);
        int height = (int)(data[3] * frame.rows);
        int left = centerX - width / 2;
        int top = centerY - height / 2;

        classIds.push_back(classIdPoint.x);
        confidences.push_back((float)confidence);
        boxes.push_back(cv::Rect(left, top, width, height));
      }
    }
  }

  // Perform non maximum suppression to eliminate redundant overlapping boxes
  // with lower confidences
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
  for (size_t i = 0; i < indices.size(); ++i) {
    int idx = indices[i];
    cv::Rect box = boxes[idx];
    DecisionMaking::drawPred(classIds[idx], confidences[idx], box.x, box.y,
                             box.x + box.width, box.y + box.height, frame);
  }

  if (indices.empty()) {
    return 5; // 5 = no detection -> car won't update decision
  }

  int max_conf =
      std::distance(confidences.begin(),
                    std::max_element(confidences.begin(), confidences.end()));
  classId = classIds[max_conf]; // argmax of confidences
  return classId;
}

// Draw the predicted bounding box
void DecisionMaking::drawPred(int classId, float conf, int left, int top,
                              int right, int bottom, cv::Mat &frame) {
  // Draw a rectangle displaying the bounding box
  rectangle(frame, cv::Point(left, top), cv::Point(right, bottom),
            cv::Scalar(255, 178, 50), 3);

  // Get the label for the class name and its confidence
  std::string label = cv::format("%.2f", conf);
  if (!classes.empty()) {
    CV_Assert(classId < (int)classes.size());
    label = classes[classId] + ":" + label;
  }

  // Display the label at the top of the bounding box
  int baseLine;
  cv::Size labelSize =
      getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
  top = cv::max(top, labelSize.height);
  cv::rectangle(frame, cv::Point(left, top - round(1.5 * labelSize.height)),
                cv::Point(left + round(1.5 * labelSize.width), top + baseLine),
                cv::Scalar(255, 255, 255), cv::FILLED);
  cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX,
              0.75, cv::Scalar(0, 0, 0), 1);
}

} // namespace kal_decision_making_ros_tool
