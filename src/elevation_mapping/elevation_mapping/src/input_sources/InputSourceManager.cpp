/*
 *  InputSourceManager.cpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/ElevationMapping.hpp"

namespace elevation_mapping {

InputSourceManager::InputSourceManager(const ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {}

bool InputSourceManager::configureFromRos(const std::string& inputSourcesNamespace) {
  XmlRpc::XmlRpcValue inputSourcesConfiguration;
  if (!nodeHandle_.getParam(inputSourcesNamespace, inputSourcesConfiguration)) {
    ROS_WARN(
        "Could not load the input sources configuration from parameter\n "
        "%s, are you sure it was pushed to the parameter server? Assuming\n "
        "that you meant to leave it empty. Not subscribing to any inputs!\n",
        nodeHandle_.resolveName(inputSourcesNamespace).c_str());
    return false;
  }
  return configure(inputSourcesConfiguration, inputSourcesNamespace);
}

bool InputSourceManager::configure(const XmlRpc::XmlRpcValue& config, const std::string& sourceConfigurationName) {
  if (config.getType() == XmlRpc::XmlRpcValue::TypeArray &&
      config.size() == 0) {  // Use Empty array as special case to explicitly configure no inputs.
    return true;
  }

  if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR(
        "%s: The input sources specification must be a struct. but is of "
        "of XmlRpcType %d",
        sourceConfigurationName.c_str(), config.getType());
    ROS_ERROR("The xml passed in is formatted as follows:\n %s", config.toXml().c_str());
    return false;
  }

  bool successfulConfiguration = true;
  std::set<std::string> subscribedTopics;

  // AIR-GROUND EXTENSION (mobile_manipulator): robot_base_frame_id is PER INPUT SOURCE.
  //
  // Upstream reads it once, node-wide, and hands the same GeneralParameters to every
  // source. But this frame is the PIVOT OF THE POSE-COVARIANCE LEVER ARM: SensorProcessorBase
  // looks up base->sensor and map->base and propagates the robot pose covariance through that
  // arm into each point's height variance. One node-wide value therefore cannot serve two
  // sensors carried by two different bodies.
  //
  // That is exactly our case: the UGV's Velodyne pivots at base_link (arm ~0.7 m) and the
  // UAV's pivots at uav0/base_link (arm ~0.1 m). Sharing one pivot means either breaking the
  // UGV source, or propagating the UAV's covariance through a ~20 m arm from the ground
  // vehicle's chassis to an airborne sensor, which inflates the variance into nonsense.
  //
  // So each source may now override it in its own namespace:
  //     input_sources:
  //       lidar: { robot_base_frame_id: base_link,      ... }
  //       uav:   { robot_base_frame_id: uav0/base_link, ... }
  // Omitted -> falls back to the node-wide value, so existing configs behave identically.
  const std::string nodeWideBaseFrame{nodeHandle_.param("robot_base_frame_id", std::string("/robot"))};
  const std::string mapFrame{nodeHandle_.param("map_frame_id", std::string("/map"))};

  // Configure all input sources in the list.
  for (const auto& inputConfig : config) {
    ros::NodeHandle sourceNodeHandle{nodeHandle_.resolveName(sourceConfigurationName + "/" + inputConfig.first)};

    std::string sourceBaseFrame{nodeWideBaseFrame};
    if (sourceNodeHandle.getParam("robot_base_frame_id", sourceBaseFrame) && sourceBaseFrame != nodeWideBaseFrame) {
      ROS_INFO("Input source '%s': robot_base_frame_id overridden to '%s' (node-wide is '%s').", inputConfig.first.c_str(),
               sourceBaseFrame.c_str(), nodeWideBaseFrame.c_str());
    }
    const SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{sourceBaseFrame, mapFrame};

    Input source{sourceNodeHandle};

    const bool configured{source.configure(inputConfig.first, inputConfig.second, generalSensorProcessorConfig)};
    if (!configured) {
      successfulConfiguration = false;
      continue;
    }

    if (!source.isEnabled()) {
      continue;
    }

    const std::string subscribedTopic{source.getSubscribedTopic()};
    const bool topicIsUnique{subscribedTopics.insert(subscribedTopic).second};

    if (topicIsUnique) {
      sources_.push_back(std::move(source));
    } else {
      ROS_WARN(
          "The input sources specification tried to subscribe to %s "
          "multiple times. Only subscribing once.",
          subscribedTopic.c_str());
      successfulConfiguration = false;
    }
  }

  return successfulConfiguration;
}

int InputSourceManager::getNumberOfSources() {
  return static_cast<int>(sources_.size());
}

}  // namespace elevation_mapping