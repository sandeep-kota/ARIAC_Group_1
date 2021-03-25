#ifndef SENSORCONTROL_H
#define SENSORCONTROL_H

#include <string>
#include <vector>
#include <array>
#include <unordered_map>

#include <nist_gear/LogicalCameraImage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <ros/ros.h>
#include <ros/console.h>
#include "utils.h"

/**
 * @brief Enumeration of the different types of parts
 */
enum part_code
{
  kDisk,
  kPulley,
  kPiston,
  kGear,
  kGasket
};

/**
 * @brief Enumeration of the different colors of parts
 */
enum color_code
{
  kRed,
  kBlue,
  kGreen
};

class SensorControl
{

public:
  explicit SensorControl(ros::NodeHandle &node);
  void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg, int sensor_n);
  void quality_cntrl_sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg, int sensor_n);
  void init();
  Part findPart(std::string type);
  void save_part_array(std::string type, std::string color, part part);
  part_code hashit_type(std::string const &partString);
  color_code hashit_color(std::string const &colorString);

  geometry_msgs::Pose frame_to_world(int i, geometry_msgs::Pose original_pos, geometry_msgs::TransformStamped c_w_transform);
  void print_available_parts();
  bool read_all_sensors_ = false;
  bool faulty_parts_ = false;

  std::vector<Product> getFaultyProducts()
  {
    return faultyProducts;
  }

  void clearFaultyProducts()
  { // as of now there will only be one and that will be processed then and there.
    faultyProducts.erase(faultyProducts.begin());
  }

  bool isFaultyPartDetected()
  {
    return faultyPartDetected;
  }

  void setFaultyPartDetectedFlag(bool isPartFaulty)
  {
    faultyPartDetected = isPartFaulty;
  }

private:
  ros::NodeHandle node_;

  ros::Subscriber logical_camera_0_subcriber_;
  ros::Subscriber logical_camera_1_subcriber_;
  ros::Subscriber logical_camera_2_subcriber_;
  ros::Subscriber logical_camera_3_subcriber_;
  ros::Subscriber logical_camera_4_subcriber_;
  ros::Subscriber logical_camera_5_subcriber_;
  ros::Subscriber logical_camera_6_subcriber_;
  ros::Subscriber logical_camera_7_subcriber_;
  ros::Subscriber logical_camera_8_subcriber_;
  ros::Subscriber logical_camera_9_subcriber_;
  ros::Subscriber logical_camera_10_subcriber_;
  ros::Subscriber logical_camera_11_subcriber_;
  ros::Subscriber logical_camera_12_subcriber_;
  ros::Subscriber logical_camera_13_subcriber_;
  ros::Subscriber logical_camera_14_subcriber_;
  ros::Subscriber logical_camera_15_subcriber_;
  ros::Subscriber logical_camera_16_subcriber_;

  ros::Subscriber quality_ctrl_sensor1_subs;
  ros::Subscriber quality_ctrl_sensor2_subs;

  std::array<int, 17> logic_call_{0};

  std::array<geometry_msgs::TransformStamped, 17> c_w_transforms_{};
  std::array<geometry_msgs::TransformStamped, NUM_QUALITY_SENSORS> qualitySensorsTransforms{};

  std::array<std::array<std::vector<part>, 3>, 5> parts_{}; //Datastructure to store the info from each part detected by the sensors

  // std::array <std::array <std::vector < part >, 3>, 5> current_parts_ {};

  std::vector<Product> faultyProducts;
  bool faultyPartDetected = false;
};

#endif
