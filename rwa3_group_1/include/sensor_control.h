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


class SensorControl 
{

  public:
    explicit SensorControl(ros::NodeHandle & node);
    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg, int sensor_n);
    void init();
    Part findPart(std::string type);
    geometry_msgs::Pose frame_to_world(int i, geometry_msgs::Pose original_pos, geometry_msgs::TransformStamped c_w_transform);

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
    int logic_call_ {0};
    
    std::array <geometry_msgs::TransformStamped, 17> c_w_transforms_ {};

    std::unordered_map <std::string, std::vector<part>> parts_;
    std::unordered_map <std::string, std::vector<part>> current_parts_;
    
};

#endif
