#include <ros/ros.h>

#include "sensor_control.h"
#include "utils.h"

SensorControl::SensorControl(ros::NodeHandle &node)
{

    node_ = node;
    
}
void SensorControl::init()
{
    logical_camera_0_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_0", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 0));
    logical_camera_1_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_1", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 1));
      logical_camera_2_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_2", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 2));
      logical_camera_3_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_3", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 3));
      logical_camera_4_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_4", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 4));
      logical_camera_5_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_5", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 5));
      logical_camera_6_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_6", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 6));
      logical_camera_7_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_7", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 7));
      logical_camera_8_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_8", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 8));
      logical_camera_9_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_9", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 9));
      logical_camera_10_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_10", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 10));
      logical_camera_11_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_11", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 11));
      logical_camera_12_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_12", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 12));
      logical_camera_13_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_13", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 13));
      logical_camera_14_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_14", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 14));
      logical_camera_15_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_15", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 15));
      logical_camera_16_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_16", 1, boost::bind(&SensorControl::logical_camera_callback, this, _1, 16));

      //Get transforms world to logical camera sensors
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Duration timeout(5.0);

        geometry_msgs::TransformStamped transformStamped;    

        for (int i = 0; i < 17; i++) {
            try{
            transformStamped = tfBuffer.lookupTransform("world", "logical_camera_" + std::to_string(i) +"_frame",
                                    ros::Time(0), timeout);
            }
            catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
            }
            //Initialize attribute that stores the frame transforms to world of each camera
            c_w_transforms_.at(i) = transformStamped;
  }

}
  /**
   * @brief Maps part type to keys. 
   */
  part_code SensorControl::hashit_type (std::string const& partString) {
    if (partString == "disk") return kDisk;
    if (partString == "pulley") return kPulley;
    if (partString == "gasket") return kGasket;
    if (partString == "piston") return kPiston;
    if (partString == "gear") return kGear;
  }

  /**
   * @brief Maps part color to keys. 
   */
  color_code SensorControl::hashit_color (std::string const& colorString) {
    if (colorString == "red") return kRed;
    if (colorString == "blue") return kBlue;
    if (colorString == "green") return kGreen;
  }
void SensorControl::logical_camera_callback (const nist_gear::LogicalCameraImage::ConstPtr &msg, int sensor_n)
{
    
    int pos_t {};
    int pos_c {};
    int ktype {};
    int kcolor {};
    std::string type {};
    std::string color {};
    part Part;
    logic_call_ ++;

    // ROS_WARN_STREAM(logic_call_);


    if (msg->models.size() > 0 && sensor_n != 0 && sensor_n != 1) {
      for (int i = 0; i < msg->models.size(); i++)
        {

          pos_t = msg->models.at(i).type.find("_");
          pos_c = msg->models.at(i).type.rfind("_");

          type = msg->models.at(i).type.substr(0,pos_t);
          color = msg->models.at(i).type.substr(pos_c+1);

          ktype = hashit_type(type);
          kcolor = hashit_color(color);

          Part.type = msg->models.at(i).type;
          Part.pose = frame_to_world(i, msg->models.at(i).pose, c_w_transforms_.at(sensor_n));
          Part.save_pose = Part.pose;
          Part.frame = "logical_camera_" + std::to_string(sensor_n) + "_frame";
          Part.time_stamp = ros::Time::now();
          Part.id = Part.type + std::to_string(parts_.at(ktype).at(kcolor).size());
          if (sensor_n == 3 || sensor_n == 4 || sensor_n == 5 || sensor_n == 6)
          {
            Part.location = "bins";
          } else if (sensor_n == 9 || sensor_n == 10)
          {
              Part.location = "shelf_2";
          } else if (sensor_n == 7 || sensor_n == 8)
          {
              Part.location = "shelf_1";
          } else if (sensor_n == 13 || sensor_n == 14)
          {
              Part.location = "shelf_5";
          } else if (sensor_n == 11 || sensor_n == 12)
          {
              Part.location = "shelf_8";
          } else if (sensor_n == 15 || sensor_n == 16)
          {
              Part.location = "shelf_11";
          } else if (sensor_n == 0) 
          {
              Part.location = "agv_1";
          } else if (sensor_n == 1)
          {
              Part.location = "agv_2";
          } else 
          {
              Part.location = "conveyor_belt";
          }

          parts_.at(ktype).at(kcolor).push_back(Part); 
          
          }
    }

    // Once every logical camera sensor callback is runned (17) print the output and reset the parts_ data structure

    // ROS_WARN_STREAM("BEFORE IF");
    if (logic_call_ >= 17)
    {   
      // ROS_WARN_STREAM("INSIDE IF");
        current_parts_ = parts_;
        
        for (int p = 0; p < 5; p++) {
        for (int c = 0; c < 3; c++) {
          for (int part = 0; part < parts_.at(p).at(c).size(); part++){
          }

          // ROS_WARN_STREAM(current_parts_.at(0).at(0).size());
          parts_.at(p).at(c).clear();

            }
          }
      logic_call_ = 0;
          }       
    }       


geometry_msgs::Pose SensorControl::frame_to_world(int i, geometry_msgs::Pose original_pos, geometry_msgs::TransformStamped c_w_transform)
  {   
    geometry_msgs::PoseStamped pose_target, pose_rel;
    geometry_msgs::Pose world_pose;

    pose_rel.header.frame_id = c_w_transform.header.frame_id + "_part_" + std::to_string(i);
    pose_rel.pose = original_pos;
    tf2::doTransform(pose_rel, pose_target, c_w_transform);

    world_pose.position = pose_target.pose.position;
    world_pose.orientation = pose_target.pose.orientation;

    return world_pose;
  }

Part SensorControl::findPart(std::string type)
{

  int pos_t = type.find("_");
  int pos_c = type.rfind("_");
  std::string parttype = type.substr(0,pos_t);
  std::string partcolor = type.substr(pos_c+1);
  part no_part;

  int ktype = hashit_type(parttype);
  int kcolor = hashit_color(partcolor);
  if(current_parts_.at(ktype).at(kcolor).empty())
  {
    return no_part;
  } else{
    return current_parts_.at(ktype).at(kcolor).at(0);
  }
  
  
}


