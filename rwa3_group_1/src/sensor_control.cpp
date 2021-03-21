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

    parts_ = {
            {"piston_rod_part_red", {}}, 
            {"piston_rod_part_green", {}},
            {"piston_rod_part_blue", {}},
            {"pulley_part_red", {}},
            {"pulley_part_green", {}},
            {"pulley_part_blue", {}},
            {"gear_part_red", {}},
            {"gear_part_green", {}},
            {"gear_part_blue", {}},
            {"gasket_part_red", {}},
            {"gasket_part_green", {}},
            {"gasket_part_blue", {}},
            {"disk_part_red", {}},
            {"disk_part_green", {}},
            {"disk_part_blue", {}}
    };

    // current_parts_ = {
    //         {"piston_rod_part_red", {}}, 
    //         {"piston_rod_part_green", {}},
    //         {"piston_rod_part_blue", {}},
    //         {"pulley_part_red", {}},
    //         {"pulley_part_green", {}},
    //         {"pulley_part_blue", {}},
    //         {"gear_part_red", {}},
    //         {"gear_part_green", {}},
    //         {"gear_part_blue", {}},
    //         {"gasket_part_red", {}},
    //         {"gasket_part_green", {}},
    //         {"gasket_part_blue", {}},
    //         {"disk_part_red", {}},
    //         {"disk_part_green", {}},
    //         {"disk_part_blue", {}}
    // };
    

}
void SensorControl::logical_camera_callback (const nist_gear::LogicalCameraImage::ConstPtr &msg, int sensor_n)
{
    part Part;
    logic_call_ ++;
    if (msg->models.size() > 0) {
      for (int i = 0; i < msg->models.size(); i++)
        {
          Part.type = msg->models.at(i).type;
          Part.pose = frame_to_world(i, msg->models.at(i).pose, c_w_transforms_.at(sensor_n));
          Part.save_pose = Part.pose;
          Part.frame = "logical_camera_" + std::to_string(sensor_n) + "_frame";
          Part.time_stamp = ros::Time::now();
          Part.id = Part.type + std::to_string(parts_.at(Part.type).size());
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

          parts_.at(Part.type).push_back(Part);         
          
          }
    }

    // Once every logical camera sensor callback is runned (17) print the output and reset the parts_ data structure
    if (logic_call_ == 17)
    {   

        // current_parts_.at("piston_rod_part_red").clear(); 
        // current_parts_.at("piston_rod_part_green").clear();
        // current_parts_.at("piston_rod_part_blue").clear();
        // current_parts_.at("pulley_part_red").clear();
        // current_parts_.at("pulley_part_green").clear();
        // current_parts_.at("pulley_part_blue").clear();
        // current_parts_.at("gear_part_red").clear();
        // current_parts_.at("gear_part_green").clear();
        // current_parts_.at("gear_part_blue").clear();
        // current_parts_.at("gasket_part_red").clear();
        // current_parts_.at("gasket_part_green").clear();
        // current_parts_.at("gasket_part_blue").clear();
        // current_parts_.at("disk_part_red").clear();
        // current_parts_.at("disk_part_green").clear();
        // current_parts_.at("disk_part_blue").clear();

        // current_parts_.at("piston_rod_part_red") = parts_.at("piston_rod_part_red"); 
        // current_parts_.at("piston_rod_part_green") = parts_.at("piston_rod_part_green");
        // current_parts_.at("piston_rod_part_blue") = parts_.at("piston_rod_part_blue");
        // current_parts_.at("pulley_part_red") = parts_.at("pulley_part_red");
        // current_parts_.at("pulley_part_green") = parts_.at("pulley_part_green");
        // current_parts_.at("pulley_part_blue") = parts_.at("pulley_part_blue");
        // current_parts_.at("gear_part_red") = parts_.at("gear_part_red");
        // current_parts_.at("gear_part_green") = parts_.at("gear_part_green") ;
        // current_parts_.at("gear_part_blue") = parts_.at("gear_part_blue");
        // current_parts_.at("gasket_part_red") = parts_.at("gasket_part_red");
        // current_parts_.at("gasket_part_green") = parts_.at("gasket_part_green");
        // current_parts_.at("gasket_part_blue") = parts_.at("gasket_part_blue");
        // current_parts_.at("disk_part_red") = parts_.at("disk_part_red");
        // current_parts_.at("disk_part_green") = parts_.at("disk_part_green");
        // current_parts_.at("disk_part_blue") = parts_.at("disk_part_blue");

        parts_.at("piston_rod_part_red").clear(); 
        parts_.at("piston_rod_part_green").clear();
        parts_.at("piston_rod_part_blue").clear();
        parts_.at("pulley_part_red").clear();
        parts_.at("pulley_part_green").clear();
        parts_.at("pulley_part_blue").clear();
        parts_.at("gear_part_red").clear();
        parts_.at("gear_part_green").clear();
        parts_.at("gear_part_blue").clear();
        parts_.at("gasket_part_red").clear();
        parts_.at("gasket_part_green").clear();
        parts_.at("gasket_part_blue").clear();
        parts_.at("disk_part_red").clear();
        parts_.at("disk_part_green").clear();
        parts_.at("disk_part_blue").clear();            
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
  return parts_.at(type).at(0);
}