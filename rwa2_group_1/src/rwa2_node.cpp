// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <vector>
#include <array>
// #include <boost/bind.hpp>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3
#include "../include/Part.hh"

enum part_code {
    kDisk,
    kPulley,
    kPiston,
    kGear,
    kGasket
};

/**
 * @brief Start the competition
 * Create a service client to /ariac/start_competition
 */
void start_competition(ros::NodeHandle &node)
{
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
      node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists())
  {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv); // Call the start Service.
  if (!srv.response.success)
  { // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  }
  else
  {
    ROS_INFO("Competition started!");
  }
}

/**
 * @brief A simple competition class.
 * 
 * This class can hold state and provide methods that handle incoming data.
 * 
 */
class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle &node)
      : current_score_(0)
  {
  }

  /**
   * @brief Called when a new Message is received on the Topic /ariac/current_score
   * 
   * This function sets the value of the attribute current_score_ 
   * @param msg Message used to set the state of the competition.
   */
  void current_score_callback(const std_msgs::Float32::ConstPtr &msg)
  {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }

  /**
   * @brief Called when a new Message is received on /ariac/competition_state
   * 
   * This function sets the state of the competition to 'done'.
   * @param msg Message used to set the state of the competition.
   */
  void competition_state_callback(const std_msgs::String::ConstPtr &msg)
  {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }

  /**
   * @brief Called when a new Message is received on the Topic /ariac/orders
   * 
   * This function adds the Message to received_orders_
   * 
   * @param msg Message containing information on the order.
   */
  void order_callback(const nist_gear::Order::ConstPtr &msg)
  {
    ROS_INFO_STREAM("Received order:\n" << *msg);
    received_orders_.push_back(*msg);
  }

  geometry_msgs::PoseStamped frame_to_world(int i, geometry_msgs::Pose original_pos, geometry_msgs::TransformStamped c_w_transform)
  {   
    geometry_msgs::PoseStamped pose_target, pose_rel;  

    pose_rel.header.frame_id = c_w_transform.header.frame_id + "_part_" + std::to_string(i);
    pose_rel.pose = original_pos;
    tf2::doTransform(pose_rel, pose_target, c_w_transform);

    return pose_target;
  }

  part_code hashit (std::string const& partString) {
    if (partString == "disk") return kDisk;
    if (partString == "pulley") return kPulley;
    if (partString == "gasket") return kGasket;
    if (partString == "piston") return kPiston;
    if (partString == "gear") return kGear;
  }
  /**
   * @brief Called when a new Message is received on the Topic /ariac/logical_camera_x
   * 
   * This function reports the number of objects detected by a logical camera.
   * 
   * @param msg Message containing information on objects detected by the camera.
   */
  void logical_camera_callback(
      const nist_gear::LogicalCameraImage::ConstPtr &msg, int sensor_n)
  {
    
    logic_call_ ++; 
    geometry_msgs::TransformStamped c_w_transform {};

    c_w_transform.header.frame_id = "world_logical_camera_" + std::to_string(sensor_n) + "_frame";
    c_w_transform.transform.translation.x = msg->pose.position.x;
    c_w_transform.transform.translation.y = msg->pose.position.y;
    c_w_transform.transform.translation.z = msg->pose.position.z;

    c_w_transform.transform.rotation.x = msg->pose.orientation.x;
    c_w_transform.transform.rotation.y = msg->pose.orientation.y;
    c_w_transform.transform.rotation.z = msg->pose.orientation.z;
    c_w_transform.transform.rotation.w = msg->pose.orientation.w;

    if (msg->models.size() > 0) {
      for (int i = 0; i < msg->models.size(); i++)
        {
          int pos_t = msg->models.at(i).type.find("_");
          int pos_c = msg->models.at(i).type.rfind("_");

          std::string type = msg->models.at(i).type.substr(0,pos_t);
          std::string color = msg->models.at(i).type.substr(pos_c+1);
          geometry_msgs::PoseStamped world_pose = frame_to_world(i, msg->models.at(i).pose, c_w_transform);

          rwa2::Part part (type, color, sensor_n, world_pose);

          switch(hashit(type)) {
            case kPiston:
              if (color == "red") {
                parts_.at(0).at(0).push_back(part);
              } else if (color == "blue") {
                parts_.at(0).at(1).push_back(part);
              } else {
                parts_.at(0).at(2).push_back(part);
              }       
              break; 
            case kDisk:
            if (color == "red") {
                parts_.at(1).at(0).push_back(part);
              } else if (color == "blue") {
                parts_.at(1).at(1).push_back(part);
              } else {
                parts_.at(1).at(2).push_back(part);
              }       
              break;
            case kGasket:
              if (color == "red") {
                  parts_.at(2).at(0).push_back(part);
                } else if (color == "blue") {
                  parts_.at(2).at(1).push_back(part);
                } else {
                  parts_.at(2).at(2).push_back(part);
                }       
                break;
            case kGear:
              if (color == "red") {
                  parts_.at(3).at(0).push_back(part);
                } else if (color == "blue") {
                  parts_.at(3).at(1).push_back(part);
                } else {
                  parts_.at(3).at(2).push_back(part);
                }       
                break;
            default:
              if (color == "red") {
                  parts_.at(4).at(0).push_back(part);
                } else if (color == "blue") {
                  parts_.at(4).at(1).push_back(part);
                } else {
                  parts_.at(4).at(2).push_back(part);
                }       
                break;         
            }
          
        }
    }
    if (logic_call_ == 14){
      std::cout << logic_call_ << std::endl;
      for (int p = 0; p < 5; p++) {

        std::cout << "\n\n*****  " + part_names_.at(p) + " (" + std::to_string(parts_.at(p).at(0).size()+parts_.at(p).at(1).size()
                  +parts_.at(p).at(2).size()) + " parts)" + "  *****" <<std::endl;

        for (int c = 0; c < 3; c++) {

          std::cout << "================\n" + color_names_.at(c)
           + " (" + std::to_string(parts_.at(p).at(c).size()) + " parts)\n================" <<std::endl;

            for (int i = 0; i < parts_.at(p).at(c).size(); i++) {

              parts_.at(p).at(c).at(i).Print_Info();

              }
            }
          }
      for (int p = 0; p < 5; p++) {
        for (int c = 0; c < 3; c++) {
          parts_.at(p).at(c).clear();
            }
          }
      logic_call_ = 0;
        }       
  }

  /**
   * @brief Called when a new Message is received on the Topic /ariac/break_beam_x_change
   * 
   * This function reports when an object crossed the beam.
   * 
   * @param msg Message of Boolean type returning true if an object crossed the beam.
   */
  void break_beam_callback(const nist_gear::Proximity::ConstPtr &msg)
  {
    if (msg->object_detected) // If there is an object in proximity.
      ROS_WARN("Break beam triggered.");
  }

  void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr &msg)
  {
    if ((msg->max_range - msg->range) > 0.01)
    { // If there is an object in proximity.
      ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
    }
  }

  void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    size_t number_of_valid_ranges = std::count_if(
        msg->ranges.begin(), msg->ranges.end(), [](const float f) { return std::isfinite(f); });
    if (number_of_valid_ranges > 0)
    {
      ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
    }
  }

private:
  std::string competition_state_;
  double current_score_;
  std::vector<nist_gear::Order> received_orders_; 
  std::array <std::array <std::vector < rwa2::Part >, 3>, 5> parts_ {}; //Datastructure to store the info from each part detected by the sensors
  int logic_call_ {0};
  std::array <const std::string, 5> part_names_ {"PISTON", "DISK", "GASKET", "GEAR", "PULLEY"};
  std::array <const std::string, 3> color_names_ {"Red", "Blue", "Green"};
};

int main(int argc, char **argv)
{
  // Last argument is the default name of the node.
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // Subscribe to the '/ariac/current_score' Topic.
  ros::Subscriber current_score_subscriber = node.subscribe(
      "/ariac/current_score", 10,
      &MyCompetitionClass::current_score_callback, &comp_class);

  // Subscribe to the '/ariac/competition_state' Topic.
  ros::Subscriber competition_state_subscriber = node.subscribe(
      "/ariac/competition_state", 10,
      &MyCompetitionClass::competition_state_callback, &comp_class);

  // Subscribe to the '/ariac/orders' Topic.
  ros::Subscriber orders_subscriber = node.subscribe(
      "/ariac/orders", 10,
      &MyCompetitionClass::order_callback, &comp_class);

  // // Subscribe to the '/ariac/range_finder_0' Topic.
  // ros::Subscriber proximity_sensor_subscriber = node.subscribe(
  //     "/ariac/range_finder_0", 
  //     10, 
  //     &MyCompetitionClass::proximity_sensor_callback,
  //     &comp_class);

  // Subscribe to the '/ariac/breakbeam_0_change' Topic.
  // ros::Subscriber break_beam_subscriber = node.subscribe(
  //     "/ariac/breakbeam_0_change", 10,
  //     &MyCompetitionClass::break_beam_callback,
  //     &comp_class);

  // Subscribe to the '/ariac/logical_camera_12' Topic.

  ros::Subscriber logical_camera_0_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_0", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 0));
  
  ros::Subscriber logical_camera_1_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_1", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 1));
      
  ros::Subscriber logical_camera_2_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_2", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 2));

  ros::Subscriber logical_camera_3_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_3", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 3));

  ros::Subscriber logical_camera_10_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_10", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 10));

  ros::Subscriber logical_camera_11_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_11", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 11));

  ros::Subscriber logical_camera_20_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_20", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 20));

  ros::Subscriber logical_camera_21_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_21", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 21));

 ros::Subscriber logical_camera_80_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_80", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 80));

  ros::Subscriber logical_camera_81_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_81", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 81));

  ros::Subscriber logical_camera_50_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_50", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 50));

  ros::Subscriber logical_camera_51_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_51", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 51));

  ros::Subscriber logical_camera_110_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_110", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 110));

  ros::Subscriber logical_camera_111_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_111", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 111));

  // Subscribe to the '/ariac/laser_profiler_0' Topic.
  // ros::Subscriber laser_profiler_subscriber = node.subscribe(
  //     "/ariac/laser_profiler_0", 10, &MyCompetitionClass::laser_profiler_callback, &comp_class);

  ROS_INFO("Setup complete.");
  start_competition(node);

  int i {0};

  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();

  }
  // ros::spin(); // This executes callbacks on new data until ctrl-c.

  return 0;
}