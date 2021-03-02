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

enum color_code {
  kRed,
  kBlue,
  kGreen
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
    
    int pos_t {};
    int pos_c {};
    std::string type {};
    std::string color {};
    geometry_msgs::PoseStamped world_pose {};


    if (msg->models.size() > 0) {
      for (int i = 0; i < msg->models.size(); i++)
        {
          pos_t = msg->models.at(i).type.find("_");
          pos_c = msg->models.at(i).type.rfind("_");

          type = msg->models.at(i).type.substr(0,pos_t);
          color = msg->models.at(i).type.substr(pos_c+1);
          world_pose = frame_to_world(i, msg->models.at(i).pose, c_w_transforms_.at(sensor_n));

          save_part_array(type, color, sensor_n, world_pose);                    
        }
    }
    if (logic_call_ == 17){

      std::cout << "\n\n" <<std::endl;
      ROS_INFO("LOGICAL CAMERA DETECTED PARTS");
      remove_duplicate_parts();
      print_part_poses("all","all",0);


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

  /**
   * @brief [brief description]
   * @details [long description]
   * 
   * @param i [description]
   * @param original_pos [description]
   * @param c_w_transform [description]
   * @return [description]
   */
  geometry_msgs::PoseStamped frame_to_world(int i, geometry_msgs::Pose original_pos, geometry_msgs::TransformStamped c_w_transform)
  {   
    geometry_msgs::PoseStamped pose_target, pose_rel;  

    pose_rel.header.frame_id = c_w_transform.header.frame_id + "_part_" + std::to_string(i);
    pose_rel.pose = original_pos;
    tf2::doTransform(pose_rel, pose_target, c_w_transform);

    return pose_target;
  }

  /**
   * @brief Maps part type to keys. 
   */
  part_code hashit_type (std::string const& partString) {
    if (partString == "disk") return kDisk;
    if (partString == "pulley") return kPulley;
    if (partString == "gasket") return kGasket;
    if (partString == "piston") return kPiston;
    if (partString == "gear") return kGear;
  }

  /**
   * @brief Maps part color to keys. 
   */
  color_code hashit_color (std::string const& colorString) {
    if (colorString == "red") return kRed;
    if (colorString == "blue") return kBlue;
    if (colorString == "green") return kGreen;
  }


  /**
   * @brief Used to save the parts into the array.
   */
  void save_part_array(std::string type, std::string color, int sensor_n, geometry_msgs::PoseStamped world_pose) {
    std::string part_id {};
    int ktype = hashit_type(type);
    int kcolor =hashit_color(color);
    // std::cout << kcolor <<std::endl;
    part_id = type + color + "part" + std::to_string(parts_.at(ktype).at(kcolor).size());

    rwa2::Part part (part_id, type, color, sensor_n, world_pose);

    parts_.at(ktype).at(kcolor).push_back(part); 
  } 

/**
 * @brief Rmove duplicate parts added to the list
 * @details Checks for similar poses occuring in adjacent cameras
 */
  void remove_duplicate_parts() {
    for (int i=0; i<5;i++) {
      for (int j=0; j<3; j++) {
      // std::cout<<i<<" "<<j<<" "<<parts_[i][j].size()<<std::endl;
        for (int k1=0; k1<parts_[i][j].size();k1++) {
          for (int k2=k1; k2<parts_[i][j].size();k2++) {
            std::vector<double> p1 = parts_[i][j][k1].get_pose_world();
            std::vector<double> p2 = parts_[i][j][k2].get_pose_world();
            int dist = std::sqrt(((p1[0]-p2[0])*(p1[0]-p2[0])) + ((p1[1]-p2[1])*(p1[1]-p2[1])) + ((p1[2]-p2[2])*(p1[2]-p2[2])));
            if ((dist<=0.00001) && (std::abs(parts_[i][j][k1].get_sensor() - parts_[i][j][k2].get_sensor())==1)) {
              std::cout<<"Duplicate :" << dist<<std::endl;
              std::cout<<"P1 "<<"x :"<<p1[0]<<"y :"<<p1[1]<<"z :"<<p1[2]<<std::endl;
              std::cout<<"P2 "<<"x :"<<p2[0]<<"y :"<<p2[1]<<"z :"<<p2[2]<<std::endl;
              parts_[i][j].erase(parts_[i][j].begin()+k2);
              std::cout<<"!!!"<<std::endl;
              break;
            }
          }
        }        
      }
    }
  }

  
  /**
   * @brief      Prints part poses.
   *
   * @param[in]  p_color  The color of the parts
   * @param[in]  p_type   The tyype of the parts
   * @param[in]  verbose  The verbose
   */
  void print_part_poses(std::string p_color = "all",std::string p_type = "all", int verbose=0) {
    
      int p_it_min=0;
      int p_it_max=5;
      int c_it_min=0;
      int c_it_max=3;

      
      if (p_type!="all") {
        p_it_min = hashit_type(p_type);
        p_it_max = hashit_type(p_type)+1;
      } 

      if (p_color!="all") {
        c_it_min = hashit_color(p_color);
        c_it_max = hashit_color(p_color)+1;
      } 


      for (int p = p_it_min; p < p_it_max; p++) {
        std::cout << "\n\n*****  " + part_names_[p] + " (" + std::to_string(parts_[p][0].size()+parts_[p][1].size()
                  +parts_.at(p).at(2).size()) + " parts)" + "  *****" <<std::endl;
        
        for (int c = c_it_min; c < c_it_max; c++) {
          std::cout << "===================\n" + color_names_[c] + " " + part_names_[p] 
          + " (" + std::to_string(parts_[p][c].size()) + " parts)\n===================" <<std::endl;
            
            if (verbose==0) {
              for (int i = 0; i < parts_[p][c].size(); i++) {
                std::cout<< i <<std::endl;
                parts_[p][c][i].Print_Info();
              }
            }
        }
      }
  

    std::cout << "------------!!!----------" << std::endl;
  }

  void set_c_w_transform (int c, geometry_msgs::TransformStamped transform) {
    c_w_transforms_.at(c) = transform;
  }

private:
  std::string competition_state_;
  double current_score_;
  std::vector<nist_gear::Order> received_orders_; 
  std::array <std::array <std::vector < rwa2::Part >, 3>, 5> parts_ {}; //Datastructure to store the info from each part detected by the sensors
  int logic_call_ {0};
  std::array <const std::string, 5> part_names_ {"DISK", "PULLEY", "PISTON", "GEAR", "GASKET"};
  std::array <const std::string, 3> color_names_ {"Red", "Blue", "Green"};
  std::array <geometry_msgs::TransformStamped, 17> c_w_transforms_ {}; 

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

  ros::Subscriber logical_camera_4_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_4", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 4));

  ros::Subscriber logical_camera_5_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_5", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 5));

  ros::Subscriber logical_camera_6_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_6", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 6));

  ros::Subscriber logical_camera_7_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_7", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 7));

  ros::Subscriber logical_camera_8_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_8", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 8));

 ros::Subscriber logical_camera_9_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_9", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 9));

  ros::Subscriber logical_camera_10_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_10", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 10));

  ros::Subscriber logical_camera_11_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_11", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 11));

  ros::Subscriber logical_camera_12_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_12", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 12));

  ros::Subscriber logical_camera_13_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_13", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 13));

  ros::Subscriber logical_camera_14_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_14", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 14));

  ros::Subscriber logical_camera_15_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_15", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 15));

  ros::Subscriber logical_camera_16_subcriber = node.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_16", 1, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, 16));
  // Subscribe to the '/ariac/laser_profiler_0' Topic.
  // ros::Subscriber laser_profiler_subscriber = node.subscribe(
  //     "/ariac/laser_profiler_0", 10, &MyCompetitionClass::laser_profiler_callback, &comp_class);

  ROS_INFO("Setup complete.");
  start_competition(node);

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
    comp_class.set_c_w_transform(i, transformStamped);
  }

  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

  }
  // ros::spin(); // This executes callbacks on new data until ctrl-c.

  return 0;
}