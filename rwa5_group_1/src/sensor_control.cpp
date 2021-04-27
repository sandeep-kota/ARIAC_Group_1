#include <ros/ros.h>
#include <math.h>

#include "sensor_control.h"
#include "utils.h"

/**
 * @brief Construct a new Sensor Control:: Sensor Control object
 * 
 * @param node 
 */
SensorControl::SensorControl(ros::NodeHandle &node)
{

  node_ = node;
}
/**
 * @brief Contructor
 * 
 */
void SensorControl::init()
{
  read_all_sensors_ = false;

  ros::param::set("/check_flipped", false);

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

  quality_ctrl_sensor1_subs = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/quality_control_sensor_1", 1, boost::bind(&SensorControl::quality_cntrl_sensor_callback, this, _1, 1));
  quality_ctrl_sensor2_subs = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/quality_control_sensor_2", 1, boost::bind(&SensorControl::quality_cntrl_sensor_callback, this, _1, 2));

  // Subscribe to the '/ariac/breakbeam' Topic.
  break_beam_subscriber_ = node_.subscribe("/ariac/breakbeam_0_change", 1, &SensorControl::break_beam_callback, this);

  //Get transforms world to logical camera sensors
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Duration timeout(5.0);

  geometry_msgs::TransformStamped transformStamped;

  for (int i = 0; i < 17; i++)
  {
    try
    {
      transformStamped = tfBuffer.lookupTransform("world", "logical_camera_" + std::to_string(i) + "_frame",
                                                  ros::Time(0), timeout);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    //Initialize attribute that stores the frame transforms to world of each camera
    c_w_transforms_.at(i) = transformStamped;
  }

  for (int i = 0; i < NUM_QUALITY_SENSORS; i++)
  {
    try
    {
      transformStamped = tfBuffer.lookupTransform("world", "quality_control_sensor_" + std::to_string(i + 1) + "_frame", ros::Time(0), timeout);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    //Initialize attribute that stores the frame transforms to world of each camera
    qualitySensorsTransforms.at(i) = transformStamped;
  }

  for (int i = 1; i < 17; i++)
  {
    ROS_WARN_STREAM("bin bumber: "<<i);
    try
    {
      transformStamped = tfBuffer.lookupTransform("world", "bin" + std::to_string(i) + "_frame", ros::Time(0), timeout);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    //Initialize attribute that stores the frame transforms to world of each camera
    binTransforms.at(i) = transformStamped;
  }
}

/**
   * @brief Maps part type to keys. 
   */
part_code SensorControl::hashit_type(std::string const &partString)
{
  if (partString == "disk")
    return kDisk;
  if (partString == "pulley")
    return kPulley;
  if (partString == "gasket")
    return kGasket;
  if (partString == "piston")
    return kPiston;
  if (partString == "gear")
    return kGear;
}

/**
   * @brief Maps part color to keys. 
   */
color_code SensorControl::hashit_color(std::string const &colorString)
{
  if (colorString == "red")
    return kRed;
  if (colorString == "blue")
    return kBlue;
  if (colorString == "green")
    return kGreen;
}
/**
 * @brief Logical Camera Callback
 * 
 * @param msg Subscribed Message 
 * @param sensor_n Sensor ID
 */
void SensorControl::logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg, int sensor_n)
{

  int explicit_activated_sensor = -1;
  ros::param::get(ACTIVATE_LOG_CAM, explicit_activated_sensor);

  if (explicit_activated_sensor == sensor_n) {
    if (logicalCamNumProducts.find(sensor_n) == logicalCamNumProducts.end()) {
      logicalCamNumProducts.insert({sensor_n, msg->models.size()});
    }
    else {
      logicalCamNumProducts.find(sensor_n)->second = msg->models.size();
    }
    return;

  }

  
  
  // ROS_INFO_STREAM("READING LOGICAL CAMERAS");
  int pos_t{};
  int pos_c{};
  int ktype{};
  int kcolor{};
  std::string type{};
  std::string color{};
  part Part;
  bool check_flipped{false};
  bool new_part_conveyor{false};

  int sum = 0;
  for (int i = 3; i < 17; i++)
  {
    sum += logic_call_[i];
  }

  std::string sensorLoc = sensorLocationMap.find(sensor_n) != sensorLocationMap.end() ? sensorLocationMap.at(sensor_n) : CONV_BELT;

  if ((logic_call_[sensor_n] == 0) && (read_all_sensors_ == false) && ((sensor_n != 0 && sensor_n != 1 && sensor_n != 2)))
  {
    //getting the set of empty locations to be used to place the products as temp locations
    if (msg->models.size() == 0 && sensor_n > 2 && sensorLoc != CONV_BELT)
    {
      emptyLocations.insert(sensorLoc);
    }

    for (int i = 0; i < msg->models.size(); i++)
    {

      pos_t = msg->models.at(i).type.find("_");
      pos_c = msg->models.at(i).type.rfind("_");

      type = msg->models.at(i).type.substr(0, pos_t);
      color = msg->models.at(i).type.substr(pos_c + 1);

      ktype = hashit_type(type);
      kcolor = hashit_color(color);

      Part.picked_status = false;
      Part.type = msg->models.at(i).type;
      Part.pose = frame_to_world(i, msg->models.at(i).pose, c_w_transforms_.at(sensor_n));
      Part.save_pose = Part.pose;
      Part.frame = "logical_camera_" + std::to_string(sensor_n) + "_frame";
      Part.time_stamp = ros::Time::now();
      Part.id = Part.type + std::to_string(parts_.at(ktype).at(kcolor).size());

      Part.location = sensorLoc;

      if (sensor_n > 2 && sensor_n < 7) {
        if (Part.pose.position.x > c_w_transforms_.at(sensor_n).transform.translation.x && Part.pose.position.y > msg->pose.position.y){
          if (binTransforms.at(binMap[sensor_n]["up_left"]).transform.translation.x + 0.1 < Part.pose.position.x){
            Part.bin_location = "top";
          } else if (binTransforms.at(binMap[sensor_n]["up_left"]).transform.translation.x - 0.1 > Part.pose.position.x){
            Part.bin_location = "bottom";
          } else {
            Part.bin_location = "middle";
          }
          
        } else if (Part.pose.position.x < c_w_transforms_.at(sensor_n).transform.translation.x && Part.pose.position.y > msg->pose.position.y){
          if (binTransforms.at(binMap[sensor_n]["down_left"]).transform.translation.x + 0.1 < Part.pose.position.x){
            Part.bin_location = "top";
          } else if (binTransforms.at(binMap[sensor_n]["down_left"]).transform.translation.x - 0.1 > Part.pose.position.x){
            Part.bin_location = "bottom";
          } else {
            Part.bin_location = "middle";
          }
        } else if (Part.pose.position.x > c_w_transforms_.at(sensor_n).transform.translation.x && Part.pose.position.y < msg->pose.position.y){
          if (binTransforms.at(binMap[sensor_n]["up_right"]).transform.translation.x + 0.1 < Part.pose.position.x){
            Part.bin_location = "top";
          } else if (binTransforms.at(binMap[sensor_n]["up_right"]).transform.translation.x - 0.1 > Part.pose.position.x){
            Part.bin_location = "bottom";
          } else {
            Part.bin_location = "middle";
          }
        } else {
          if (binTransforms.at(binMap[sensor_n]["down_right"]).transform.translation.x + 0.1 < Part.pose.position.x){
            Part.bin_location = "top";
          } else if (binTransforms.at(binMap[sensor_n]["down_right"]).transform.translation.x - 0.1 > Part.pose.position.x){
            Part.bin_location = "bottom";
          } else {
            Part.bin_location = "middle";
          }
        }
      }

      int partsCount = parts_.at(ktype).at(kcolor).size();
      bool canPartBeAdded = true;

      for (int j = 0; (j < partsCount && canPartBeAdded); j++)
      {
        part partAlreadyPresent = parts_.at(ktype).at(kcolor).at(j);
        float xDiff = partAlreadyPresent.pose.position.x - Part.pose.position.x;
        float yDiff = partAlreadyPresent.pose.position.y - Part.pose.position.y;
        float zDiff = partAlreadyPresent.pose.position.z - Part.pose.position.z;
        double dist = pow(pow(xDiff, 2) + pow(yDiff, 2) + pow(zDiff, 2), 0.5);
        if (dist <= 0.01)
        { // dist threshold can be put into a constant.
          ROS_INFO_STREAM("Part already exists. Cannot be added. Part type:: " << Part.type << " Location: [x,y,z]:: " << Part.pose.position.x << ", " << Part.pose.position.y << ", " << Part.pose.position.z);
          canPartBeAdded = false;
        }
      }

      if (canPartBeAdded)
      {
        parts_.at(ktype).at(kcolor).push_back(Part);
        ROS_INFO_STREAM("New part added from camera " << sensor_n << ". Part type, Part Location " << Part.type << "," << Part.location);
      }
    }
    logic_call_[sensor_n] = 1;
  }

  if (logic_call_[sensor_n] == 1 && sum == 14)
  {
    read_all_sensors_ = true;
  }

  //Check for flipped parts on agvs
  // ros::param::get("/check_flipped", check_flipped);

  // if (check_flipped && (sensor_n == 0 || sensor_n == 1))
  // {
    // int t_sum = 0;
    // for (int i = 0; i < 2; i++)
    // {
    //   t_sum += logic_call_agv_[i];
    // }
    // if (t_sum == 2)
    // {
    //   ros::param::set("/check_flipped", false);
    //   logic_call_agv_[0] = 0;
    //   logic_call_agv_[1] = 0;
    // }
    // if ((logic_call_agv_[sensor_n] == 0) && t_sum < 2)
    // {
  //     ROS_WARN_STREAM("SENSOR: " << sensor_n);
  //     logic_call_agv_[sensor_n] = 1;
  //     for (int i = 0; i < msg->models.size(); i++)
  //     {
  //       pos_t = msg->models.at(i).type.find("_");
  //       type = msg->models.at(i).type.substr(0, pos_t);

  //       Part.picked_status = false;
  //       Part.type = msg->models.at(i).type;
  //       Part.pose = frame_to_world(i, msg->models.at(i).pose, c_w_transforms_.at(sensor_n));
  //       Part.save_pose = Part.pose;
  //       Part.frame = "logical_camera_" + std::to_string(sensor_n) + "_frame";
  //       Part.time_stamp = ros::Time::now();

  //       if (type.compare("pulley") == 0) //flipped parts can only be pulleys
  //       {
  //         ROS_WARN_STREAM("TYPE: " << type);
  //         partsToFlip.push_back(Part);
  //       }
  //     }
  //   }
  // }

  //Check for new parts in conveyor

  ros::param::get("/ariac/new_part_conveyor", new_part_conveyor);

  if (new_part_conveyor && sensor_n == 2)
  {
    part last_part_conveyor;
    double current_y;

    for (int i = 0; i < msg->models.size(); i++)
    {
      pos_t = msg->models.at(i).type.find("_");
      type = msg->models.at(i).type.substr(0, pos_t);

      Part.picked_status = false;
      Part.type = msg->models.at(i).type;
      Part.pose = frame_to_world(i, msg->models.at(i).pose, c_w_transforms_.at(sensor_n));
      Part.save_pose = Part.pose;
      Part.frame = "logical_camera_" + std::to_string(sensor_n) + "_frame";
      Part.time_stamp = ros::Time::now();

      if (partsConveyor.empty())
      {
        partsConveyor.push_back(Part);
      }
      else
      {
        last_part_conveyor = partsConveyor.back();
        current_y = last_part_conveyor.pose.position.y - 0.2 * (ros::Time::now().toSec() - last_part_conveyor.time_stamp.toSec());

        if ((current_y + 0.1) < Part.pose.position.y)
        {
          partsConveyor.push_back(Part);
        }
      }
    }
    ros::param::set("/ariac/new_part_conveyor", false);
  }

  bool check_parts_to_flip_in_trays;
  ros::param::get("/check_parts_to_flip", check_parts_to_flip_in_trays);
  if (check_parts_to_flip_in_trays && (sensor_n == 0 || sensor_n == 1))
  {
    // ROS_WARN_STREAM("CHECKING PARTS IN TRAY TO FLIP");
    int t_sum = 0;
    for (int i = 0; i < 2; i++)
    {
      t_sum += logic_call_agv_[i];
    }
    if (t_sum == 2)
    {
      ros::param::set("/check_parts_to_flip", false);
      logic_call_agv_[0] = 0;
      logic_call_agv_[1] = 0;
    }
    if ((logic_call_agv_[sensor_n] == 0) && t_sum < 2)
    {
      logic_call_agv_[sensor_n] = 1;
      if (msg->models.size()>0){
        for (int i = 0; i < msg->models.size(); i++)
        {
          pos_t = msg->models.at(i).type.find("_");
          type = msg->models.at(i).type.substr(0, pos_t);
          if (type == "pulley"){
            // ROS_WARN_STREAM("IT IS A: "<<type);
            Part.picked_status = false;
            Part.type = msg->models.at(i).type;
            Part.pose = frame_to_world(i, msg->models.at(i).pose, c_w_transforms_.at(sensor_n));
            Part.save_pose = Part.pose;
            Part.frame = "logical_camera_" + std::to_string(sensor_n) + "_frame";
            Part.time_stamp = ros::Time::now();

            if (sensor_n == 0)
            {
              Part.location = "agv_1";
            }
            if (sensor_n == 1)
            {
              Part.location = "agv_2";
            }
            checkPartsToFlip.push_back(Part);
          }   
        }
      }
    }
  }


  bool update_agv_parts;
  ros::param::get("/update_agv_parts", update_agv_parts);
  if (update_agv_parts && (sensor_n == 0 || sensor_n == 1))
  {
    int t_sum = 0;
    for (int i = 0; i < 2; i++)
    {
      t_sum += logic_call_agv_[i];
    }
    // ROS_WARN_STREAM("SUM AGV :" << t_sum);
    if (t_sum == 2)
    {
      ros::param::set("/update_agv_parts", false);
      logic_call_agv_[0] = 0;
      logic_call_agv_[1] = 0;
    }
    if ((logic_call_agv_[sensor_n] == 0) && t_sum < 2)
    {
      logic_call_agv_[sensor_n] = 1;
      // ROS_WARN_STREAM("Update AGV Parts" << sensor_n);
      for (int i = 0; i < 5; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          if (sensor_n == 0)
          {
            parts_agv1_.at(i).at(j).clear();
          }
          if (sensor_n == 1)
          {
            parts_agv2_.at(i).at(j).clear();
          }
        }
      }

      for (int i = 0; i < msg->models.size(); i++)
      {
        pos_t = msg->models.at(i).type.find("_");
        pos_c = msg->models.at(i).type.rfind("_");

        type = msg->models.at(i).type.substr(0, pos_t);
        color = msg->models.at(i).type.substr(pos_c + 1);

        ktype = hashit_type(type);
        kcolor = hashit_color(color);

        Part.picked_status = false;
        Part.type = msg->models.at(i).type;
        Part.pose = frame_to_world(i, msg->models.at(i).pose, c_w_transforms_.at(sensor_n));
        Part.save_pose = Part.pose;
        Part.frame = "logical_camera_" + std::to_string(sensor_n) + "_frame";
        Part.time_stamp = ros::Time::now();

        if (sensor_n == 0)
        {
          Part.location = "agv_1";
          Part.id = Part.type + std::to_string(parts_agv1_.at(ktype).at(kcolor).size());
          parts_agv1_.at(ktype).at(kcolor).push_back(Part);
          // ROS_INFO_STREAM("New part added to AGV1. Part tye:: " << Part.type);
        }
        if (sensor_n == 1)
        {
          Part.location = "agv_2";
          Part.id = Part.type + std::to_string(parts_agv2_.at(ktype).at(kcolor).size());
          parts_agv2_.at(ktype).at(kcolor).push_back(Part);
          // ROS_INFO_STREAM("New part added to AGV2. Part tye:: " << Part.type);
        }
      }
    }
  }
}

/**
 * @brief Transform frame to World Frame
 * 
 * @param i Part ID
 * @param original_pos Original Pose 
 * @param c_w_transform Transform Stamped object for each transformation
 * @return geometry_msgs::Pose 
 */
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

/**
 * @brief Locate a part in the world
 * 
 * @param type Search Part type 
 * @return Part 
 */
Part SensorControl::findPart(std::string type)
{

  int pos_t = type.find("_");
  int pos_c = type.rfind("_");
  std::string parttype = type.substr(0, pos_t);
  std::string partcolor = type.substr(pos_c + 1);
  part no_part;

  int ktype = hashit_type(parttype);
  int kcolor = hashit_color(partcolor);
  if (parts_.at(ktype).at(kcolor).empty())
  {
    return no_part;
  }
  else
  {
    for (int i = 0; i < parts_.at(ktype).at(kcolor).size(); i++)
    {
      if (parts_.at(ktype).at(kcolor).at(i).picked_status == false)
      {
        parts_.at(ktype).at(kcolor).at(i).picked_status = true;
        return parts_.at(ktype).at(kcolor).at(i);
      }
    }
    return no_part;
  }
}

/**
 * @brief callback function for breakbeam sensor
 * 
 * @param msg  Subscribed message
 */
void SensorControl::break_beam_callback(const nist_gear::Proximity::ConstPtr &msg)
{
  if (msg->object_detected) // If there is an object in proximity.
  {
    ROS_WARN("Break beam triggered.");
  }
  else
  {
    ros::param::set("/ariac/new_part_conveyor", true);
  }
}

/**
 * @brief Quality Control Sensor Callback
 * 
 * @param msg Subscribed message
 * @param sensor_n Sensor ID
 */
void SensorControl::quality_cntrl_sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg, int sensor_n)
{
  bool activate_quality;
  ros::param::get("/activate_quality", activate_quality);
  // ROS_WARN_STREAM("QUALITY CALL");
  // if (msg == nullptr) {
  //   ROS_WARN_STREAM("MSG SIZE: EMPTY");
  // }
  // else {
  //   ROS_WARN_STREAM("MSG SIZE: " << msg->models.size());
  // }
  
  
  if (activate_quality)
  {
    ROS_WARN_STREAM("SENSOR N: " << sensor_n);
    ROS_WARN_STREAM("MSG MODEL SIZE: " << msg->models.size());
    int sum = 0;
    for (int i = 0; i < 2; i++)
    {
      sum += logic_call_quality_[i];
      ROS_WARN_STREAM("SUM " << sum);
    }
    if (sum == 2)
    {
      ros::param::set("/activate_quality", false);
      ROS_WARN_STREAM("ALL SENSORS CALLED " << sensor_n);
      
      if (faultyPartsList.size() > 0)
      {
        setFaultyPartDetectedFlag(true);
        // ROS_WARN_STREAM("SUM " << sum);
      } else {
        setFaultyPartDetectedFlag(false);
      }
      
    }
    if ((logic_call_quality_[sensor_n-1] == 0) && sum < 2)
    {
      logic_call_quality_[sensor_n-1] = 1;
      for (int i = 0; i < msg->models.size(); i++)
      {
        part faultyPart;

        faultyPart.picked_status = false;
        faultyPart.pose = frame_to_world(i, msg->models.at(i).pose, qualitySensorsTransforms.at(sensor_n - 1));
        faultyPart.save_pose = faultyPart.pose;
        faultyPart.frame = "quality_control_sensor_" + std::to_string(sensor_n) + "_frame";
        faultyPart.time_stamp = ros::Time::now();
        faultyPart.id = faultyPart.type + std::to_string(i);

        if (sensor_n == 2)
        {
          faultyPart.location = "agv_1";
        }
        else if (sensor_n == 1)
        {
          faultyPart.location = "agv_2";
        }
        faultyPartsList.push_back(faultyPart);
      }
    }
  }
}

/**
 * @brief Return Parts in AGV
 * 
 * @param agv_id AGV ID
 * @return std::array<std::array<std::vector<part>, 3>, 5> 
 */
std::array<std::array<std::vector<part>, 3>, 5> SensorControl::getPartsAGV(std::string agv_id)
{
  if (agv_id.compare("agv1") == 1)
  {
    return parts_agv1_;
  }
  else
  {
    return parts_agv2_;
  }
}

/**
 * @brief Check if the given part is aligned  properly in the tray
 * 
 * @param target Target part that has to be checked
 * @param agv_id AGV ID
 * @return true 
 * @return false 
 */
bool SensorControl::isPartPoseAGVCorrect(part target, std::string agv_id)
{
  int pos_t = target.type.find("_");
  int pos_c = target.type.rfind("_");
  std::string parttype = target.type.substr(0, pos_t);
  std::string partcolor = target.type.substr(pos_c + 1);

  ROS_WARN_STREAM("PART TO BE INSPECTION FOR ORIENTATION: " << target.type);

  int ktype = hashit_type(parttype);
  int kcolor = hashit_color(partcolor);
  bool ret_val = false;
  if (agv_id == "agv1")
  {
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // ros::Duration timeout(1.0);
    // geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("world", "kit_tray_2",ros::Time(0), timeout);

    // tf2::doTransform(p.pose,p.pose,transformStamped);

    for (int j = 0; j < parts_agv1_.at(ktype).at(kcolor).size(); j++)
    {
      part partAlreadyPresent = parts_agv1_.at(ktype).at(kcolor).at(j);
      float p_xDiff = partAlreadyPresent.pose.position.x - target.pose.position.x;
      float p_yDiff = partAlreadyPresent.pose.position.y - target.pose.position.y;
      float p_zDiff = partAlreadyPresent.pose.position.z - target.pose.position.z;
      double dist = pow(pow(p_xDiff, 2) + pow(p_yDiff, 2) + pow(p_zDiff, 2), 0.5);

      tf2::Quaternion q1(partAlreadyPresent.pose.orientation.x, partAlreadyPresent.pose.orientation.y, partAlreadyPresent.pose.orientation.z, partAlreadyPresent.pose.orientation.w);
      tf2::Quaternion q2(target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w);

      tf2::Matrix3x3 m1(q1);
      tf2::Matrix3x3 m2(q2);

      double r1, r2, p1, p2, y1, y2;
      m1.getRPY(r1, p1, y1);
      m2.getRPY(r2, p2, y2);

      float o_xDiff = (r2 - r1) * (180 / PI);
      float o_yDiff = (p2 - p1) * (180 / PI);
      float o_zDiff = (y2 - y1) * (180 / PI);

      if (o_zDiff < 0 && abs(o_zDiff) > 180)
      {
        o_zDiff += 360;
      }

      ROS_INFO_STREAM("Difference AGV1 " << j << ":(Dist,Ori) :" << dist << ";(" << o_xDiff << "," << o_yDiff << "," << o_zDiff << ");" << ((dist <= 0.01) || (std::abs(o_zDiff) < 2)));
      incorrect_part_agv1 = parts_agv1_.at(ktype).at(kcolor).at(j);
      if ((dist <= 0.01) && (std::abs(o_zDiff) < 2))
      {
        ROS_INFO_STREAM("Part Aligned Properly");
        Part dummy_part;
        incorrect_part_agv1 = dummy_part;
        ret_val = true;
        break;
      }
    }
  }

  else if (agv_id == "agv2")
  {
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // ros::Duration timeout(1.0);
    // geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("world", "kit_tray_2",ros::Time(0), timeout);

    // tf2::doTransform(p.pose,p.pose,transformStamped);

    for (int j = 0; j < parts_agv2_.at(ktype).at(kcolor).size(); j++)
    {
      part partAlreadyPresent = parts_agv2_.at(ktype).at(kcolor).at(j);
      float p_xDiff = partAlreadyPresent.pose.position.x - target.pose.position.x;
      float p_yDiff = partAlreadyPresent.pose.position.y - target.pose.position.y;
      float p_zDiff = partAlreadyPresent.pose.position.z - target.pose.position.z;
      double dist = pow(pow(p_xDiff, 2) + pow(p_yDiff, 2) + pow(p_zDiff, 2), 0.5);

      tf2::Quaternion q1(partAlreadyPresent.pose.orientation.x, partAlreadyPresent.pose.orientation.y, partAlreadyPresent.pose.orientation.z, partAlreadyPresent.pose.orientation.w);
      tf2::Quaternion q2(target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w);

      tf2::Matrix3x3 m1(q1);
      tf2::Matrix3x3 m2(q2);

      double r1, r2, p1, p2, y1, y2;
      m1.getRPY(r1, p1, y1);
      m2.getRPY(r2, p2, y2);

      float o_xDiff = (r2 - r1) * (180 / PI);
      float o_yDiff = (p2 - p1) * (180 / PI);
      float o_zDiff = (y2 - y1) * (180 / PI);

      ROS_INFO_STREAM("Difference AGV2 " << j << ":(Dist,Ori) :" << dist << ";(" << o_xDiff << "," << o_yDiff << "," << o_zDiff << ");" << ((dist <= 0.01) || (std::abs(o_zDiff) < 2)));
      incorrect_part_agv2 = parts_agv2_.at(ktype).at(kcolor).at(j);
      if ((dist <= 0.01) && (std::abs(o_zDiff) < 2))
      {
        incorrect_part_agv2 = parts_agv2_.at(ktype).at(kcolor).at(j - 1);
        ret_val = true;
        break;
      }
    }
  }
  return ret_val;
}

/**
 * @brief Get parts from a logical camera
 * 
 * @param sensorNum Logical camera id.
 * @return int 
 */
int SensorControl::getLogicalCameraNumProducts(int sensorNum) {
  if (logicalCamNumProducts.find(sensorNum) == logicalCamNumProducts.end()) {
    return 0;
  }
  return logicalCamNumProducts.at(sensorNum);
}

