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

  // Separate Callback for Conveyor belt Logical Camera
  logical_camera_16_subcriber_ = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/logical_camera_16", 1, boost::bind(&SensorControl::belt_camera_callback, this, _1, 0));

  quality_ctrl_sensor1_subs = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/quality_control_sensor_1", 1, boost::bind(&SensorControl::quality_cntrl_sensor_callback, this, _1, 1));
  quality_ctrl_sensor2_subs = node_.subscribe<nist_gear::LogicalCameraImage>(
      "/ariac/quality_control_sensor_2", 1, boost::bind(&SensorControl::quality_cntrl_sensor_callback, this, _1, 2));

  //Get transforms world to logical camera sensors
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Duration timeout(5.0);

  geometry_msgs::TransformStamped transformStamped;

  for (int i = 0; i < NUM_LOGICAL_CAMERAS; i++)
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

  for (int i = 0; i < NUM_BELT_CAMERAS; i++)
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
    beltCamTransforms_.at(i) = transformStamped;
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

  // ROS_INFO_STREAM("READING LOGICAL CAMERAS");
  int pos_t{};
  int pos_c{};
  int ktype{};
  int kcolor{};
  std::string type{};
  std::string color{};
  part Part;

  int sum = 0;
  for (int i = 0; i < NUM_LOGICAL_CAMERAS; i++)
  {
    sum += logic_call_[i];
  }

  if (logic_call_[sensor_n] == 0)
  {
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
      if (sensor_n == 3 || sensor_n == 4 || sensor_n == 5 || sensor_n == 6)
      {
        Part.location = "bins";
      }
      else if (sensor_n == 9 || sensor_n == 10)
      {
        Part.location = "shelf_2";
      }
      else if (sensor_n == 7 || sensor_n == 8)
      {
        Part.location = "shelf_1";
      }
      else if (sensor_n == 13 || sensor_n == 14)
      {
        Part.location = "shelf_5";
      }
      else if (sensor_n == 11 || sensor_n == 12)
      {
        Part.location = "shelf_8";
      }
      else if (sensor_n == 15 || sensor_n == 16)
      {
        Part.location = "shelf_11";
      }
      else if (sensor_n == 0)
      {
        Part.location = "agv_1";
      }
      else if (sensor_n == 1)
      {
        Part.location = "agv_2";
      }
      else
      {
        Part.location = "conveyor_belt";
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
        ROS_INFO_STREAM("New part added. Part tye:: " << Part.type);
      }
    }
    logic_call_[sensor_n] = 1;
  }

  if (logic_call_[sensor_n] == 1 && sum == NUM_LOGICAL_CAMERAS)
  {
    read_all_sensors_ = true;
  }
}

/**
 * @brief Logical Camera Callback specifically over the conveyor belt 
 * 
 * @param msg Subscribed Message 
 * @param sensor_n Sensor ID
 */
void SensorControl::belt_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg, int sensor_n) {
	
	int pos_t;
	int pos_c;
	int ktype;
	int kcolor;
	int num_belt_parts;
	std::string type;
	std::string color;
	part cur_part;

	for (int i = 0; i < msg->models.size(); i++) {
		pos_t = msg->models.at(i).type.find("_");
		pos_c = msg->models.at(i).type.rfind("_");

		type = msg->models.at(i).type.substr(0, pos_t);
		color = msg->models.at(i).type.substr(pos_c + 1);

		ktype = hashit_type(type);
		kcolor = hashit_color(color);

		cur_part.picked_status = false;
		cur_part.type = msg->models.at(i).type;
		cur_part.pose = frame_to_world(i, msg->models.at(i).pose, beltCamTransforms_.at(sensor_n));
		cur_part.save_pose = cur_part.pose;
		cur_part.frame = "logical_camera_" + std::to_string(sensor_n) + "_frame";
		cur_part.time_stamp = ros::Time::now();


		num_belt_parts = belt_parts_.at(ktype).at(kcolor).size();
		cur_part.id = cur_part.type + std::to_string(num_belt_parts);

		bool dupl = false;

		for(int j =0; j< num_belt_parts; ++j) {
			
			part prev_part = belt_parts_[ktype][kcolor][j];
			double detected_y_pos = prev_part.pose.position.y;
			double detected_time = prev_part.time_stamp.toSec();

			double cur_y_pos = cur_part.pose.position.y;
			double cur_time = cur_part.time_stamp.toSec();


			double expected_pos = detected_y_pos - BELT_SPEED*(cur_time - detected_time);

			if(abs(cur_y_pos - expected_pos) <= 0.05) {
				dupl = true;
				break;
			}

		}


		if(! dupl) {
			belt_parts_.at(ktype).at(kcolor).emplace_back(cur_part);

			print_available_parts();
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
  }
}

/**
 * @brief Locates Part on Belt 
 * 
 * @param type Search Part type 
 * @return Part 
 */
Part SensorControl::find_belt_part(std::string type) {
	int pos_t = type.find("_");
	int pos_c = type.rfind("_");
	std::string parttype = type.substr(0, pos_t);
	std::string partcolor = type.substr(pos_c + 1);
	part no_part;

	int ktype = hashit_type(parttype);
	int kcolor = hashit_color(partcolor);
	if (belt_parts_.at(ktype).at(kcolor).empty()) {
		return no_part;
	} else {
		std::cout<<belt_parts_.at(ktype).at(kcolor).size()<<std::endl;
		for (int i = 0; i < belt_parts_.at(ktype).at(kcolor).size(); i++) {

			double detected_y_pos = belt_parts_[ktype][kcolor][i].pose.position.y;
			double detected_time = belt_parts_[ktype][kcolor][i].time_stamp.toSec();

			double cur_time = ros::Time::now().toSec();


			double expected_pos = detected_y_pos - BELT_SPEED*(cur_time - detected_time);

			if (belt_parts_.at(ktype).at(kcolor).at(i).picked_status == false && expected_pos > - 3.5) {
				belt_parts_.at(ktype).at(kcolor).at(i).picked_status = true;
				return belt_parts_.at(ktype).at(kcolor).at(i);
			}
		}

		return no_part;
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
  // ROS_WARN_STREAM("ACTIVATE QUALITY: " << activate_quality);
  if (msg->models.size() != 0 && activate_quality)
  {

    setFaultyPartDetectedFlag(true);

    ROS_INFO_STREAM("Faulty part detected!" << faultyProducts.size());

    if (faultyProducts.size() == 0)
    {

      for (int i = 0; i < msg->models.size(); i++)
      {
        Product faultyProduct;
        part faultyPart;

        faultyPart.picked_status = false;
        faultyPart.type = msg->models.at(i).type;
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

        faultyProduct.p = faultyPart;
        faultyProduct.type = faultyPart.type;
        faultyProduct.pose = faultyPart.pose;
        faultyProduct.actual_pose_frame = faultyPart.frame;
        faultyProduct.agv_id = "agv" + (sensor_n == 1) ? std::to_string(2) : std::to_string(1);

        if (faultyProduct.agv_id == "agv1")
        {
          faultyProduct.tray = "kit_tray_1";
        }

        if (faultyProduct.agv_id == "agv2")
        {
          faultyProduct.tray = "kit_tray_2";
        }

        if (faultyProduct.agv_id == "any")
        {
          faultyProduct.tray = "kit_tray_1";
        }

        faultyProducts.push_back(faultyProduct);
      }
    }
  }
}


void SensorControl::print_available_parts() {

	std::cout<< "Static Parts (not on conveyor belt):"<<std::endl;
	for(int i= 0; i<5; ++i) {
		for(int j=0; j<3; ++j) {
			for(part p: parts_[i][j]) {
				std::cout<<p<<std::endl;
			}
		}
	}
	std::cout<<std::endl;
	std::cout<< "Parts on Conveyor Belt:"<<std::endl;
	for(int i= 0; i<5; ++i) {
		for(int j=0; j<3; ++j) {
			for(part p: belt_parts_[i][j]) {
				std::cout<<p<<std::endl;
			}
		}
	}


}