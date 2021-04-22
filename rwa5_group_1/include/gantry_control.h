#ifndef GANTRYCONTROL_H
#define GANTRYCONTROL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/tf.h>

#include "utils.h"

class GantryControl
{

public:
  GantryControl(ros::NodeHandle &node);

  void init();
  void initializeShelfConfiguration();

  //    bool moveGantry(std::string waypoints);

  //    bool pickPart(part part, std::string arm_name);
  bool pickPartLeftArm(part part);
  bool pickPartRightArm(part part);
  void placePartLeftArm();
  void placePartRightArm();

  bool pickPartFromTrayLeftArm(part part, std::string agv_id);
  bool correctPosePartLeftArm(Part part, geometry_msgs::Pose target, std::string agv_id);

  bool pickPartFromTrayRightArm(part part, std::string agv_id);
  bool correctPosePartRightArm(Part part, geometry_msgs::Pose target, std::string agv_id);

  bool throwPartLeft(part part);
  bool throwPartRight(part part);

  /// Send command message to robot controller
  bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
  void goToPresetLocation(PresetLocation location);
  void rotateTorso(const double angle);
  void reachPartShelfLeftArm(part part);
  void reachPartShelfRightArm(part part);
  void reachPartBinLeftArm(part part);
  void reachPartBinRightArm(part part);
  void goToBottomShelfs();
  void retriveFromBottomShelf();
  void FKLeftArm(std::vector<double> joints);
  void FKRightArm(std::vector<double> joints);
  void FKGantry(std::vector<double> joints);
  void moveOverPart(part part, std::string arm);
  void getProduct(product product);
  // void getProductsToFlip(std::vector<Part> partsToFlip);
  void flipProductsAGV(std::vector<Part> checkPartsToFlip);
  bool getPartConveyorLeftArm(product product);
  bool getPartConveyorRightArm(product product);
  void getLeftArmRoll();
  void getRightArmRoll();
  void moveLeftArmRoll(double roll);
  std::string checkFreeGripper();
  std::string getGantryLocation()
  {

    if (gantry_location_.size() == 0)
    {
      ROS_INFO_STREAM("its none");
    }
    return gantry_location_;
  }
  product getProductLeftArm()
  {
    return product_left_arm_;
  }
  product getProductRightArm()
  {
    return product_right_arm_;
  }

  std::vector<Product> getProductsTray1()
  {
    return products_kit_tray_1_;
  }

  std::vector<Product> getProductsTray2()
  {
    return products_kit_tray_2_;
  }

  void set_product_left_arm_(const product &product)
  {
    product_left_arm_ = product;
  }
  void set_product_right_arm_(const product &product)
  {
    product_right_arm_ = product;
  }

  void setGantryLocation(std::string location)
  {
    gantry_location_ = location;
  }


  void activateGripper(std::string gripper_id);
  void deactivateGripper(std::string gripper_id);
  nist_gear::VacuumGripperState getGripperState(std::string arm_name);
  geometry_msgs::Pose getTargetWorldPose(geometry_msgs::Pose target, std::string agv);
  //--preset locations;
  start start_;
  PresetLocation safe_spot_1_, safe_spot_2_;
  aisle1 aisle1_;
  aisle2 aisle2_;
  shelf1 shelf1_;
  shelf2 shelf2_;
  bins bins_;
  agv1 agv1_;
  agv2 agv2_;
  agv1_left agv1_left_;
  agv1_right agv1_right_;
  agv2_left agv2_left_;
  agv2_right agv2_right_;
  tray1_left_positive tray1_left_positive_;
  tray1_left_negative tray1_left_negative_;
  tray1_right_positive tray1_right_positive_;
  tray1_right_negative tray1_right_negative_;
  tray2_left_positive tray2_left_positive_;
  tray2_left_negative tray2_left_negative_;
  tray2_right_positive tray2_right_positive_;
  tray2_right_negative tray2_right_negative_;
  product product_left_arm_;
  product product_right_arm_;
  tf2::Quaternion q_left_ee_link_part;
  tf2::Quaternion q_right_ee_link_part;
  PresetLocation safe_spot_3_, ssi1, ssi2, ssi3, ssi4, ssi5, ssi6;

private:
  std::vector<double> joint_group_positions_;
  ros::NodeHandle node_;
  std::string planning_group_;
  moveit::planning_interface::MoveGroupInterface::Options full_robot_options_;
  moveit::planning_interface::MoveGroupInterface::Options gantry_options_;
  moveit::planning_interface::MoveGroupInterface::Options left_arm_options_;
  moveit::planning_interface::MoveGroupInterface::Options right_arm_options_;
  moveit::planning_interface::MoveGroupInterface::Options left_ee_link_options_;
  moveit::planning_interface::MoveGroupInterface::Options right_ee_link_options_;
  moveit::planning_interface::MoveGroupInterface full_robot_group_;
  moveit::planning_interface::MoveGroupInterface gantry_group_;
  moveit::planning_interface::MoveGroupInterface left_arm_group_;
  moveit::planning_interface::MoveGroupInterface right_arm_group_;
  moveit::planning_interface::MoveGroupInterface left_ee_link_group_;
  moveit::planning_interface::MoveGroupInterface right_ee_link_group_;

  double left_ee_roll_;
  double left_ee_pitch_;
  double left_ee_yaw_;
  std::array<float, 4> left_ee_quaternion_;
  std::string gantry_location_;
  std::vector<Product> products_to_flip_ {};
  std::vector<Product> products_kit_tray_1_ {};
  std::vector<Product> products_kit_tray_2_ {};
  std::array<geometry_msgs::TransformStamped, 9> shelf_w_transforms_{};
  std::array<int, 3> shelf_configuration {0,0,0};

  sensor_msgs::JointState current_joint_states_;

  // geometry_msgs::Pose last_placed_part_pose_; //Not Necessary

  nist_gear::VacuumGripperState current_left_gripper_state_;
  nist_gear::VacuumGripperState current_right_gripper_state_;

  control_msgs::JointTrajectoryControllerState current_gantry_controller_state_;
  control_msgs::JointTrajectoryControllerState current_left_arm_controller_state_;
  control_msgs::JointTrajectoryControllerState current_right_arm_controller_state_;

  ros::Publisher gantry_joint_trajectory_publisher_;
  ros::Publisher left_arm_joint_trajectory_publisher_;
  ros::Publisher right_arm_joint_trajectory_publisher_;

  ros::Subscriber joint_states_subscriber_;
  ros::Subscriber left_gripper_state_subscriber_;
  ros::Subscriber right_gripper_state_subscriber_;
  ros::Subscriber gantry_controller_state_subscriber_;
  ros::Subscriber left_arm_controller_state_subscriber_;
  ros::Subscriber right_arm_controller_state_subscriber_;

  ros::ServiceClient left_gripper_control_client;
  ros::ServiceClient right_gripper_control_client;

  // ---------- Callbacks ----------
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr &joint_state_msg);
  void left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &msg);
  void right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &msg);
  void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
  void left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
  void right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
};

#endif
