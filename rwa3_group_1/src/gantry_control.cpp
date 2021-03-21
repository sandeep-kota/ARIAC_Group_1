#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

/**
 * @brief Construct a new Gantry Control:: Gantry Control object
 * 
 * @param node 
 */
GantryControl::GantryControl(ros::NodeHandle &node) : node_("/ariac/gantry"),
                                                      planning_group_("/ariac/gantry/robot_description"),
                                                      full_robot_options_("Full_Robot", planning_group_, node_),
                                                      gantry_options_("Gantry", planning_group_, node_),
                                                      left_arm_options_("Left_Arm", planning_group_, node_),
                                                      right_arm_options_("Right_Arm", planning_group_, node_),
                                                      left_ee_link_options_("Left_Endeffector", planning_group_, node_),
                                                      right_ee_link_options_("Right_Endeffector", planning_group_, node_),
                                                      full_robot_group_(full_robot_options_),
                                                      gantry_group_(gantry_options_),
                                                      left_arm_group_(left_arm_options_),
                                                      right_arm_group_(right_arm_options_),
                                                      left_ee_link_group_(left_ee_link_options_),
                                                      right_ee_link_group_(right_ee_link_options_)
{
    ROS_INFO_STREAM("[GantryControl::GantryControl] constructor called... ");
}

////////////////////////////
void GantryControl::init()
{
    ROS_INFO_STREAM("[GantryControl::init] init... ");
    double time_called = ros::Time::now().toSec();

    ROS_INFO_NAMED("init", "Planning frame: %s", left_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", full_robot_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_ee_link_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", left_ee_link_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", gantry_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "End effector link: %s", left_arm_group_.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("init", "End effector link: %s", right_arm_group_.getEndEffectorLink().c_str());

    left_arm_group_.setPoseReferenceFrame("world");

    // preset locations

    std::vector<double> aisle_left_arm = {0.0, -PI, 3*PI/4, -3*PI/4, -PI/2, 0.};
    std::vector<double> aisle_right_arm = {PI, -PI, 3*PI/4, -3*PI/4, -PI/2, 0.};

    // joint positions to go to start location
    start_.location = "start";
    start_.gantry = {0, 0, 0};
    start_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    start_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    bins_.location = "bins";
    bins_.gantry = {4, 0, 0};
    bins_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bins_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    shelf1_.location = "shelf_1";
    shelf1_.gantry = {0, 4, -2.2};
    shelf1_.left_arm = aisle_left_arm;
    shelf1_.right_arm = aisle_right_arm;
    
    shelf2_.location = "shelf_2";
    shelf2_.gantry = {0, 4, -2.2};
    shelf2_.left_arm = aisle_left_arm;
    shelf2_.right_arm = aisle_right_arm;

    aisle1_.location = "aisle_1";
    aisle1_.gantry = {0, -1.5, 0.};
    aisle1_.left_arm = aisle_left_arm;
    aisle1_.right_arm = aisle_right_arm;

    aisle2_.location = "aisle_2";
    aisle2_.gantry = {0, 1.5, 0.};
    aisle2_.left_arm = aisle_left_arm;
    aisle2_.right_arm = aisle_right_arm;

    // joint positions to go to agv2
    agv2_.location = "agv2";
    agv2_.gantry = {0.0, 5.6, -PI/2};
    agv2_.left_arm = {PI, -1.51, -1.51, 0, PI / 2, 0};
    agv2_.right_arm = {PI, -1.51, -1.51, 0, PI / 2, 0};

    //--Raw pointers are frequently used to refer to the planning group for improved performance.
    //--To start, we will create a pointer that references the current robot’s state.
    const moveit::core::JointModelGroup *joint_model_group =
        full_robot_group_.getCurrentState()->getJointModelGroup("Full_Robot");

    //--Let’s set a joint space goal and move towards it.
    moveit::core::RobotStatePtr current_state = full_robot_group_.getCurrentState();

    //--Next get the current set of joint values for the group.
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

    gantry_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

    left_arm_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/left_arm_controller/command", 10);

    right_arm_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/right_arm_controller/command", 10);

    joint_states_subscriber_ = node_.subscribe(
        "/ariac/gantry/joint_states", 10, &GantryControl::joint_states_callback, this);

    left_gripper_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/left_arm/gripper/state", 10, &GantryControl::left_gripper_state_callback, this);

    right_gripper_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/right_arm/gripper/state", 10, &GantryControl::right_gripper_state_callback, this);

    gantry_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/gantry_controller/state", 10, &GantryControl::gantry_controller_state_callback, this);

    left_arm_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/left_arm_controller/state", 10, &GantryControl::left_arm_controller_state_callback, this);

    right_arm_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/right_arm_controller/state", 10, &GantryControl::right_arm_controller_state_callback, this);

    while ((current_gantry_controller_state_.joint_names.size() == 0) || 
    (current_left_arm_controller_state_.joint_names.size() == 0) || 
    (current_right_arm_controller_state_.joint_names.size() == 0))
    {
        ROS_WARN("[GantryControl::init] Waiting for first controller_state callbacks...");
        ros::Duration(0.1).sleep();
    }

    left_gripper_control_client =
        node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/left_arm/gripper/control");
    left_gripper_control_client.waitForExistence();

    right_gripper_control_client =
        node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/right_arm/gripper/control");
    right_gripper_control_client.waitForExistence();

    // Move robot to init position
    ROS_INFO("[GantryControl::init] Init position ready)...");
}
std::string GantryControl::checkFreeGripper()
{
    auto state_left = getGripperState("left_arm");
    auto state_right = getGripperState("right_arm");

    if (state_left.attached && state_right.attached)
    {
        return "none";
    } else if (state_left.attached && !state_right.attached)
    {
        return "right";
    } else if (!state_left.attached && state_right.attached)
    {
        return "left";
    } else
    {
        return "any";
    }   
}


void GantryControl::getProduct(product product)
{
    std::string location = product.p.location;
    std::string free_arm = checkFreeGripper();

    std::cout << gantry_location_ << free_arm << std::endl;
    //If part is located in the two top shelfs 1 and 2
    if (location == "shelf_1" || location == "shelf_2")
    {
        if (gantry_location_ == "aisle_1")
        {
            goToPresetLocation(aisle1_);
            FKGantry(start_.gantry);
        } else if (gantry_location_ == "aisle_2")
        {
            goToPresetLocation(aisle2_);
            FKGantry(start_.gantry);
        }

        if (location == "shelf_1")
        {
            goToPresetLocation(shelf1_);
        } else {
            goToPresetLocation(shelf2_);
        }

        if (free_arm == "any" || free_arm == "left")
        {
            reachPartShelfLeftArm(product.p);
            moveOverPart(product.p, free_arm);
            pickPartLeftArm(product.p);
            FKLeftArm(shelf1_.left_arm);
                
        } else {
            reachPartShelfRightArm(product.p);
            moveOverPart(product.p, free_arm);
            pickPartRightArm(product.p);
            FKRightArm(shelf1_.right_arm);
        }

        FKGantry(shelf1_.gantry);
        ros::Duration(0.5).sleep();
        rotateTorso(0.);   
    
    // If part is located in any of the bins
    } else if (location == "bins")
    {
        if (gantry_location_ == "aisle_1")
        {
            goToPresetLocation(aisle1_);
            goToPresetLocation(start_);
            
        } else if (gantry_location_ == "aisle_2")
        {
            goToPresetLocation(aisle2_);
            goToPresetLocation(start_);
        } else if (gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2")
        {
            FKRightArm(bins_.right_arm);
            FKLeftArm(bins_.left_arm);
        }

        if (free_arm == "any" || free_arm == "left")
        {
            reachPartBinLeftArm(product.p);
            ros::Duration(0.5).sleep();
            pickPartLeftArm(product.p);
            ros::Duration(1).sleep();
                
        } else {
            reachPartBinRightArm(product.p);
            ros::Duration(0.5).sleep();
            pickPartRightArm(product.p);
            ros::Duration(1).sleep();
        }

        gantry_location_ = "bins";

    //If part is located in bottom shelf 5
    } else if (location == "shelf_5")
    {
        if(gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "bins" || gantry_location_ == "start")
        {
            FKGantry(start_.gantry);
            goToPresetLocation(aisle1_);
        } else if (gantry_location_ == "aisle_2")
        {
            goToPresetLocation(aisle2_);
            goToPresetLocation(aisle1_);
        }

        goToBottomShelfs();
        if (free_arm == "any" || free_arm == "left")
        {
            reachPartShelfLeftArm(product.p);
            pickPartLeftArm(product.p);
            ros::Duration(1).sleep();
            retriveFromBottomShelf();
                
        } else {
            reachPartShelfRightArm(product.p);
            pickPartRightArm(product.p);
            retriveFromBottomShelf();
        }

        gantry_location_ = "aisle_1";

    //If part is located in bottom shelf 11
    } else if (location == "shelf_11")
    {
        if (gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "bins" || gantry_location_ == "start")
        {
            FKGantry(start_.gantry);
            goToPresetLocation(aisle1_);
        } else if (gantry_location_ == "aisle_1")
        {
            goToPresetLocation(aisle1_);
            goToPresetLocation(aisle2_);
        }

        goToBottomShelfs();
        if (free_arm == "any" || free_arm == "left")
        {
            reachPartShelfLeftArm(product.p);
            pickPartLeftArm(product.p);
            retriveFromBottomShelf();
                
        } else {
            reachPartShelfRightArm(product.p);
            pickPartRightArm(product.p);
            retriveFromBottomShelf();
        }

        gantry_location_ = "aisle_2";
    
    //If part is located in bottom shelf 8
    } else if (location == "shelf_8")
    {
        if (gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "bins" || gantry_location_ == "start")
        {
            FKGantry(start_.gantry);
            if (product.p.pose.position.y > 0)
            {
                goToPresetLocation(aisle1_);
                gantry_location_ = "aisle_1";
            } else {
                goToPresetLocation(aisle2_);
                gantry_location_ = "aisle_1";
            }
    }

    goToBottomShelfs();
    if (free_arm == "any" || free_arm == "left")
        {   
            reachPartShelfLeftArm(product.p);
            pickPartLeftArm(product.p);
            retriveFromBottomShelf();
                
        } else {
            reachPartShelfRightArm(product.p);
            pickPartRightArm(product.p);
            retriveFromBottomShelf();
        }
    }

}
    
////////////////////////////
geometry_msgs::Pose GantryControl::getTargetWorldPose(geometry_msgs::Pose target,
                                                      std::string agv)
{
    
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    std::string kit_tray;
    if (agv.compare("agv1") == 0)
        kit_tray = "kit_tray_1";
    else
        kit_tray = "kit_tray_2";
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "kit_tray_2";
    transformStamped.child_frame_id = "target_frame";
    transformStamped.transform.translation.x = target.position.x;
    transformStamped.transform.translation.y = target.position.y;
    transformStamped.transform.translation.z = target.position.z;
    transformStamped.transform.rotation.x = target.orientation.x;
    transformStamped.transform.rotation.y = target.orientation.y;
    transformStamped.transform.rotation.z = target.orientation.z;
    transformStamped.transform.rotation.w = target.orientation.w;

    
    for (int i{0}; i < 15; ++i)
        br.sendTransform(transformStamped);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // ros::Rate rate(10);
    ros::Duration timeout(5.0);

    geometry_msgs::TransformStamped world_target_tf;
    geometry_msgs::TransformStamped ee_target_tf;
    for (int i = 0; i < 10; i++)
    {
        try
        {
            world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                       ros::Time(0), timeout);
            // ROS_WARN_STREAM("target in world frame: " << world_target_tf.transform.rotation.x <<" "<<world_target_tf.transform.rotation.y<<" "<<world_target_tf.transform.rotation.z<<" "<<world_target_tf.transform.rotation.w);

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    for (int i = 0; i < 10; i++)
    {
        try
        {
            ee_target_tf = tfBuffer.lookupTransform("world", "left_ee_link",
                                                    ros::Time(0), timeout);
            // ROS_WARN_STREAM("left_ee_link in target frame: " << ee_target_tf.transform.rotation.x <<" "<<ee_target_tf.transform.rotation.y<<" "<<ee_target_tf.transform.rotation.z<<" "<<ee_target_tf.transform.rotation.w);

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

/**
 *ee_target_tf and  world_target_tf are expressed in the same frame: "world"
 We want to find the relative rotation, q_r, to go from ee_target_tf to world_target_tf
 q_r = world_target_tf*ee_target_tf_inverse
 * 
 */
    tf2::Quaternion ee_target_tf_inverse(
        ee_target_tf.transform.rotation.x,
        ee_target_tf.transform.rotation.y,
        ee_target_tf.transform.rotation.z,
        -ee_target_tf.transform.rotation.w);

    tf2::Quaternion part_in_tray(world_target_tf.transform.rotation.x,
                                 world_target_tf.transform.rotation.y,
                                 world_target_tf.transform.rotation.z,
                                 world_target_tf.transform.rotation.w);

    tf2::Quaternion qr = part_in_tray * ee_target_tf_inverse;
    qr.normalize();

    geometry_msgs::Pose world_target{target};
    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = ee_target_tf.transform.rotation.x;
    world_target.orientation.y = ee_target_tf.transform.rotation.y;
    world_target.orientation.z = ee_target_tf.transform.rotation.z;
    world_target.orientation.w = ee_target_tf.transform.rotation.w;

    // ros::Duration(10).sleep();
    return world_target;
}

void GantryControl::reachPartBinLeftArm(part part)
{

    geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;

    const double offset_y = part.pose.position.y - currentArmPose.position.y;
    
    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;
    joint_group_positions_.at(1) -= offset_y;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_x;
    bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
}
void GantryControl::reachPartBinRightArm(part part)
{
    geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;

    const double offset_y = part.pose.position.y - currentArmPose.position.y;
    
    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;
    joint_group_positions_.at(1) -= offset_y;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_x;
    bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
  
}
void GantryControl::reachPartShelfLeftArm(part part)
{
    double offset_y {};

    geometry_msgs::Pose currentGantryPose = gantry_group_.getCurrentPose().pose;

    const double dy = part.pose.position.y - currentGantryPose.position.y;

    if (dy > 0)
    {
        offset_y = 1.8 - dy;

        rotateTorso(L_LEFT_ARM);
    } else
    {
        offset_y = -1.8 - dy;
        rotateTorso(R_LEFT_ARM);
    }
    FKLeftArm({0, -2.13, 1.49, -2.48, -1.57, 0});
    geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;

    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_x;
    bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();

    joint_group_positions_.at(1) += offset_y;
    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_y;
    success = (full_robot_group_.plan(move_y) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();

}

void GantryControl::reachPartShelfRightArm(part part)
{
    double offset_y {};

    geometry_msgs::Pose currentGantryPose = gantry_group_.getCurrentPose().pose;

    const double dy = part.pose.position.y - currentGantryPose.position.y;

    if (dy > 0)
    {
        offset_y = 1.8 - dy;

        rotateTorso(L_RIGHT_ARM);
        //--left arm
    } else
    {
        offset_y = -1.8 - dy;
        rotateTorso(R_RIGHT_ARM);
        //--left arm
    }
    FKRightArm({PI, -2.13, 1.49, -2.48, -1.57, 0});
    geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;

    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_x;
    bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();

    joint_group_positions_.at(1) += offset_y;
    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_y;
    success = (full_robot_group_.plan(move_y) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();           
}

void GantryControl::retriveFromBottomShelf()
{

  FKLeftArm(aisle1_.left_arm);
  FKRightArm(aisle1_.right_arm);
  goToBottomShelfs();
  rotateTorso(0.);
}

void GantryControl::FKLeftArm(std::vector<double> joints)
{
    joint_group_positions_.at(3) = joints.at(0);
    joint_group_positions_.at(4) = joints.at(1);
    joint_group_positions_.at(5) = joints.at(2);
    joint_group_positions_.at(6) = joints.at(3);
    joint_group_positions_.at(7) = joints.at(4);
    joint_group_positions_.at(8) = joints.at(5);

    full_robot_group_.setJointValueTarget(joint_group_positions_);
    full_robot_group_.move(); 
}
void GantryControl::FKRightArm(std::vector<double> joints)
{
    joint_group_positions_.at(9) = joints.at(0);
    joint_group_positions_.at(10) = joints.at(1);
    joint_group_positions_.at(11) = joints.at(2);
    joint_group_positions_.at(12) = joints.at(3);
    joint_group_positions_.at(13) = joints.at(4);
    joint_group_positions_.at(14) = joints.at(5);

    full_robot_group_.setJointValueTarget(joint_group_positions_);
    full_robot_group_.move(); 
}

void GantryControl::FKGantry(std::vector<double> joints)
{
    joint_group_positions_.at(0) = joints.at(0);
    joint_group_positions_.at(1) = joints.at(1);

    full_robot_group_.setJointValueTarget(joint_group_positions_);
    full_robot_group_.move();
}
////
void GantryControl::goToBottomShelfs()
{
    joint_group_positions_.at(0) = -14.5;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move(); 
}

void GantryControl::moveOverPart(part part, std::string arm)
{   
    geometry_msgs::Pose currentPose {};
    if (arm == "left")
    {
        currentPose  = left_arm_group_.getCurrentPose().pose;
    } else {
        currentPose  = right_arm_group_.getCurrentPose().pose;
    }    
    part.pose.position.z += 0.5; 
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;

    if (arm == "left")
    {
        left_arm_group_.setPoseTarget(part.pose);
        left_arm_group_.move();
    } else {
        right_arm_group_.setPoseTarget(part.pose);
        right_arm_group_.move();
    }    

    ros::Duration(0.5).sleep();
}
////////////////////////
bool GantryControl::pickPartLeftArm(part part)
{
    
    //--Activate gripper
    activateGripper("left_arm");
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;


    tf2::Quaternion part_orientation (part.pose.orientation.x, part.pose.orientation.y, part.pose.orientation.z, part.pose.orientation.w);
    part_orientation.normalize();

    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;

    tf2::Quaternion ee_link_orientation (currentPose.orientation.x, currentPose.orientation.y ,currentPose.orientation.z,
                                            currentPose.orientation.w);

    ee_link_orientation.normalize();
    qr_part_left_arm_ = part_orientation * ee_link_orientation.inverse();

    qr_part_left_arm_.normalize();
   

    //    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);

    auto state = getGripperState("left_arm");
    if (state.enabled)
    {
        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        left_arm_group_.setPoseTarget(part.pose);
        left_arm_group_.move();
        auto state = getGripperState("left_arm");
        if (state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object attached");
                
            //--Move arm to previous position
            left_arm_group_.setPoseTarget(currentPose);
            left_arm_group_.move();
        }
        else
        {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            int max_attempts{5};
            int current_attempt{0};
            //--try to pick up the part 5 times
            while (current_attempt<max_attempts)
            {
                left_arm_group_.setPoseTarget(currentPose);
                left_arm_group_.move();
                ros::Duration(0.5).sleep();
                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();
                
                activateGripper("left_arm");
                current_attempt++;
            }
            left_arm_group_.setPoseTarget(currentPose);
            left_arm_group_.move();
        }
            
    }
    else
    {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }
    return false;
}

bool GantryControl::pickPartRightArm(part part)
{
    
    //--Activate gripper
    activateGripper("right_arm");
    geometry_msgs::Pose currentPose = right_arm_group_.getCurrentPose().pose;
    std::cout << model_height.at(part.type);
    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;
    //    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);

    auto state = getGripperState("right_arm");
    if (state.enabled)
    {
        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        right_arm_group_.setPoseTarget(part.pose);
        right_arm_group_.move();
        auto state = getGripperState("right_arm");
        if (state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object attached");
            //--Move arm to previous position
            right_arm_group_.setPoseTarget(currentPose);
            right_arm_group_.move();
            // goToPresetLocation(start_);
            return true;
        }
        else
        {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            int max_attempts{5};
            int current_attempt{0};
            //--try to pick up the part 5 times
            while (current_attempt<max_attempts)
            {
                right_arm_group_.setPoseTarget(currentPose);
                right_arm_group_.move();
                ros::Duration(0.5).sleep();
                right_arm_group_.setPoseTarget(part.pose);
                right_arm_group_.move();
                activateGripper("right_arm");
                current_attempt++;
            }
            right_arm_group_.setPoseTarget(currentPose);
            right_arm_group_.move();
        }
    }
    else
    {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }
    return false;
}
////////////////////////////
void GantryControl::placePart(part part, std::string agv)
{
    auto target_pose_in_tray = getTargetWorldPose(part.pose, agv);

    tf2::Quaternion target_orientation_in_tray (target_pose_in_tray.orientation.x, target_pose_in_tray.orientation.y, target_pose_in_tray.orientation.z,
                                                    target_pose_in_tray.orientation.w);

    // target_orientation_in_tray = qr_part_left_arm_ * target_orientation_in_tray;

    ROS_INFO_STREAM(target_pose_in_tray);
    ros::Duration(2.0).sleep();
    //--TODO: Consider agv1 too
    if (agv == "agv2")
        goToPresetLocation(agv2_);
    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[part.type]);

    
    target_pose_in_tray.orientation.x = target_orientation_in_tray.x();
    target_pose_in_tray.orientation.y = target_orientation_in_tray.y();
    target_pose_in_tray.orientation.z = target_orientation_in_tray.z();
    target_pose_in_tray.orientation.w = target_orientation_in_tray.w();
    
    left_arm_group_.setPoseTarget(target_pose_in_tray);
    left_arm_group_.move();

    deactivateGripper("left_arm");
    auto state = getGripperState("left_arm");
    if (state.attached)
        goToPresetLocation(start_);
}

////////////////////////////
void GantryControl::goToPresetLocation(PresetLocation location)
{
    //--gantry
    joint_group_positions_.at(0) = location.gantry.at(0);
    joint_group_positions_.at(1) = location.gantry.at(1);
    joint_group_positions_.at(2) = location.gantry.at(2);
    //--left arm
    joint_group_positions_.at(3) = location.left_arm.at(0);
    joint_group_positions_.at(4) = location.left_arm.at(1);
    joint_group_positions_.at(5) = location.left_arm.at(2);
    joint_group_positions_.at(6) = location.left_arm.at(3);
    joint_group_positions_.at(7) = location.left_arm.at(4);
    joint_group_positions_.at(8) = location.left_arm.at(5);
    //--right arm
    joint_group_positions_.at(9) = location.right_arm.at(0);
    joint_group_positions_.at(10) = location.right_arm.at(1);
    joint_group_positions_.at(11) = location.right_arm.at(2);
    joint_group_positions_.at(12) = location.right_arm.at(3);
    joint_group_positions_.at(13) = location.right_arm.at(4);
    joint_group_positions_.at(14) = location.right_arm.at(5);

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
    
    gantry_location_ = location.location;
}

/////////
void GantryControl::rotateTorso(const double angle)
{   
    joint_group_positions_.at(2) = angle;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
}


////////////////////////////
void GantryControl::activateGripper(std::string arm_name)
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;

    if (arm_name == "left_arm")
    {
        left_gripper_control_client.call(srv);
    }
    else
    {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][activateGripper] DEBUG: srv.response =" << srv.response);
}

////////////////////////////
void GantryControl::deactivateGripper(std::string arm_name)
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;

    if (arm_name == "left_arm")
    {
        left_gripper_control_client.call(srv);
    }
    else
    {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][deactivateGripper] DEBUG: srv.response =" << srv.response);
}

////////////////////////////
nist_gear::VacuumGripperState GantryControl::getGripperState(std::string arm_name)
{
    if (arm_name == "left_arm")
    {
        return current_left_gripper_state_;
    }
    else
    {
        return current_right_gripper_state_;
    }
}

////////////////////////////
void GantryControl::left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &gripper_state_msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_left_gripper_state_ = *gripper_state_msg;
}

////////////////////////////
void GantryControl::right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &gripper_state_msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_right_gripper_state_ = *gripper_state_msg;
}

////////////////////////////
void GantryControl::joint_states_callback(const sensor_msgs::JointState::ConstPtr &joint_state_msg)
{
    if (joint_state_msg->position.size() == 0)
    {
        ROS_ERROR("[gantry_control][joint_states_callback] msg->position.size() == 0!");
    }
    current_joint_states_ = *joint_state_msg;
}

////////////////////////////
void GantryControl::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gantry controller states (throttled to 0.1 Hz):\n" << *msg);
    current_gantry_controller_state_ = *msg;
}

////////////////////////////
void GantryControl::left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Left arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_left_arm_controller_state_ = *msg;
}

////////////////////////////
void GantryControl::right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Right arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_right_arm_controller_state_ = *msg;
}

////////////////////////////
bool GantryControl::sendJointPosition(trajectory_msgs::JointTrajectory command_msg)
{
    // ROS_INFO_STREAM("[gantry_control][sendJointPosition] called.");

    if (command_msg.points.size() == 0)
    {
        ROS_WARN("[gantry_control][sendJointPosition] Trajectory is empty or NAN, returning.");
        return false;
    }
    else if ((command_msg.joint_names[0] == "small_long_joint") // command is for gantry
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0]))
    {

        gantry_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][sendJointPosition] gantry command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0, 4) == "left") // command is for left_arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0]))
    {

        left_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][sendJointPosition] left_arm command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0, 5) == "right") // command is for right arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0]))
    {

        right_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][sendJointPosition] right_arm command published!");
        return true;
    }
    else
    {
        return false;
    }
}

void GantryControl::printPartOrient()
{
    geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;

    tf2::Quaternion q_arm (currentArmPose.orientation.x, currentArmPose.orientation.y, currentArmPose.orientation.z, currentArmPose.orientation.w);

    q_arm.normalize();

    tf2::Quaternion q_part;

    q_part = qr_part_left_arm_ * q_arm;
    q_part.normalize();

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_part).getRPY(yaw, pitch, roll);

    ROS_INFO_STREAM("q of part with respect to world: " << roll << " " << pitch << " " << yaw);
}
