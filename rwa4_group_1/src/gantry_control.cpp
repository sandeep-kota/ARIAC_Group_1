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

    std::vector<double> aisle_left_arm = {0.0, -PI, 3 * PI / 4, -3 * PI / 4, -PI / 2, 0.};
    std::vector<double> aisle_right_arm = {PI, -PI, 3 * PI / 4, -3 * PI / 4, -PI / 2, 0.};

    // joint positions to go to start location
    start_.location = "start";
    start_.gantry = {0, 0, 0};
    start_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    start_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    // // joint positions to go to middle of bins
    bins_.location = "bins";
    bins_.gantry = {4, 0, 0};
    bins_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bins_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to top left shelf
    shelf1_.location = "shelf_1";
    shelf1_.gantry = {0, 4, -2.2};
    shelf1_.left_arm = aisle_left_arm;
    shelf1_.right_arm = aisle_right_arm;

    // joint positions to go to top right shelf
    shelf2_.location = "shelf_2";
    shelf2_.gantry = {0, 4, -2.2};
    shelf2_.left_arm = aisle_left_arm;
    shelf2_.right_arm = aisle_right_arm;

    // joint positions to go to aisle1 (between shelf 5 and 8)
    aisle1_.location = "aisle_1";
    aisle1_.gantry = {0, -1.5, 0.};
    aisle1_.left_arm = aisle_left_arm;
    aisle1_.right_arm = aisle_right_arm;

    // joint positions to go to aisle2 (between shelf 8 and 11)
    aisle2_.location = "aisle_2";
    aisle2_.gantry = {0, 1.5, 0.};
    aisle2_.left_arm = aisle_left_arm;
    aisle2_.right_arm = aisle_right_arm;

    // joint positions to go to agv2  // joint positions to go to agv2
     agv2_.location = "agv2";
    agv2_.gantry = {0, 5.5, 0};
    agv2_.left_arm = start_.left_arm;
    agv2_.right_arm = start_.right_arm;

    agv2_left_.location = "agv2";
    agv2_left_.gantry = {0, 5.5, -PI / 2};
    agv2_left_.left_arm = start_.left_arm;
    agv2_left_.right_arm = start_.right_arm;
    agv2_right_.location = "agv2";
    agv2_right_.gantry = {0, 5.5, PI / 2};
    agv2_right_.left_arm = start_.left_arm;
    agv2_right_.right_arm = start_.right_arm;

    // joint positions to go to agv1
    agv1_.location = "agv1";
    agv1_.gantry = {0, -5.5, 0};
    agv1_.left_arm = start_.left_arm;
    agv1_.right_arm = start_.right_arm;

    agv1_left_.location = "agv1";
    agv1_left_.gantry = {0, -5.5, PI / 2};
    agv1_left_.left_arm = start_.left_arm;
    agv1_left_.right_arm = start_.right_arm;
    agv1_right_.location = "agv1";
    agv1_right_.gantry = {0, -5.5, -PI / 2};
    agv1_right_.left_arm = start_.left_arm;
    agv1_right_.right_arm = start_.right_arm;

       // joint positions to go to tray1
    tray1_left_negative_.location = "tray1";
    tray1_left_negative_.gantry = {0, -5.5, PI/4};
    tray1_left_negative_.left_arm = start_.left_arm;
    tray1_left_negative_.right_arm = start_.right_arm;

    tray1_left_positive_.location = "tray1";
    tray1_left_positive_.gantry = {0, -5.5, PI/4 + PI/2};
    tray1_left_positive_.left_arm = start_.left_arm;
    tray1_left_positive_.right_arm = start_.right_arm;

    tray1_right_negative_.location = "tray1";
    tray1_right_negative_.gantry = {0, -5.5, -PI/4-PI/2};
    tray1_right_negative_.left_arm = start_.left_arm;
    tray1_right_negative_.right_arm = start_.right_arm;

    tray1_right_positive_.location = "tray1";
    tray1_right_positive_.gantry = {0, -5.5, -PI/4};
    tray1_right_positive_.left_arm = start_.left_arm;
    tray1_right_positive_.right_arm = start_.right_arm;

    // joint positions to go to tray2
    tray2_left_positive_.location = "tray2";
    tray2_left_positive_.gantry = {0, 5.5, -PI/4};
    tray2_left_positive_.left_arm = start_.left_arm;
    tray2_left_positive_.right_arm = start_.right_arm;

    tray2_left_negative_.location = "tray2";
    tray2_left_negative_.gantry = {0, 5.5, -PI/4-PI/2};
    tray2_left_negative_.left_arm = start_.left_arm;
    tray2_left_negative_.right_arm = start_.right_arm;

    tray2_right_positive_.location = "tray2";
    tray2_right_positive_.gantry = {0, 5.5, PI/4+PI/2};
    tray2_right_positive_.left_arm = start_.left_arm;
    tray2_right_positive_.right_arm = start_.right_arm;

    tray2_right_negative_.location = "tray2";
    tray2_right_negative_.gantry = {0, 5.5, PI/4};
    tray2_right_negative_.left_arm = start_.left_arm;
    tray2_right_negative_.right_arm = start_.right_arm;

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
/**
 * @brief check which of the grippers are free
 * 
 * @return std::string "any" if all, "none" if none of them free, "left" and "right"
 */
std::string GantryControl::checkFreeGripper()
{
    auto state_left = getGripperState("left_arm");
    auto state_right = getGripperState("right_arm");

    std::string val = "any";

    if (state_left.attached && state_right.attached)
    {
        val = "none";
    }
    else if (state_left.attached && !state_right.attached)
    {
        val = "right";
    }
    else if (!state_left.attached && state_right.attached)
    {
        val = "left";
    }

    return val;
}

/**
 * @brief retrieve a specific product detected from the sensors from wherever in the environment
 * 
 * @param product product of product list to be found and retrieved
 */
void GantryControl::getProduct(product product)
{
    std::string location = product.p.location;
    std::string free_arm = checkFreeGripper();

    //If part is located in the two top shelfs 1 and 2
    if (location == "shelf_1" || location == "shelf_2")
    {
        if (gantry_location_ == "aisle_1")
        {
            goToPresetLocation(aisle1_);
            FKGantry(start_.gantry);
        }
        else if (gantry_location_ == "aisle_2")
        {
            goToPresetLocation(aisle2_);
            FKGantry(start_.gantry);
        }

        if (location == "shelf_1")
        {
            goToPresetLocation(shelf1_);
        }
        else
        {
            goToPresetLocation(shelf2_);
        }

        if (free_arm == "any" || free_arm == "left")
        {
            reachPartShelfLeftArm(product.p);
            moveOverPart(product.p, free_arm);
            pickPartLeftArm(product.p);
            FKLeftArm(shelf1_.left_arm);
        }
        else
        {
            reachPartShelfRightArm(product.p);
            moveOverPart(product.p, free_arm);
            pickPartRightArm(product.p);
            FKRightArm(shelf1_.right_arm);
        }

        FKGantry(shelf1_.gantry);
        ros::Duration(0.5).sleep();
        rotateTorso(0.);

        // If part is located in any of the bins
    }
    else if (location == "bins")
    {
        if (gantry_location_ == "aisle_1")
        {
            goToPresetLocation(aisle1_);
            goToPresetLocation(start_);
        }
        else if (gantry_location_ == "aisle_2")
        {
            goToPresetLocation(aisle2_);
            goToPresetLocation(start_);
        }
        else if (gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2")
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
        }
        else
        {
            reachPartBinRightArm(product.p);
            ros::Duration(0.5).sleep();
            pickPartRightArm(product.p);
            ros::Duration(1).sleep();
        }
        goToPresetLocation(bins_);
        gantry_location_ = "bins";

        //If part is located in bottom shelf 5
    }
    else if (location == "shelf_5")
    {
        if (gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "bins" || gantry_location_ == "start")
        {
            FKGantry(start_.gantry);
            goToPresetLocation(aisle1_);
            gantry_location_ = "aisle_1";
        }
        else if (gantry_location_ == "aisle_2")
        {
            goToPresetLocation(aisle2_);
            goToPresetLocation(aisle1_);
            gantry_location_ = "aisle_1";
        }

        goToBottomShelfs();
        if (free_arm == "any" || free_arm == "left")
        {
            reachPartShelfLeftArm(product.p);
            ros::Duration(1).sleep();
            pickPartLeftArm(product.p);
            ros::Duration(1).sleep();
            retriveFromBottomShelf();
        }
        else
        {
            reachPartShelfRightArm(product.p);
            ros::Duration(1).sleep();
            pickPartRightArm(product.p);
            ros::Duration(1).sleep();
            retriveFromBottomShelf();
        }

        gantry_location_ = "aisle_1";

        //If part is located in bottom shelf 11
    }
    else if (location == "shelf_11")
    {
        if (gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "bins" || gantry_location_ == "start")
        {
            FKGantry(start_.gantry);
            goToPresetLocation(aisle1_);
        }
        else if (gantry_location_ == "aisle_1")
        {
            goToPresetLocation(aisle1_);
            goToPresetLocation(aisle2_);
            gantry_location_ = "aisle_2";
        }

        goToBottomShelfs();
        if (free_arm == "any" || free_arm == "left")
        {
            reachPartShelfLeftArm(product.p);
            ros::Duration(1).sleep();
            pickPartLeftArm(product.p);
            retriveFromBottomShelf();
        }
        else
        {
            reachPartShelfRightArm(product.p);
            ros::Duration(1).sleep();
            pickPartRightArm(product.p);
            ros::Duration(1).sleep();
            retriveFromBottomShelf();
        }

        gantry_location_ = "aisle_2";

        //If part is located in bottom shelf 8
    }
    else if (location == "shelf_8")
    {
        if (gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "bins" || gantry_location_ == "start")
        {
            FKGantry(start_.gantry);
            if (product.p.pose.position.y > 0)
            {
                goToPresetLocation(aisle1_);
                gantry_location_ = "aisle_1";
            }
            else
            {
                goToPresetLocation(aisle2_);
                gantry_location_ = "aisle_2";
            }
        }

        goToBottomShelfs();
        if (free_arm == "any" || free_arm == "left")
        {
            reachPartShelfLeftArm(product.p);
            ros::Duration(1).sleep();
            pickPartLeftArm(product.p);
            ros::Duration(1).sleep();
            retriveFromBottomShelf();
        }
        else
        {
            reachPartShelfRightArm(product.p);
            ros::Duration(1).sleep();
            pickPartRightArm(product.p);
            ros::Duration(1).sleep();
            retriveFromBottomShelf();
        }
    }

    //add product to arm
    if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
    {
        product_left_arm_ = product;
    }
    else
    {
        product_right_arm_ = product;
    }
}

/**
 * @brief Gets the pose in pose of part in world frame
 * 
 * @param target Part Pose
 * @param agv AGV ID
 * @return geometry_msgs::Pose 
 */
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
    transformStamped.header.frame_id = kit_tray;
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
/**
 * @brief Reach the bin with left arm
 * 
 * @param Part part object
 */
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

/**
 * @brief Reach bin with right arm
 * 
 * @param part Part object
 */
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

/**
 * @brief Reach Shelf with left arm
 * 
 * @param part part object
 */
void GantryControl::reachPartShelfLeftArm(part part)
{
    double offset_y{};

    geometry_msgs::Pose currentGantryPose = gantry_group_.getCurrentPose().pose;

    const double dy = part.pose.position.y - currentGantryPose.position.y;

    if (dy > 0)
    {
        offset_y = 1.8 - dy;

        rotateTorso(L_LEFT_ARM);
    }
    else
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

/**
 * @brief Reach part with right arm
 * 
 * @param part Part object
 */
void GantryControl::reachPartShelfRightArm(part part)
{
    double offset_y{};

    geometry_msgs::Pose currentGantryPose = gantry_group_.getCurrentPose().pose;

    const double dy = part.pose.position.y - currentGantryPose.position.y;

    if (dy > 0)
    {
        offset_y = 1.8 - dy;

        rotateTorso(L_RIGHT_ARM);
        //--left arm
    }
    else
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

/**
 * @brief Get back to the aisle after collecting the part.
 * 
 */
void GantryControl::retriveFromBottomShelf()
{

    FKLeftArm(aisle1_.left_arm);
    FKRightArm(aisle1_.right_arm);
    goToBottomShelfs();
    rotateTorso(0.);
}

/**
 * @brief Move left arm to the joint angles.
 * 
 * @param joints Joint angles for the left arm
 */
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

/**
 * @brief Move right arm to the joint angles.
 * 
 * @param joints Joint angles for the right arm
 */
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

/**
 * @brief Move the gantry to a location
 * 
 * @param joints Joint angles for gantry
 */
void GantryControl::FKGantry(std::vector<double> joints)
{
    joint_group_positions_.at(0) = joints.at(0);
    joint_group_positions_.at(1) = joints.at(1);

    full_robot_group_.setJointValueTarget(joint_group_positions_);
    full_robot_group_.move();
}

/**
 * @brief Enter the bottom shelf
 * 
 */
void GantryControl::goToBottomShelfs()
{
    joint_group_positions_.at(0) = -14.5;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
}

/**
 * @brief Move over the part
 * 
 * @param part Part object
 * @param arm Arm name
 * 
 */
void GantryControl::moveOverPart(part part, std::string arm)
{
    geometry_msgs::Pose currentPose{};
    if (arm == "left")
    {
        currentPose = left_arm_group_.getCurrentPose().pose;
    }
    else
    {
        currentPose = right_arm_group_.getCurrentPose().pose;
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
    }
    else
    {
        right_arm_group_.setPoseTarget(part.pose);
        right_arm_group_.move();
    }

    ros::Duration(0.5).sleep();
}

/**
 * @brief Pick part with left arm
 * 
 * @param part Part object to be picked
 * @return true 
 * @return false 
 */
bool GantryControl::pickPartLeftArm(part part)
{

    //--Activate gripper
    activateGripper("left_arm");
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;

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
            while (current_attempt < max_attempts)
            {
                left_arm_group_.setPoseTarget(currentPose);
                left_arm_group_.move();
                ros::Duration(0.5).sleep();
                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();

                activateGripper("left_arm");
                current_attempt++;
            }
            part.picked_status = true;

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

/**
 * @brief Pick the part with right arm
 * 
 * @param part Part Object to be picked
 * @return true 
 * @return false 
 */
bool GantryControl::pickPartRightArm(part part)
{

    //--Activate gripper
    activateGripper("right_arm");
    geometry_msgs::Pose currentPose = right_arm_group_.getCurrentPose().pose;
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
            while (current_attempt < max_attempts)
            {
                right_arm_group_.setPoseTarget(currentPose);
                right_arm_group_.move();
                ros::Duration(0.5).sleep();
                right_arm_group_.setPoseTarget(part.pose);
                right_arm_group_.move();
                activateGripper("right_arm");
                current_attempt++;
            }
            //part.pose.position.z += 0.05;
            part.picked_status = true;

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

/**
 * @brief Throw part from left hand if faulty
 * 
 * @param part Part object of the faulty part
 * @param ptype Type of the part obtained from the orders list.
 * @return true 
 * @return false 
 */
bool GantryControl::throwLastPartLeft(part part)
{

    ROS_WARN_STREAM("Faulty Part: " << product_left_arm_.type);
    ROS_WARN_STREAM("FAULTY POSE Z: " << part.pose.position.z);

    part.pose.position.z += 0.015;
    part.type = product_left_arm_.type;
    if (product_left_arm_.agv_id.compare("agv2") == 0 || product_left_arm_.agv_id.compare("any") == 0)
    {   
        goToPresetLocation(agv2_);
        if (part.pose.position.x >= 0)
        {
            goToPresetLocation(tray2_left_positive_);
        } else {
            goToPresetLocation(tray2_left_negative_);
        }
    }
    else
    {
        goToPresetLocation(agv1_);
        if (-part.pose.position.x >= 0)
        {
            goToPresetLocation(tray1_left_positive_);
        } else {
            goToPresetLocation(tray1_left_negative_);
        }
    }


    part.pose.orientation.x = 0;
    part.pose.orientation.y = 0.707;
    part.pose.orientation.z = 0;
    part.pose.orientation.w = 0.707;

    geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;

    // // //--TODO: Consider agv1 too

    ROS_WARN_STREAM("TYPE: " << product_left_arm_.type);

    const double offset_y = part.pose.position.y - currentArmPose.position.y;

    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;
    joint_group_positions_.at(1) -= offset_y;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_x;
    bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();

    ros::Duration(1).sleep();
    pickPartLeftArm(part);
    ros::Duration(1).sleep();

    auto state = getGripperState("left_arm");
    if (state.attached){
        if (product_left_arm_.agv_id.compare("agv2") == 0 || product_left_arm_.agv_id.compare("any") == 0)
        {
            goToPresetLocation(agv2_);
        }
        else
        {
            goToPresetLocation(agv1_);
        }

        deactivateGripper("left_arm");
    }

    
}


/**
 * @brief Throw part from right hand if faulty
 * 
 * @param part Part object of the faulty part
 * @return true 
 * @return false 
 */
bool GantryControl::throwLastPartRight(part part)
{
    ROS_WARN_STREAM("Faulty Part: " << product_right_arm_.type);

    part.pose.position.z += 0.015;
    part.type = product_right_arm_.type;
    if (product_right_arm_.agv_id.compare("agv2") == 0 || product_right_arm_.agv_id.compare("any") == 0)
    {
        goToPresetLocation(agv2_);
        if (part.pose.position.x >= 0)
        {
            goToPresetLocation(tray2_right_positive_);
        } else {
            goToPresetLocation(tray2_right_negative_);
        }
    }
    else
    {
        goToPresetLocation(agv1_);
        if (-part.pose.position.x >= 0)
        {
            goToPresetLocation(tray1_right_positive_);
        } else {
            goToPresetLocation(tray1_right_negative_);
        }
    }


    // // ROS_WARN_STREAM("TYPE: " << product_right_arm_.type);
    part.pose.orientation.x = 0;
    part.pose.orientation.y = 0.707;
    part.pose.orientation.z = 0;
    part.pose.orientation.w = 0.707;

    geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;

    // // //--TODO: Consider agv1 too


    const double offset_y = part.pose.position.y - currentArmPose.position.y;

    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;
    joint_group_positions_.at(1) -= offset_y;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_x;
    bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();

    ros::Duration(1).sleep();
    pickPartRightArm(part);
    ros::Duration(1).sleep();

    auto state = getGripperState("right_arm");
    if (state.attached){
        if (product_right_arm_.agv_id.compare("agv2") == 0 || product_right_arm_.agv_id.compare("any") == 0)
        {
            goToPresetLocation(agv2_);
        }
        else
        {
            goToPresetLocation(agv1_);
        }
        deactivateGripper("right_arm");
    }    
}


/**
 * @brief Empty parts from left arm to corresponding AGV
 * 
 */
void GantryControl::placePartLeftArm()
{

    if (product_left_arm_.agv_id.compare("agv2") == 0 || product_left_arm_.agv_id.compare("any") == 0)
    {   
        goToPresetLocation(agv2_);
        if (product_left_arm_.pose.position.x >= 0)
        {
            goToPresetLocation(tray2_left_positive_);
        } else {
            goToPresetLocation(tray2_left_negative_);
        }
    }
    else
    {
        goToPresetLocation(agv1_);
        if (product_left_arm_.pose.position.x >= 0)
        {
            goToPresetLocation(tray1_left_positive_);
        } else {
            goToPresetLocation(tray1_left_negative_);
        }
    }

    // // ROS_WARN_STREAM("TYPE: " << product_left_arm_.type);
    auto target_pose_in_tray = getTargetWorldPose(product_left_arm_.pose, product_left_arm_.agv_id);

    geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;

    // // //--TODO: Consider agv1 too

    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[product_left_arm_.type]);

    const double offset_y = target_pose_in_tray.position.y - currentArmPose.position.y;

    const double offset_x = target_pose_in_tray.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;
    joint_group_positions_.at(1) -= offset_y;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (full_robot_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();

    ros::Duration(0.5).sleep();

    left_arm_group_.setPoseTarget(target_pose_in_tray);
    left_arm_group_.move();

    deactivateGripper("left_arm");

    ros::Duration(0.5).sleep();

    std::vector<double> retrieve {joint_group_positions_.at(0) -= offset_x, joint_group_positions_.at(1) += offset_y};

    FKGantry(retrieve);  


}

/**
 * @brief Empty parts from right arm to corresponding AGV
 * 
 */
void GantryControl::placePartRightArm()
{

        if (product_right_arm_.agv_id.compare("agv2") == 0 || product_right_arm_.agv_id.compare("any") == 0)
    {
        goToPresetLocation(agv2_);
        if (product_right_arm_.pose.position.x >= 0)
        {
            goToPresetLocation(tray2_right_positive_);
        } else {
            goToPresetLocation(tray2_right_negative_);
        }
    }
    else
    {
        goToPresetLocation(agv1_);
        if (product_right_arm_.pose.position.x >= 0)
        {
            goToPresetLocation(tray1_right_positive_);
        } else {
            goToPresetLocation(tray1_right_negative_);
        }
    }

    auto target_pose_in_tray = getTargetWorldPose(product_right_arm_.pose, product_right_arm_.agv_id);

    geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;

    // // //--TODO: Consider agv1 too

    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[product_right_arm_.type]);

    const double offset_y = target_pose_in_tray.position.y - currentArmPose.position.y;

    const double offset_x = target_pose_in_tray.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;
    joint_group_positions_.at(1) -= offset_y;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_x;
    bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();

    ros::Duration(0.5).sleep();

    right_arm_group_.setPoseTarget(target_pose_in_tray);
    right_arm_group_.move();

    deactivateGripper("right_arm");

     ros::Duration(0.5).sleep();

    std::vector<double> retrieve {joint_group_positions_.at(0) -= offset_x, joint_group_positions_.at(1) += offset_y};

    FKGantry(retrieve);
}

/**
 * @brief Move gantry to a pre-defined location
 * 
 * @param location PreSetLocation object 
 */
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
    full_robot_group_.setGoalTolerance(0.0001);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();

    gantry_location_ = location.location;
}

/**
 * @brief Rotate the TORSO by an angle
 * 
 * @param angle Angle to rotate
 */
void GantryControl::rotateTorso(const double angle)
{
    joint_group_positions_.at(2) = angle;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
}

/**
 * @brief Activate the gripper
 * 
 * @param arm_name Left or right gripper
 */
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

/**
 * @brief Deactivate Gripper
 * 
 * @param arm_name Left or right
 */
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

/**
 * @brief Get gripper state
 * 
 * @param arm_name Left or Right
 * @return nist_gear::VacuumGripperState 
 */
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

/**
 * @brief Callback to get gripper state
 * 
 * @param gripper_state_msg Subscribed Message
 */
void GantryControl::left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &gripper_state_msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_left_gripper_state_ = *gripper_state_msg;
}

/**
 * @brief Callback to get gripper state
 * 
 * @param gripper_state_msg Subscribed Message
 */
void GantryControl::right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr &gripper_state_msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_right_gripper_state_ = *gripper_state_msg;
}

/**
 * @brief Callback to get joint states
 * 
 * @param joint_state_msg Subscribed Message
 */
void GantryControl::joint_states_callback(const sensor_msgs::JointState::ConstPtr &joint_state_msg)
{
    if (joint_state_msg->position.size() == 0)
    {
        ROS_ERROR("[gantry_control][joint_states_callback] msg->position.size() == 0!");
    }
    current_joint_states_ = *joint_state_msg;
}

/**
 * @brief Callback to get gantry controller state
 * 
 * @param msg Subscribed Message
 */
void GantryControl::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gantry controller states (throttled to 0.1 Hz):\n" << *msg);
    current_gantry_controller_state_ = *msg;
}

/**
 * @brief Callback to get left arm controller
 * 
 * @param msg Subscribed Message
 */
void GantryControl::left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Left arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_left_arm_controller_state_ = *msg;
}

/**
 * @brief Callback to get right arm controller
 * 
 * @param msg Subscribed Message
 */
void GantryControl::right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Right arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_right_arm_controller_state_ = *msg;
}

/**
 * @brief Send joint to a position
 * 
 * @param command_msg Joint trajectory message
 * @return true 
 * @return false 
 */
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
