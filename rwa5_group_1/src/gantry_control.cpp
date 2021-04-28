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
    dynamic_obstacle_1_subscriber_ = node_.subscribe<geometry_msgs::Point32>("/obstacle_1_pos", 1, boost::bind(&GantryControl::dynamic_obstacle_callback, this, _1, 1));
    dynamic_obstacle_2_subscriber_ = node_.subscribe<geometry_msgs::Point32>("/obstacle_2_pos", 1, boost::bind(&GantryControl::dynamic_obstacle_callback, this, _1, 2));

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

    initializeShelfConfiguration(); // check the location of the free_spaces in the bottom shelfs
    // std::vector<int> shelf_configuration{1,1,1};
    // double safe_1 = -11.3;
    std::array<double, 3> safe{0, 0, 0};
    for (int i = 0; i < 3; i++)
    {
        if (shelf_configuration[i] == 1)
            safe[i] = -11.3;
        else if (shelf_configuration[i] == 0)
            safe[i] = -7.2;
    }
    // joint positions to go to start location
    start_.location = "start";
    start_.gantry = {0, 0, 0};
    start_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    start_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to start_2 location
    start_90_.location = "start_90";
    start_90_.gantry = {0, 0, PI / 2};
    start_90_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    start_90_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // // joint positions to go to middle of bins
    bins_.location = "bins";
    bins_.gantry = {4, 0, 0};
    bins_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bins_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to top left shelf
    shelf1_.location = "shelf_1";
    shelf1_.gantry = {4, -2.4, 0};
    shelf1_.left_arm = aisle_left_arm;
    shelf1_.right_arm = aisle_right_arm;

    // joint positions to go to top right shelf
    shelf2_.location = "shelf_2";
    shelf2_.gantry = {4, 2.4, 0};
    shelf2_.left_arm = aisle_left_arm;
    shelf2_.right_arm = aisle_right_arm;

    // joint positions to go to aisle1 (between shelf 5 and 8)
    aisle1_.location = "aisle1";
    aisle1_.gantry = {0, -1.5, 0.};
    aisle1_.left_arm = aisle_left_arm;
    aisle1_.right_arm = aisle_right_arm;

    // joint positions to go to aisle1 rotated 90(between shelf 5 and 8)
    aisle1_90_.location = "aisle1_90";
    aisle1_90_.gantry = {0, -1.5, PI / 2};
    aisle1_90_.left_arm = start_.left_arm;
    aisle1_90_.right_arm = start_.right_arm;

    // joint positions to go to aisle2 (between shelf 8 and 11)
    aisle2_.location = "aisle2";
    aisle2_.gantry = {0, 1.5, 0.};
    aisle2_.left_arm = aisle_left_arm;
    aisle2_.right_arm = aisle_right_arm;
    // joint positions to go to aisle2 rotated 90 (between shelf 8 and 11)
    aisle2_90_.location = "aisle2_90";
    aisle2_90_.gantry = {0, 1.5, PI / 2};
    aisle2_90_.left_arm = start_.left_arm;
    aisle2_90_.right_arm = start_.right_arm;

    // joint positions to go to tray1
    tray1_left_negative_.location = "tray1";
    tray1_left_negative_.gantry = {0, -5.5, PI / 4};
    tray1_left_negative_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, PI / 4};
    tray1_left_negative_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, PI / 4};

    tray1_left_positive_.location = "tray1";
    tray1_left_positive_.gantry = {0, -5.5, PI / 4 + PI / 2};
    tray1_left_positive_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, PI / 4 + PI / 2};
    tray1_left_positive_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, PI / 4 + PI / 2};

    tray1_right_negative_.location = "tray1";
    tray1_right_negative_.gantry = {0, -5.5, -PI / 4 - PI / 2};
    tray1_right_negative_.left_arm = start_.left_arm;
    tray1_right_negative_.right_arm = start_.right_arm;

    tray1_right_positive_.location = "tray1";
    tray1_right_positive_.gantry = {0, -5.5, -PI / 4};
    tray1_right_positive_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, -PI / 4};
    tray1_right_positive_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, -PI / 4};

    // joint positions to go to tray2
    tray2_left_positive_.location = "tray2";
    tray2_left_positive_.gantry = {0, 5.5, -PI / 4};
    tray2_left_positive_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, -PI / 4};
    tray2_left_positive_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, -PI / 4};

    tray2_left_negative_.location = "tray2";
    tray2_left_negative_.gantry = {0, 5.5, -PI / 4 - PI / 2};
    tray2_left_negative_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, -PI / 4 - PI / 2};
    tray2_left_negative_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, -PI / 4 - PI / 2};

    tray2_right_positive_.location = "tray2";
    tray2_right_positive_.gantry = {0, 5.5, PI / 4 + PI / 2};
    tray2_right_positive_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, PI / 4 + PI / 2};
    tray2_right_positive_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, PI / 4 + PI / 2};

    tray2_right_negative_.location = "tray2";
    tray2_right_negative_.gantry = {0, 5.5, PI / 4};
    tray2_right_negative_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, PI / 4};
    tray2_right_negative_.right_arm = {PI, -PI / 4, PI / 2, -PI / 4, PI / 2, PI / 4};

    // Reach Rail 1 from start
    rail_1_.location = "rail_1";
    rail_1_.gantry = {0., -6.8, 0.};
    rail_1_.left_arm = aisle_left_arm;
    rail_1_.right_arm = aisle_right_arm;

    // joint positions to go to agv1
    agv1_.location = "agv1";
    agv1_.gantry = {0, -5.5, 0};
    agv1_.left_arm = start_.left_arm;
    agv1_.right_arm = start_.right_arm;

    // Reach Rail 1 from start with torso rotated 90
    agv1_90_.location = "agv1_90";
    agv1_90_.gantry = {0., -5.5, PI / 2};
    agv1_90_.left_arm = start_.left_arm;
    agv1_90_.right_arm = start_.right_arm;

    // Reach Rail 2 from start
    rail_2_.location = "rail_2";
    rail_2_.gantry = {0., 6.8, 0.};
    rail_2_.left_arm = aisle_left_arm;
    rail_2_.right_arm = aisle_right_arm;

    // joint positions to go to agv2  // joint positions to go to agv2
    agv2_.location = "agv2";
    agv2_.gantry = {0, 5.5, 0};
    agv2_.left_arm = start_.left_arm;
    agv2_.right_arm = start_.right_arm;

    // Reach Rail 2 from start with torso rotated 90
    agv2_90_.location = "agv2_90";
    agv2_90_.gantry = {0., 5.5, PI / 2};
    agv2_90_.left_arm = start_.left_arm;
    agv2_90_.right_arm = start_.right_arm;

    // Safe spot on Self Row 1
    safe_spot_1_.location = "safe_spot_1";
    safe_spot_1_.gantry = {safe[0], -3, -PI / 2};
    safe_spot_1_.left_arm = aisle_left_arm;
    safe_spot_1_.right_arm = aisle_right_arm;

    // Left approach to Safe spot 1
    ssi1_.location = "ssi1";
    ssi1_.gantry = {safe_spot_1_.gantry[0], -6.8, 0};
    ssi1_.left_arm = aisle_left_arm;
    ssi1_.right_arm = aisle_right_arm;

    // Right approach to Safe spot 1
    ssi2_.location = "ssi2";
    ssi2_.gantry = {safe_spot_1_.gantry[0], -1.5, 0};
    ssi2_.left_arm = aisle_left_arm;
    ssi2_.right_arm = aisle_right_arm;

    // Safe spot on shelf row 3
    safe_spot_3_.location = "safe_spot_3";
    safe_spot_3_.gantry = {safe[2], 3, PI / 2};
    safe_spot_3_.left_arm = aisle_left_arm;
    safe_spot_3_.right_arm = aisle_right_arm;

    // Left approach to Safe spot 3
    ssi3_.location = "ssi3";
    ssi3_.gantry = {safe_spot_3_.gantry[0], 1.5, 0.};
    ssi3_.left_arm = aisle_left_arm;
    ssi3_.right_arm = aisle_right_arm;

    // Right approach to Safe spot 3
    ssi4_.location = "ssi4";
    ssi4_.gantry = {safe_spot_3_.gantry[0], 6.8, 0.};
    ssi4_.left_arm = aisle_left_arm;
    ssi4_.right_arm = aisle_right_arm;

    // Safe spot on shelf row 2
    safe_spot_2_.location = "safe_spot_2";
    safe_spot_2_.gantry = {safe[1], 0, PI / 2};
    safe_spot_2_.left_arm = aisle_left_arm;
    safe_spot_2_.right_arm = aisle_right_arm;

    // Left approach to Safe spot 2
    ssi5_.location = "ssi5";
    ssi5_.gantry = {safe_spot_2_.gantry[0], -1.5, 0.};
    ssi5_.left_arm = aisle_left_arm;
    ssi5_.right_arm = aisle_right_arm;

    // Right approach to Safe spot 2
    ssi6_.location = "ssi6";
    ssi6_.gantry = {safe_spot_2_.gantry[0], 1.5, 0.};
    ssi6_.left_arm = aisle_left_arm;
    ssi6_.right_arm = aisle_right_arm;

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

    ROS_WARN_STREAM("SHELF 5 CONFIG: " << shelf_configuration.at(0));
    ROS_WARN_STREAM("SHELF 8 CONFIG: " << shelf_configuration.at(1));
    ROS_WARN_STREAM("SHELF 11 CONFIG: " << shelf_configuration.at(2));

    // Move robot to init position
    ROS_INFO("[GantryControl::init] Init position ready)...");
}

/**
 * @brief initialize the current configuration of the bottom shelves
 * 
 */
void GantryControl::initializeShelfConfiguration()
{
    //Get transforms world to shelfs
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Duration timeout(5.0);

    geometry_msgs::TransformStamped transformStamped;

    for (int i = 3; i < 12; i++)
    {
        try
        {
            transformStamped = tfBuffer.lookupTransform("world", "shelf" + std::to_string(i) + "_frame",
                                                        ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        //Initialize attribute that stores the frame transforms to world of each camera
        shelf_w_transforms_.at(i - 3) = transformStamped;
    }

    double d3_4 = abs(shelf_w_transforms_.at(0).transform.translation.x - shelf_w_transforms_.at(1).transform.translation.x);
    double d4_5 = abs(shelf_w_transforms_.at(1).transform.translation.x - shelf_w_transforms_.at(2).transform.translation.x);

    if (d4_5 > d3_4)
    {
        shelf_configuration.at(0) += 1;
    }

    double d6_7 = abs(shelf_w_transforms_.at(3).transform.translation.x - shelf_w_transforms_.at(4).transform.translation.x);
    double d7_8 = abs(shelf_w_transforms_.at(4).transform.translation.x - shelf_w_transforms_.at(5).transform.translation.x);

    if (d7_8 > d6_7)
    {
        shelf_configuration.at(1) += 1;
    }

    double d9_10 = abs(shelf_w_transforms_.at(6).transform.translation.x - shelf_w_transforms_.at(7).transform.translation.x);
    double d10_11 = abs(shelf_w_transforms_.at(7).transform.translation.x - shelf_w_transforms_.at(8).transform.translation.x);

    if (d10_11 > d9_10)
    {
        shelf_configuration.at(2) += 1;
    }
}

/**
 * @brief Dynamic obstacle callback
 * 
 * @param msg Subscribed message
 * @param obstacle_n Obstacle ID
 */
void GantryControl::dynamic_obstacle_callback(const geometry_msgs::Point32::ConstPtr &msg, int obstacle_n)
{

    if (obstacle_n == 1)
    {
        obstacle_1_pos.at(0) = msg->x;
        obstacle_1_pos.at(1) = msg->y;
        obstacle_1_pos.at(2) = msg->z;
        if ((obstacle_1_pos.at(1) + 5.0) < 0.00001)
        {
            obstacle_1_pos.at(3) = 1;
        }
        else if ((obstacle_1_pos.at(1) + 1.57) < 0.00001)
        {
            obstacle_1_pos.at(3) = 2;
        }
        else if ((obstacle_1_pos.at(1) - 1.57) < 0.00001)
        {
            obstacle_1_pos.at(3) = 3;
        }
        else if ((obstacle_1_pos.at(1) - 5.00) < 0.00001)
        {
            obstacle_1_pos.at(3) = 4;
        }
    }
    else if (obstacle_n == 2)
    {
        obstacle_2_pos.at(0) = msg->x;
        obstacle_2_pos.at(1) = msg->y;
        obstacle_2_pos.at(2) = msg->z;
        if ((obstacle_2_pos.at(1) + 5.0) < 0.00001)
        {
            obstacle_2_pos.at(3) = 1;
        }
        else if ((obstacle_2_pos.at(1) + 1.57) < 0.00001)
        {
            obstacle_2_pos.at(3) = 2;
        }
        else if ((obstacle_2_pos.at(1) - 1.57) < 0.00001)
        {
            obstacle_2_pos.at(3) = 3;
        }
        else if ((obstacle_2_pos.at(1) - 5.00) < 0.00001)
        {
            obstacle_2_pos.at(3) = 4;
        }
    }
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
 * @brief Pick part from shelf 5 with or without obstacles
 * 
 * @param product Product to be picked
 */
void GantryControl::pickPartFromShelf5Aisle1(product product)
{

    std::string free_arm = checkFreeGripper();
    PresetLocation offset;
    offset.location = "part_offset";
    offset.gantry = {product.p.pose.position.x, rail_1_.gantry[1], rail_1_.gantry[2]};
    offset.left_arm = rail_1_.left_arm;
    offset.right_arm = rail_1_.right_arm;
    goToPresetLocation(offset);

    goToBottomShelfs();
    if (free_arm == "any" || free_arm == "left")
    {
        FKLeftArm({0, -2.13, 1.49, -2.48, -1.57, 0});
    }
    else if (free_arm == "right")
    {
        FKRightArm({PI, -2.13, 1.49, -2.48, -1.57, 0});
    }

    if (obstacle_1_pos[3] != 1 && obstacle_2_pos[3] != 1)
    {
        if (free_arm == "any" || free_arm == "left")
        {
            reachPartShelfLeftArm(product.p);
            pickPartLeftArm(product.p);
        }
        else
        {
            reachPartShelfRightArm(product.p);
            pickPartRightArm(product.p);
        }
        goToPresetLocation(offset);
        goToPresetLocation(ssi1_);
    }
    else if (obstacle_1_pos[3] == 1 || obstacle_2_pos[3] == 1)
    {
        if (obstacle_1_pos[3] == 1)
        {
            while (((obstacle_1_pos[0] <= product.p.pose.position.x) || (obstacle_1_pos[2] == -1)))
            {
                ROS_INFO_STREAM("Wait for Obstacle 1");
            }
            if (free_arm == "any" || free_arm == "left")
            {
                reachPartShelfLeftArm(product.p);
                pickPartLeftArm(product.p);
            }
            else
            {
                reachPartShelfRightArm(product.p);
                pickPartRightArm(product.p);
            }
            goToPresetLocation(offset);
            goToPresetLocation(ssi1_);
        }
        else if (obstacle_2_pos[3] == 1)
        {
            while (((obstacle_2_pos[0] <= product.p.pose.position.x) || (obstacle_2_pos[2] == -1)))
            {
                ROS_INFO_STREAM("Wait for Obstacle 2");
            }
            if (free_arm == "any" || free_arm == "left")
            {
                reachPartShelfLeftArm(product.p);
                pickPartLeftArm(product.p);
            }
            else
            {
                reachPartShelfRightArm(product.p);
                pickPartRightArm(product.p);
            }
            goToPresetLocation(offset);
            goToPresetLocation(ssi1_);
        }
    }
}

/**
 * @brief Pick part from shelf 8 or shelf 5 without obstacles using Aisle2
 * 
 * @param product Product to be picked
 */
void GantryControl::pickPartFromShelfAisle2(product product)
{
    goToPresetLocation(aisle1_);
    std::string free_arm = checkFreeGripper();
    PresetLocation offset;
    offset.location = "part_offset";
    offset.gantry = {product.p.pose.position.x, aisle1_.gantry[1], aisle1_.gantry[2]};
    offset.left_arm = aisle1_.left_arm;
    offset.right_arm = aisle1_.right_arm;
    goToPresetLocation(offset);
    goToBottomShelfs();
    if (free_arm == "any" || free_arm == "left")
    {
        FKLeftArm({0, -2.13, 1.49, -2.48, -1.57, 0});
        reachPartShelfLeftArm(product.p);
        pickPartLeftArm(product.p);
        retriveFromBottomShelf();
        goToPresetLocation(offset);
    }
    else
    {
        FKRightArm({PI, -2.13, 1.49, -2.48, -1.57, 0});
        reachPartShelfRightArm(product.p);
        pickPartRightArm(product.p);
        retriveFromBottomShelf();
        goToPresetLocation(offset);
    }
    goToPresetLocation(aisle1_);
    goToPresetLocation(aisle1_90_);
}

/**
 * @brief Pick part from shelf 8 or shelf 5 without obstacles using Aisle3
 * 
 * @param product Product to be picked
 */
void GantryControl::pickPartFromShelfAisle3(product product)
{
    goToPresetLocation(aisle2_);
    std::string free_arm = checkFreeGripper();
    PresetLocation offset;
    offset.location = "part_offset";
    offset.gantry = {product.p.pose.position.x, aisle2_.gantry[1], aisle2_.gantry[2]};
    offset.left_arm = aisle2_.left_arm;
    offset.right_arm = aisle2_.right_arm;
    goToPresetLocation(offset);

    goToBottomShelfs();

    if (free_arm == "any" || free_arm == "left")
    {
        FKLeftArm({0, -2.13, 1.49, -2.48, -1.57, 0});
        reachPartShelfLeftArm(product.p);
        pickPartLeftArm(product.p);
        retriveFromBottomShelf();
        goToPresetLocation(offset);
    }
    else
    {
        FKRightArm({PI, -2.13, 1.49, -2.48, -1.57, 0});
        reachPartShelfRightArm(product.p);
        pickPartRightArm(product.p);
        retriveFromBottomShelf();
        goToPresetLocation(offset);
    }
    goToPresetLocation(aisle2_);
    goToPresetLocation(aisle2_90_);
}

/**
 * @brief Pick part from shelf 11 with or without obstacles
 * 
 * @param product Product to be picked
 */
void GantryControl::pickPartFromShelf11Aisle4(product product)
{
    std::string free_arm = checkFreeGripper();
    PresetLocation offset;
    offset.location = "part_offset";
    offset.gantry = {product.p.pose.position.x, rail_2_.gantry[1], rail_2_.gantry[2]};
    offset.left_arm = rail_2_.left_arm;
    offset.right_arm = rail_2_.right_arm;
    goToPresetLocation(offset);
    goToBottomShelfs();
    if (free_arm == "any" || free_arm == "left")
    {
        FKLeftArm({0, -2.13, 1.49, -2.48, -1.57, 0});
    }
    else if (free_arm == "right")
    {
        FKRightArm({PI, -2.13, 1.49, -2.48, -1.57, 0});
    }

    if (obstacle_1_pos[3] != 4 && obstacle_2_pos[3] != 4)
    {
        if (free_arm == "any" || free_arm == "left")
        {
            reachPartShelfLeftArm(product.p);
            pickPartLeftArm(product.p);
        }
        else
        {
            reachPartShelfRightArm(product.p);
            pickPartRightArm(product.p);
        }
        goToPresetLocation(offset);
        goToPresetLocation(ssi4_);
    }
    else if (obstacle_1_pos[3] == 4 || obstacle_2_pos[3] == 4)
    {
        if (obstacle_1_pos[3] == 4)
        {
            while (((obstacle_1_pos[0] <= product.p.pose.position.x) || (obstacle_1_pos[2] == -1)))
            // while(true)
            {
                ROS_INFO_STREAM("Wait for Obstacle 1");
            }
            if (free_arm == "any" || free_arm == "left")
            {
                reachPartShelfLeftArm(product.p);
                pickPartLeftArm(product.p);
            }
            else
            {
                reachPartShelfRightArm(product.p);
                pickPartRightArm(product.p);
            }
            goToPresetLocation(offset);
            goToPresetLocation(ssi4_);
        }
        else if (obstacle_2_pos[3] == 4)
        {
            while (((obstacle_2_pos[0] <= product.p.pose.position.x) || (obstacle_2_pos[2] == -1)))
            {
                ROS_INFO_STREAM("Wait for Obstacle 2");
            }
            if (free_arm == "any" || free_arm == "left")
            {
                reachPartShelfLeftArm(product.p);
                pickPartLeftArm(product.p);
            }
            else
            {
                reachPartShelfRightArm(product.p);
                pickPartRightArm(product.p);
            }
            goToPresetLocation(offset);
            goToPresetLocation(ssi4_);
        }
    }
}

/**
 * @brief Pick part from shelf 8 with obstacles when safe location only available in shelf row 2 
 * 
 * @param product Product to be picked
 */
void GantryControl::pickPartFromShelf8Aisle2ObstaclesSafe1(product product)
{
    // Wait for Obstace in Aisle 2
    if (obstacle_1_pos[3] == 2)
    {
        ROS_INFO_STREAM("Wait for Obstacle 1");
        while (((obstacle_1_pos[0] >= ssi5_.gantry[0]) || (obstacle_1_pos[2] == 1)))
        {
        }
    }
    else if (obstacle_2_pos[3] == 2)
    {
        ROS_INFO_STREAM("Wait for Obstacle 2 :");
        while (((obstacle_2_pos[0] >= ssi5_.gantry[0]) || (obstacle_2_pos[2] == 1)))
        {
        }
    }

    goToPresetLocation(ssi5_);
    goToPresetLocation(safe_spot_2_);

    std::string free_arm = checkFreeGripper();
    if (free_arm == "any" || free_arm == "left")
    {
        PresetLocation tmp = safe_spot_2_;
        if (obstacle_1_pos[3] == 2)
        {
            tmp.location = "tmp_locs";
            tmp.right_arm = {3.14, -PI, 2.89, -1.57, 0, 0};

            ROS_INFO_STREAM("Wait for Obstacle 1");
            while (((obstacle_1_pos[0] <= ssi5_.gantry[0]) || (obstacle_1_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        else if (obstacle_2_pos[3] == 2)
        {
            tmp.location = "tmp_locs";
            tmp.right_arm = {3.14, -PI, 2.89, -1.57, 0, 0};
            ROS_INFO_STREAM("Wait for Obstacle 2 :");
            while (((obstacle_2_pos[0] <= ssi5_.gantry[0]) || (obstacle_2_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        // // !--------Faster implmentation of reachShelfLeft() -------!
        PresetLocation tmp1 = ssi5_;
        tmp1.location = "tmp_locs";
        tmp1.gantry[2] = PI;
        tmp1.right_arm = {3.14, -PI, 2.89, -1.57, 0, 0};
        goToPresetLocation(tmp1);

        PresetLocation tmp2 = tmp1;
        tmp2.gantry = {product.p.pose.position.x, tmp2.gantry[1], tmp2.gantry[2] + PI / 2};
        tmp2.left_arm = {0, -2.13, 1.49, -2.48, -1.57, 0};
        goToPresetLocation(tmp2);

        geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;
        const double dx = product.p.pose.position.x - currentArmPose.position.x;
        const double dy = product.p.pose.position.y - currentArmPose.position.y;
        ROS_INFO_STREAM("Offset X,Y: " << dx << "," << dy);

        PresetLocation tmp3 = tmp2;
        tmp3.gantry = {tmp2.gantry[0] + dx, tmp2.gantry[1] - dy, tmp2.gantry[2]};
        goToPresetLocation(tmp3);

        // !--------Faster implmentation of pickPartLeftArm() -------!
        activateGripper("left_arm");
        geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

        tf2::Quaternion q_arm(currentPose.orientation.x,
                              currentPose.orientation.y,
                              currentPose.orientation.z,
                              currentPose.orientation.w);

        tf2::Quaternion q_world_part(product.p.pose.orientation.x,
                                     product.p.pose.orientation.y,
                                     product.p.pose.orientation.z,
                                     product.p.pose.orientation.w);

        product.p.pose.position.z = product.p.pose.position.z + model_height.at(product.p.type) + GRIPPER_HEIGHT - EPSILON;
        product.p.pose.orientation.x = currentPose.orientation.x;
        product.p.pose.orientation.y = currentPose.orientation.y;
        product.p.pose.orientation.z = currentPose.orientation.z;
        product.p.pose.orientation.w = currentPose.orientation.w;

        tf2::Quaternion q_world_left_ee_link(currentPose.orientation.x,
                                             currentPose.orientation.y,
                                             currentPose.orientation.z,
                                             currentPose.orientation.w);

        q_left_ee_link_part = q_world_left_ee_link.inverse() * q_world_part;
        q_left_ee_link_part.normalize();

        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        left_arm_group_.setPoseTarget(product.p.pose);
        left_arm_group_.move();
        auto state = getGripperState("left_arm");
        if (state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object attached");
            goToPresetLocation(tmp3);
            // goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);
        }

        else if (!state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            // Retreive to Safe Location
            goToPresetLocation(tmp3);
            goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);

            int max_attempts{5};
            int current_attempt{0};
            //--try to pick up the part 5 times
            while (current_attempt < max_attempts)
            {
                // Wait for Obstacles in Aisle 2
                if (obstacle_1_pos[3] == 2)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 1");
                    while (((obstacle_1_pos[0] <= ssi5_.gantry[0]) || (obstacle_1_pos[2] == -1)))
                    {
                    }
                }
                else if (obstacle_2_pos[3] == 2)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 2 :");
                    while (((obstacle_2_pos[0] <= ssi5_.gantry[0]) || (obstacle_2_pos[2] == -1)))
                    {
                    }
                }

                // Go to pick The Part again
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp2);
                goToPresetLocation(tmp3);
                activateGripper("left_arm");
                left_arm_group_.setPoseTarget(product.p.pose);
                left_arm_group_.move();

                // Retreive to Safe Location
                goToPresetLocation(tmp3);
                // goToPresetLocation(tmp2);
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp);

                auto state = getGripperState("left_arm");
                if (state.attached)
                {
                    ROS_INFO_STREAM("[Gripper] = object attached");
                    break;
                }
                current_attempt++;
            }
            product.p.picked_status = true;
        }
    }

    else if (free_arm == "right")
    {
        PresetLocation tmp = safe_spot_2_;
        if (obstacle_1_pos[3] == 2)
        {
            tmp.location = "tmp_locs";
            tmp.left_arm = {0, -PI, 2.89, -1.57, 0, 0};
            ROS_INFO_STREAM("Wait for Obstacle 1");
            while (((obstacle_1_pos[0] <= ssi5_.gantry[0]) || (obstacle_1_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        else if (obstacle_2_pos[3] == 2)
        {
            tmp.location = "tmp_locs";
            tmp.left_arm = {0, -PI, 2.89, -1.57, 0, 0};
            ROS_INFO_STREAM("Wait for Obstacle 2 :");
            while (((obstacle_2_pos[0] <= ssi5_.gantry[0]) || (obstacle_2_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        // // !--------Faster implmentation of reachShelfRight() -------!
        PresetLocation tmp1 = ssi5_;
        tmp1.location = "tmp_locs";
        tmp1.gantry[2] = 0;
        tmp1.left_arm = {0, -PI, 2.89, -1.57, 0, 0};
        goToPresetLocation(tmp1);

        PresetLocation tmp2 = tmp1;
        tmp2.gantry = {product.p.pose.position.x, tmp2.gantry[1], tmp2.gantry[2] + PI / 2};
        tmp2.right_arm = {PI, -2.13, 1.49, -2.48, -1.57, 0};
        goToPresetLocation(tmp2);

        geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;
        const double dx = product.p.pose.position.x - currentArmPose.position.x;
        const double dy = product.p.pose.position.y - currentArmPose.position.y;
        ROS_INFO_STREAM("Offset X,Y: " << dx << "," << dy);

        PresetLocation tmp3 = tmp2;
        tmp3.gantry = {tmp2.gantry[0] + dx, tmp2.gantry[1] - dy, tmp2.gantry[2]};
        goToPresetLocation(tmp3);

        // !--------Faster implmentation of pickPartLeftArm() -------!
        activateGripper("right_arm");
        geometry_msgs::Pose currentPose = right_arm_group_.getCurrentPose().pose;

        tf2::Quaternion q_arm(currentPose.orientation.x,
                              currentPose.orientation.y,
                              currentPose.orientation.z,
                              currentPose.orientation.w);

        tf2::Quaternion q_world_part(product.p.pose.orientation.x,
                                     product.p.pose.orientation.y,
                                     product.p.pose.orientation.z,
                                     product.p.pose.orientation.w);

        product.p.pose.position.z = product.p.pose.position.z + model_height.at(product.p.type) + GRIPPER_HEIGHT - EPSILON;
        product.p.pose.orientation.x = currentPose.orientation.x;
        product.p.pose.orientation.y = currentPose.orientation.y;
        product.p.pose.orientation.z = currentPose.orientation.z;
        product.p.pose.orientation.w = currentPose.orientation.w;

        tf2::Quaternion q_world_right_ee_link(currentPose.orientation.x,
                                              currentPose.orientation.y,
                                              currentPose.orientation.z,
                                              currentPose.orientation.w);

        q_right_ee_link_part = q_world_right_ee_link.inverse() * q_world_part;
        q_right_ee_link_part.normalize();

        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        right_arm_group_.setPoseTarget(product.p.pose);
        right_arm_group_.move();
        auto state = getGripperState("right_arm");
        if (state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object attached");
            goToPresetLocation(tmp3);
            goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);
        }

        else if (!state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            // Retreive to Safe Location
            goToPresetLocation(tmp3);
            goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);

            int max_attempts{5};
            int current_attempt{0};
            //--try to pick up the part 5 times
            while (current_attempt < max_attempts)
            {
                // Wait for Obstacles in Aisle 2
                if (obstacle_1_pos[3] == 2)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 1");
                    while (((obstacle_1_pos[0] <= ssi5_.gantry[0]) || (obstacle_1_pos[2] == -1)))
                    {
                    }
                }
                else if (obstacle_2_pos[3] == 2)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 2 :");
                    while (((obstacle_2_pos[0] <= ssi5_.gantry[0]) || (obstacle_2_pos[2] == -1)))
                    {
                    }
                }

                // Go to pick The Part again
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp2);
                goToPresetLocation(tmp3);
                activateGripper("right_arm");
                right_arm_group_.setPoseTarget(product.p.pose);
                right_arm_group_.move();

                // Retreive to Safe Location
                goToPresetLocation(tmp3);
                // goToPresetLocation(tmp2);
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp);

                auto state = getGripperState("right_arm");
                if (state.attached)
                {
                    ROS_INFO_STREAM("[Gripper] = object attached");
                    break;
                }
                current_attempt++;
            }
            product.p.picked_status = true;
        }
    }

    // Wait for Obstacles in Aisle 2
    if (obstacle_1_pos[3] == 2)
    {
        ROS_INFO_STREAM("Wait for Obstacle 1");
        while (((obstacle_1_pos[0] >= ssi5_.gantry[0]) || (obstacle_1_pos[2] == 1)))
        {
        }
    }
    else if (obstacle_2_pos[3] == 2)
    {
        ROS_INFO_STREAM("Wait for Obstacle 2 :");
        while (((obstacle_2_pos[0] >= ssi5_.gantry[0]) || (obstacle_2_pos[2] == 1)))
        {
        }
    }
    goToPresetLocation(ssi5_);
    goToPresetLocation(aisle1_);
}

/**
 * @brief Pick part from shelf 8 with obstacles when safe location only available in shelf row 1 
 * 
 * @param product Product to be picked
 */
void GantryControl::pickPartFromShelf8Aisle2Obstacles(product product)
{
    // Wait for Obstace in Aisle 2
    if (obstacle_1_pos[3] == 2)
    {
        ROS_INFO_STREAM("Wait for Obstacle 1");
        while (((obstacle_1_pos[0] >= ssi2_.gantry[0]) || (obstacle_1_pos[2] == 1)))
        {
        }
    }
    else if (obstacle_2_pos[3] == 2)
    {
        ROS_INFO_STREAM("Wait for Obstacle 2 :");
        while (((obstacle_2_pos[0] >= ssi2_.gantry[0]) || (obstacle_2_pos[2] == 1)))
        {
        }
    }

    goToPresetLocation(ssi2_);
    goToPresetLocation(safe_spot_1_);

    std::string free_arm = checkFreeGripper();
    if (free_arm == "any" || free_arm == "left")
    {
        PresetLocation tmp = safe_spot_1_;
        if (obstacle_1_pos[3] == 2)
        {
            tmp.location = "tmp_locs";
            tmp.right_arm = {3.14, -PI, 2.89, -1.57, 0, 0};
            ROS_INFO_STREAM("Wait for Obstacle 1");
            while (((obstacle_1_pos[0] <= ssi2_.gantry[0]) || (obstacle_1_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        else if (obstacle_2_pos[3] == 2)
        {
            tmp.location = "tmp_locs";
            tmp.right_arm = {3.14, -PI, 2.89, -1.57, 0, 0};
            goToPresetLocation(tmp);
            ROS_INFO_STREAM("Wait for Obstacle 2 :");
            while (((obstacle_2_pos[0] <= ssi2_.gantry[0]) || (obstacle_2_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        // // !--------Faster implmentation of reachShelfLeft() -------!
        PresetLocation tmp1 = ssi2_;
        tmp1.location = "tmp_locs";
        tmp1.gantry[2] = -PI;
        tmp1.right_arm = {3.14, -PI, 2.89, -1.57, 0, 0};
        goToPresetLocation(tmp1);

        PresetLocation tmp2 = tmp1;
        tmp2.gantry = {product.p.pose.position.x, tmp2.gantry[1], tmp2.gantry[2] + PI / 2};
        tmp2.left_arm = {0, -2.13, 1.49, -2.48, -1.57, 0};
        goToPresetLocation(tmp2);

        geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;
        const double dx = product.p.pose.position.x - currentArmPose.position.x;
        const double dy = product.p.pose.position.y - currentArmPose.position.y;
        ROS_INFO_STREAM("Offset X,Y: " << dx << "," << dy);

        PresetLocation tmp3 = tmp2;
        tmp3.gantry = {tmp2.gantry[0] + dx, tmp2.gantry[1] - dy, tmp2.gantry[2]};
        goToPresetLocation(tmp3);

        // !--------Faster implmentation of pickPartLeftArm() -------!
        activateGripper("left_arm");
        geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

        tf2::Quaternion q_arm(currentPose.orientation.x,
                              currentPose.orientation.y,
                              currentPose.orientation.z,
                              currentPose.orientation.w);

        tf2::Quaternion q_world_part(product.p.pose.orientation.x,
                                     product.p.pose.orientation.y,
                                     product.p.pose.orientation.z,
                                     product.p.pose.orientation.w);

        product.p.pose.position.z = product.p.pose.position.z + model_height.at(product.p.type) + GRIPPER_HEIGHT - EPSILON;
        product.p.pose.orientation.x = currentPose.orientation.x;
        product.p.pose.orientation.y = currentPose.orientation.y;
        product.p.pose.orientation.z = currentPose.orientation.z;
        product.p.pose.orientation.w = currentPose.orientation.w;

        tf2::Quaternion q_world_left_ee_link(currentPose.orientation.x,
                                             currentPose.orientation.y,
                                             currentPose.orientation.z,
                                             currentPose.orientation.w);

        q_left_ee_link_part = q_world_left_ee_link.inverse() * q_world_part;
        q_left_ee_link_part.normalize();

        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        left_arm_group_.setPoseTarget(product.p.pose);
        left_arm_group_.move();
        auto state = getGripperState("left_arm");
        if (state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object attached");
            goToPresetLocation(tmp3);
            goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);
        }

        else if (!state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            // Retreive to Safe Location
            goToPresetLocation(tmp3);
            goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);

            int max_attempts{5};
            int current_attempt{0};
            //--try to pick up the part 5 times
            while (current_attempt < max_attempts)
            {
                // Wait for Obstacles in Aisle 2
                if (obstacle_1_pos[3] == 2)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 1");
                    while (((obstacle_1_pos[0] <= ssi2_.gantry[0]) || (obstacle_1_pos[2] == -1)))
                    {
                    }
                }
                else if (obstacle_2_pos[3] == 2)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 2 :");
                    while (((obstacle_2_pos[0] <= ssi2_.gantry[0]) || (obstacle_2_pos[2] == -1)))
                    {
                    }
                }

                // Go to pick The Part again
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp2);
                goToPresetLocation(tmp3);
                activateGripper("left_arm");
                left_arm_group_.setPoseTarget(product.p.pose);
                left_arm_group_.move();

                // Retreive to Safe Location
                goToPresetLocation(tmp3);
                goToPresetLocation(tmp2);
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp);

                auto state = getGripperState("left_arm");
                if (state.attached)
                {
                    ROS_INFO_STREAM("[Gripper] = object attached");
                    break;
                }
                current_attempt++;
            }
            product.p.picked_status = true;
        }
    }

    else if (free_arm == "right")
    {
        PresetLocation tmp = safe_spot_1_;
        if (obstacle_1_pos[3] == 2)
        {
            tmp.location = "tmp_locs";
            tmp.left_arm = {0, -PI, 2.89, -1.57, 0, 0};
            ROS_INFO_STREAM("Wait for Obstacle 1");
            while (((obstacle_1_pos[0] <= ssi2_.gantry[0]) || (obstacle_1_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        else if (obstacle_2_pos[3] == 2)
        {
            tmp.location = "tmp_locs";
            tmp.left_arm = {0, -PI, 2.89, -1.57, 0, 0};
            ROS_INFO_STREAM("Wait for Obstacle 2 :");
            while (((obstacle_2_pos[0] <= ssi2_.gantry[0]) || (obstacle_2_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        // // !--------Faster implmentation of reachShelfRight() -------!
        PresetLocation tmp1 = ssi2_;
        tmp1.location = "tmp_locs";
        tmp1.gantry[2] = 0;
        tmp1.left_arm = {0, -PI, 2.89, -1.57, 0, 0};
        goToPresetLocation(tmp1);

        PresetLocation tmp2 = tmp1;
        tmp2.gantry = {product.p.pose.position.x, tmp2.gantry[1], tmp2.gantry[2] + PI / 2};
        tmp2.right_arm = {PI, -2.13, 1.49, -2.48, -1.57, 0};
        goToPresetLocation(tmp2);

        geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;
        const double dx = product.p.pose.position.x - currentArmPose.position.x;
        const double dy = product.p.pose.position.y - currentArmPose.position.y;
        ROS_INFO_STREAM("Offset X,Y: " << dx << "," << dy);

        PresetLocation tmp3 = tmp2;
        tmp3.gantry = {tmp2.gantry[0] + dx, tmp2.gantry[1] - dy, tmp2.gantry[2]};
        goToPresetLocation(tmp3);

        // !--------Faster implmentation of pickPartLeftArm() -------!
        activateGripper("right_arm");
        geometry_msgs::Pose currentPose = right_arm_group_.getCurrentPose().pose;

        tf2::Quaternion q_arm(currentPose.orientation.x,
                              currentPose.orientation.y,
                              currentPose.orientation.z,
                              currentPose.orientation.w);

        tf2::Quaternion q_world_part(product.p.pose.orientation.x,
                                     product.p.pose.orientation.y,
                                     product.p.pose.orientation.z,
                                     product.p.pose.orientation.w);

        product.p.pose.position.z = product.p.pose.position.z + model_height.at(product.p.type) + GRIPPER_HEIGHT - EPSILON;
        product.p.pose.orientation.x = currentPose.orientation.x;
        product.p.pose.orientation.y = currentPose.orientation.y;
        product.p.pose.orientation.z = currentPose.orientation.z;
        product.p.pose.orientation.w = currentPose.orientation.w;

        tf2::Quaternion q_world_right_ee_link(currentPose.orientation.x,
                                              currentPose.orientation.y,
                                              currentPose.orientation.z,
                                              currentPose.orientation.w);

        q_right_ee_link_part = q_world_right_ee_link.inverse() * q_world_part;
        q_right_ee_link_part.normalize();

        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        right_arm_group_.setPoseTarget(product.p.pose);
        right_arm_group_.move();
        auto state = getGripperState("right_arm");
        if (state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object attached");
            goToPresetLocation(tmp3);
            goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);
        }

        else if (!state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            // Retreive to Safe Location
            goToPresetLocation(tmp3);
            goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);

            int max_attempts{5};
            int current_attempt{0};
            //--try to pick up the part 5 times
            while (current_attempt < max_attempts)
            {
                // Wait for Obstacles in Aisle 2
                if (obstacle_1_pos[3] == 2)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 1");
                    while (((obstacle_1_pos[0] <= ssi2_.gantry[0]) || (obstacle_1_pos[2] == -1)))
                    {
                    }
                }
                else if (obstacle_2_pos[3] == 2)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 2 :");
                    while (((obstacle_2_pos[0] <= ssi2_.gantry[0]) || (obstacle_2_pos[2] == -1)))
                    {
                    }
                }

                // Go to pick The Part again
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp2);
                goToPresetLocation(tmp3);
                activateGripper("right_arm");
                right_arm_group_.setPoseTarget(product.p.pose);
                right_arm_group_.move();

                // Retreive to Safe Location
                goToPresetLocation(tmp3);
                // goToPresetLocation(tmp2);
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp);

                auto state = getGripperState("right_arm");
                if (state.attached)
                {
                    ROS_INFO_STREAM("[Gripper] = object attached");
                    break;
                }
                current_attempt++;
            }
            product.p.picked_status = true;
        }
    }

    // Wait for Obstacles in Aisle 2
    if (obstacle_1_pos[3] == 2)
    {
        ROS_INFO_STREAM("Wait for Obstacle 1");
        while (((obstacle_1_pos[0] >= ssi2_.gantry[0]) || (obstacle_1_pos[2] == 1)))
        {
        }
    }
    else if (obstacle_2_pos[3] == 2)
    {
        ROS_INFO_STREAM("Wait for Obstacle 2 :");
        while (((obstacle_2_pos[0] >= ssi2_.gantry[0]) || (obstacle_2_pos[2] == 1)))
        {
        }
    }
    goToPresetLocation(ssi2_);
    goToPresetLocation(aisle1_);
}

/**
 * @brief Pick part from shelf 8 with obstacles when safe location only available in shelf row 3 
 * 
 * @param product Product to be picked
 */
void GantryControl::pickPartFromShelf8Aisle3Obstacles(product product)
{
    if (obstacle_1_pos[3] == 3)
    {
        ROS_INFO_STREAM("Wait for Obstacle 1");
        while (((obstacle_1_pos[0] >= ssi3_.gantry[0]) || (obstacle_1_pos[2] == 1)))
        {
        }
    }
    else if (obstacle_2_pos[3] == 3)
    {
        // while (true)
        ROS_INFO_STREAM("Wait for Obstacle 2 :");
        while (((obstacle_2_pos[0] >= ssi3_.gantry[0]) || (obstacle_2_pos[2] == 1)))
        {
        }
    }

    goToPresetLocation(ssi3_);
    goToPresetLocation(safe_spot_3_);

    std::string free_arm = checkFreeGripper();
    if (free_arm == "any" || free_arm == "left")
    // if (free_arm == "left")
    {
        PresetLocation tmp = safe_spot_3_;
        if (obstacle_1_pos[3] == 3)
        {
            tmp.location = "tmp_locs";
            tmp.right_arm = {3.14, -PI, 2.89, -1.57, 0, 0};
            ROS_INFO_STREAM("Wait for Obstacle 1");
            while (((obstacle_1_pos[0] <= ssi3_.gantry[0]) || (obstacle_1_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        else if (obstacle_2_pos[3] == 3)
        {
            tmp.location = "tmp_locs";
            tmp.right_arm = {3.14, -PI, 2.89, -1.57, 0, 0};
            ROS_INFO_STREAM("Wait for Obstacle 2 :");
            while (((obstacle_2_pos[0] <= ssi3_.gantry[0]) || (obstacle_2_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        // // !--------Faster implmentation of reachShelfLeft() -------!
        PresetLocation tmp1 = ssi3_;
        tmp1.location = "tmp_locs";
        tmp1.gantry[2] = PI;
        tmp1.right_arm = {3.14, -PI, 2.89, -1.57, 0, 0};
        goToPresetLocation(tmp1);

        PresetLocation tmp2 = tmp1;
        tmp2.gantry = {product.p.pose.position.x, tmp2.gantry[1], tmp2.gantry[2] - PI / 2};
        tmp2.left_arm = {0, -2.13, 1.49, -2.48, -1.57, 0};
        goToPresetLocation(tmp2);

        geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;
        const double dx = product.p.pose.position.x - currentArmPose.position.x;
        const double dy = product.p.pose.position.y - currentArmPose.position.y;
        ROS_INFO_STREAM("Offset X,Y: " << dx << "," << dy);

        PresetLocation tmp3 = tmp2;
        tmp3.gantry = {tmp2.gantry[0] + dx, tmp2.gantry[1] - dy, tmp2.gantry[2]};
        goToPresetLocation(tmp3);

        // !--------Faster implmentation of pickPartLeftArm() -------!
        activateGripper("left_arm");
        geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

        tf2::Quaternion q_arm(currentPose.orientation.x,
                              currentPose.orientation.y,
                              currentPose.orientation.z,
                              currentPose.orientation.w);

        tf2::Quaternion q_world_part(product.p.pose.orientation.x,
                                     product.p.pose.orientation.y,
                                     product.p.pose.orientation.z,
                                     product.p.pose.orientation.w);

        product.p.pose.position.z = product.p.pose.position.z + model_height.at(product.p.type) + GRIPPER_HEIGHT - EPSILON;
        product.p.pose.orientation.x = currentPose.orientation.x;
        product.p.pose.orientation.y = currentPose.orientation.y;
        product.p.pose.orientation.z = currentPose.orientation.z;
        product.p.pose.orientation.w = currentPose.orientation.w;

        tf2::Quaternion q_world_left_ee_link(currentPose.orientation.x,
                                             currentPose.orientation.y,
                                             currentPose.orientation.z,
                                             currentPose.orientation.w);

        q_left_ee_link_part = q_world_left_ee_link.inverse() * q_world_part;
        q_left_ee_link_part.normalize();

        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        left_arm_group_.setPoseTarget(product.p.pose);
        left_arm_group_.move();
        auto state = getGripperState("left_arm");
        if (state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object attached");
            goToPresetLocation(tmp3);
            // goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);
        }

        else if (!state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            // Retreive to Safe Location
            goToPresetLocation(tmp3);
            // goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);

            int max_attempts{5};
            int current_attempt{0};
            //--try to pick up the part 5 times
            while (current_attempt < max_attempts)
            {
                // Wait for Obstacles in Aisle 3
                if (obstacle_1_pos[3] == 3)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 1");
                    while (((obstacle_1_pos[0] <= ssi3_.gantry[0]) || (obstacle_1_pos[2] == -1)))
                    {
                    }
                }
                else if (obstacle_2_pos[3] == 3)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 2 :");
                    while (((obstacle_2_pos[0] <= ssi3_.gantry[0]) || (obstacle_2_pos[2] == -1)))
                    {
                    }
                }

                // Go to pick The Part again
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp2);
                goToPresetLocation(tmp3);
                activateGripper("left_arm");
                left_arm_group_.setPoseTarget(product.p.pose);
                left_arm_group_.move();

                // Retreive to Safe Location
                goToPresetLocation(tmp3);
                // goToPresetLocation(tmp2);
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp);

                auto state = getGripperState("left_arm");
                if (state.attached)
                {
                    ROS_INFO_STREAM("[Gripper] = object attached");
                    break;
                }
                current_attempt++;
            }
            product.p.picked_status = true;
        }
    }

    else if (free_arm == "right")
    {
        PresetLocation tmp = safe_spot_3_;
        if (obstacle_1_pos[3] == 3)
        {
            tmp.location = "tmp_locs";
            tmp.left_arm = {0, -PI, 2.89, -1.57, 0, 0};
            ROS_INFO_STREAM("Wait for Obstacle 1");
            while (((obstacle_1_pos[0] <= ssi3_.gantry[0]) || (obstacle_1_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        else if (obstacle_2_pos[3] == 3)
        {
            tmp.location = "tmp_locs";
            tmp.left_arm = {0, -PI, 2.89, -1.57, 0, 0};
            ROS_INFO_STREAM("Wait for Obstacle 2 :");
            while (((obstacle_2_pos[0] <= ssi3_.gantry[0]) || (obstacle_2_pos[2] == -1)))
            {
                goToPresetLocation(tmp);
            }
        }
        // // !--------Faster implmentation of reachShelfRight() -------!
        PresetLocation tmp1 = ssi3_;
        tmp1.location = "tmp_locs";
        tmp1.gantry[2] = 0;
        tmp1.left_arm = {0, -PI, 2.89, -1.57, 0, 0};
        goToPresetLocation(tmp1);

        PresetLocation tmp2 = tmp1;
        tmp2.gantry = {product.p.pose.position.x, tmp2.gantry[1], tmp2.gantry[2] - PI / 2};
        tmp2.right_arm = {PI, -2.13, 1.49, -2.48, -1.57, 0};
        goToPresetLocation(tmp2);

        geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;
        const double dx = product.p.pose.position.x - currentArmPose.position.x;
        const double dy = product.p.pose.position.y - currentArmPose.position.y;
        ROS_INFO_STREAM("Offset X,Y: " << dx << "," << dy);

        PresetLocation tmp3 = tmp2;
        tmp3.gantry = {tmp2.gantry[0] + dx, tmp2.gantry[1] - dy, tmp2.gantry[2]};
        goToPresetLocation(tmp3);

        // !--------Faster implmentation of pickPartLeftArm() -------!
        activateGripper("right_arm");
        geometry_msgs::Pose currentPose = right_arm_group_.getCurrentPose().pose;

        tf2::Quaternion q_arm(currentPose.orientation.x,
                              currentPose.orientation.y,
                              currentPose.orientation.z,
                              currentPose.orientation.w);

        tf2::Quaternion q_world_part(product.p.pose.orientation.x,
                                     product.p.pose.orientation.y,
                                     product.p.pose.orientation.z,
                                     product.p.pose.orientation.w);

        product.p.pose.position.z = product.p.pose.position.z + model_height.at(product.p.type) + GRIPPER_HEIGHT - EPSILON;
        product.p.pose.orientation.x = currentPose.orientation.x;
        product.p.pose.orientation.y = currentPose.orientation.y;
        product.p.pose.orientation.z = currentPose.orientation.z;
        product.p.pose.orientation.w = currentPose.orientation.w;

        tf2::Quaternion q_world_right_ee_link(currentPose.orientation.x,
                                              currentPose.orientation.y,
                                              currentPose.orientation.z,
                                              currentPose.orientation.w);

        q_right_ee_link_part = q_world_right_ee_link.inverse() * q_world_part;
        q_right_ee_link_part.normalize();

        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        right_arm_group_.setPoseTarget(product.p.pose);
        right_arm_group_.move();
        auto state = getGripperState("right_arm");
        if (state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object attached");
            goToPresetLocation(tmp3);
            // goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);
        }

        else if (!state.attached)
        {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            // Retreive to Safe Location
            goToPresetLocation(tmp3);
            // goToPresetLocation(tmp2);
            goToPresetLocation(tmp1);
            goToPresetLocation(tmp);

            int max_attempts{5};
            int current_attempt{0};
            //--try to pick up the part 5 times
            while (current_attempt < max_attempts)
            {
                // Wait for Obstacles in Aisle 2
                if (obstacle_1_pos[3] == 3)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 1");
                    while (((obstacle_1_pos[0] <= ssi3_.gantry[0]) || (obstacle_1_pos[2] == -1)))
                    {
                    }
                }
                else if (obstacle_2_pos[3] == 3)
                {
                    ROS_INFO_STREAM("Wait for Obstacle 2 :");
                    while (((obstacle_2_pos[0] <= ssi3_.gantry[0]) || (obstacle_2_pos[2] == -1)))
                    {
                    }
                }

                // Go to pick The Part again
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp2);
                goToPresetLocation(tmp3);
                activateGripper("right_arm");
                right_arm_group_.setPoseTarget(product.p.pose);
                right_arm_group_.move();

                // Retreive to Safe Location
                goToPresetLocation(tmp3);
                // goToPresetLocation(tmp2);
                goToPresetLocation(tmp1);
                goToPresetLocation(tmp);

                auto state = getGripperState("right_arm");
                if (state.attached)
                {
                    ROS_INFO_STREAM("[Gripper] = object attached");
                    break;
                }
                current_attempt++;
            }
            product.p.picked_status = true;
        }
    }

    // Wait for Obstacles in Aisle 3
    if (obstacle_1_pos[3] == 3)
    {
        ROS_INFO_STREAM("Wait for Obstacle 1");
        while (((obstacle_1_pos[0] >= ssi3_.gantry[0]) || (obstacle_1_pos[2] == 1)))
        {
        }
    }
    else if (obstacle_2_pos[3] == 3)
    {
        ROS_INFO_STREAM("Wait for Obstacle 2 :");
        while (((obstacle_2_pos[0] >= ssi3_.gantry[0]) || (obstacle_2_pos[2] == 1)))
        {
        }
    }
    goToPresetLocation(ssi3_);
    goToPresetLocation(aisle2_);
}



/**
 * @brief Function to pick the part from the environment
 * 
 * @param product Product to be picked
 */
void GantryControl::getProduct(product product)
{
    std::string location = product.p.location;
    std::string free_arm = checkFreeGripper();

    if (location == "shelf_5")
    {
        if (gantry_location_ == "safe_spot_3")
        {
            if (obstacle_1_pos[3] != 4 && obstacle_2_pos[3] != 4)
            {
                goToPresetLocation(ssi4_);
            }
            else if (obstacle_1_pos[3] == 4 || obstacle_2_pos[3] == 4)
            {
                if (obstacle_1_pos[3] == 4)
                {
                    while (((obstacle_1_pos[0] <= ssi4_.gantry[0]) || (obstacle_1_pos[2] == -1)))
                    {
                        ROS_INFO_STREAM("Wait for Obstacle 1");
                    }
                    goToPresetLocation(ssi4_);
                }
                else if (obstacle_2_pos[3] == 4)
                {
                    while (((obstacle_2_pos[0] <= ssi4_.gantry[0]) || (obstacle_2_pos[2] == -1)))
                    {
                        ROS_INFO_STREAM("Wait for Obstacle 2");
                    }
                    goToPresetLocation(ssi4_);
                }
            }
        }
        if (gantry_location_ == "ssi4")
        {
            goToPresetLocation(rail_2_);
        }
        if (gantry_location_ == "rail_2")
        {
            goToPresetLocation(agv2_90_);
        }
        if (gantry_location_ == "agv2" || gantry_location_ == "agv2_90")
        {
            goToPresetLocation(agv2_90_);
            goToPresetLocation(aisle2_90_);
        }
        if (gantry_location_ == "aisle2" || gantry_location_ == "aisle2_90")
        {
            goToPresetLocation(aisle2_90_);
            goToPresetLocation(start_90_);
        }
        if (gantry_location_ == "bins" || gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "start" || gantry_location_ == "start_90")
        {
            goToPresetLocation(start_90_);
            goToPresetLocation(aisle1_90_);
        }
        if (gantry_location_ == "aisle1" || gantry_location_ == "aisle1_90")
        {
            if (obstacle_1_pos[3] != 2 && obstacle_2_pos[3] != 2)
            {
                pickPartFromShelfAisle2(product);
                //add product to arm
                if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
                {
                    product_left_arm_ = product;
                }
                else
                {
                    product_right_arm_ = product;
                }
                return;
            }
            else
            {
                goToPresetLocation(agv1_90_);
            }
        }
        if (gantry_location_ == "agv1" || gantry_location_ == "agv1_90")
        {
            goToPresetLocation(agv1_90_);
            goToPresetLocation(rail_1_);
            pickPartFromShelf5Aisle1(product);
            if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
            {
                product_left_arm_ = product;
            }
            else
            {
                product_right_arm_ = product;
            }
            return;
        }
        if (gantry_location_ == "safe_spot_1")
        {
            if (obstacle_1_pos[3] != 1 && obstacle_2_pos[3] != 1)
            {
                goToPresetLocation(ssi1_);
            }
            else if (obstacle_1_pos[3] == 1 || obstacle_2_pos[3] == 1)
            {
                if (obstacle_1_pos[3] == 1)
                {
                    while (((obstacle_1_pos[0] <= ssi1_.gantry[0]) || (obstacle_1_pos[2] == -1)))
                    {
                        ROS_INFO_STREAM("Wait for Obstacle 1");
                    }
                    goToPresetLocation(ssi1_);
                }
                else if (obstacle_2_pos[3] == 1)
                {
                    while (((obstacle_2_pos[0] <= ssi1_.gantry[0]) || (obstacle_2_pos[2] == -1)))
                    {
                        ROS_INFO_STREAM("Wait for Obstacle 2");
                    }
                    goToPresetLocation(ssi1_);
                }
            }
        }
        if (gantry_location_ == "ssi1")
        {
            pickPartFromShelf5Aisle1(product);
            //add product to arm
            if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
            {
                product_left_arm_ = product;
            }
            else
            {
                product_right_arm_ = product;
            }
            return;
        }
    }

    else if (location == "shelf_11")
    {
        if (gantry_location_ == "safe_spot_1")
        {
            if (obstacle_1_pos[3] != 1 && obstacle_2_pos[3] != 1)
            {
                goToPresetLocation(ssi1_);
            }
            else if (obstacle_1_pos[3] == 1 || obstacle_2_pos[3] == 1)
            {
                if (obstacle_1_pos[3] == 1)
                {
                    while (((obstacle_1_pos[0] <= ssi1_.gantry[0]) || (obstacle_1_pos[2] == -1)))
                    {
                        ROS_INFO_STREAM("Wait for Obstacle 1");
                    }
                    goToPresetLocation(ssi1_);
                }
                else if (obstacle_2_pos[3] == 4)
                {
                    while (((obstacle_2_pos[0] <= ssi1_.gantry[0]) || (obstacle_2_pos[2] == -1)))
                    {
                        ROS_INFO_STREAM("Wait for Obstacle 2");
                    }
                    goToPresetLocation(ssi1_);
                }
            }
        }
        if (gantry_location_ == "ssi1")
        {
            goToPresetLocation(rail_1_);
        }
        if (gantry_location_ == "rail_1")
        {
            goToPresetLocation(agv1_90_);
        }
        if (gantry_location_ == "agv1" || gantry_location_ == "agv1_90")
        {
            goToPresetLocation(agv1_90_);
            goToPresetLocation(aisle1_90_);
        }
        if (gantry_location_ == "aisle1" || gantry_location_ == "aisle1_90")
        {
            goToPresetLocation(aisle1_90_);
            goToPresetLocation(start_90_);
        }
        if (gantry_location_ == "bins" || gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "start" || gantry_location_ == "start_90")
        {
            goToPresetLocation(start_90_);
            goToPresetLocation(aisle2_90_);
        }
        if (gantry_location_ == "aisle2" || gantry_location_ == "aisle2_90")
        {
            if (obstacle_1_pos[3] != 3 && obstacle_2_pos[3] != 3)
            {
                pickPartFromShelfAisle3(product);
                //add product to arm
                if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
                {
                    product_left_arm_ = product;
                }
                else
                {
                    product_right_arm_ = product;
                }
                return;
            }
            else
            {
                goToPresetLocation(agv2_90_);
            }
        }
        if (gantry_location_ == "agv2" || gantry_location_ == "agv2_90")
        {
            goToPresetLocation(agv2_90_);
            goToPresetLocation(rail_2_);
            pickPartFromShelf11Aisle4(product);
            //add product to arm
            if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
            {
                product_left_arm_ = product;
            }
            else
            {
                product_right_arm_ = product;
            }
            return;
        }
        if (gantry_location_ == "safe_spot_3")
        {
            if (obstacle_1_pos[3] != 4 && obstacle_2_pos[3] != 4)
            {
                goToPresetLocation(ssi4_);
            }
            else if (obstacle_1_pos[3] == 4 || obstacle_2_pos[3] == 4)
            {
                if (obstacle_1_pos[3] == 4)
                {
                    while (((obstacle_1_pos[0] <= ssi4_.gantry[0]) || (obstacle_1_pos[2] == -1)))
                    {
                        ROS_INFO_STREAM("Wait for Obstacle 1");
                    }
                    goToPresetLocation(ssi4_);
                }
                else if (obstacle_2_pos[3] == 4)
                {
                    while (((obstacle_2_pos[0] <= ssi4_.gantry[0]) || (obstacle_2_pos[2] == -1)))
                    {
                        ROS_INFO_STREAM("Wait for Obstacle 2");
                    }
                    goToPresetLocation(ssi4_);
                }
            }
        }
        if (gantry_location_ == "ssi4")
        {
            pickPartFromShelf11Aisle4(product);
            return;
        }
    }

    else if (location == "shelf_8")
    {
        if (gantry_location_ == "ssi1")
        {
            goToPresetLocation(rail_1_);
        }
        if (gantry_location_ == "rail_1")
        {
            goToPresetLocation(agv1_90_);
        }
        if (gantry_location_ == "agv1_90" || gantry_location_ == "agv1")
        {
            goToPresetLocation(aisle1_90_);
        }
        if (gantry_location_ == "aisle1_90" || gantry_location_ == "aisle1")
        {
            if (obstacle_1_pos[3] != 2 && obstacle_2_pos[3] != 2)
            {
                goToPresetLocation(aisle1_);
                pickPartFromShelfAisle2(product);
                //add product to arm
                if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
                {
                    product_left_arm_ = product;
                }
                else
                {
                    product_right_arm_ = product;
                }
                return;
            }
            else
            {
                goToPresetLocation(start_90_);
            }
        }
        if (gantry_location_ == "ssi4")
        {
            goToPresetLocation(rail_2_);
        }
        if (gantry_location_ == "rail_2")
        {
            goToPresetLocation(agv2_90_);
        }
        if (gantry_location_ == "agv2_90" || gantry_location_ == "agv2")
        {
            goToPresetLocation(aisle2_90_);
        }
        if (gantry_location_ == "aisle2_90" || gantry_location_ == "aisle2")
        {
            if (obstacle_1_pos[3] != 3 && obstacle_2_pos[3] != 3)
            {
                goToPresetLocation(aisle2_);
                pickPartFromShelfAisle3(product);
                //add product to arm
                if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
                {
                    product_left_arm_ = product;
                }
                else
                {
                    product_right_arm_ = product;
                }
                return;
            }
            else
            {
                goToPresetLocation(start_90_);
            }
        }
        if (gantry_location_ == "bins" || gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "start" || gantry_location_ == "start_90")
        {
            if (obstacle_1_pos[3] != 2 && obstacle_2_pos[3] != 2)
            {
                goToPresetLocation(aisle1_);
                pickPartFromShelfAisle2(product);
                //add product to arm
                if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
                {
                    product_left_arm_ = product;
                }
                else
                {
                    product_right_arm_ = product;
                }
                return;
            }
            else if (obstacle_1_pos[3] != 3 && obstacle_2_pos[3] != 3)
            {
                goToPresetLocation(aisle2_);
                pickPartFromShelfAisle3(product);
                //add product to arm
                if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
                {
                    product_left_arm_ = product;
                }
                else
                {
                    product_right_arm_ = product;
                }
                return;
            }
            else if (shelf_configuration[0] == 1)
            {
                goToPresetLocation(aisle1_);
                pickPartFromShelf8Aisle2Obstacles(product);
                //add product to arm
                if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
                {
                    product_left_arm_ = product;
                }
                else
                {
                    product_right_arm_ = product;
                }
                return;
            }
            else if (shelf_configuration[2] == 1)
            {
                goToPresetLocation(aisle2_);
                pickPartFromShelf8Aisle3Obstacles(product);
                //add product to arm
                if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
                {
                    product_left_arm_ = product;
                }
                else
                {
                    product_right_arm_ = product;
                }
                return;
            }
            else if (shelf_configuration[1] == 1)
            {
                goToPresetLocation(aisle1_);
                pickPartFromShelf8Aisle2ObstaclesSafe1(product);
                //add product to arm
                if (free_arm.compare("any") == 0 || free_arm.compare("left") == 0)
                {
                    product_left_arm_ = product;
                }
                else
                {
                    product_right_arm_ = product;
                }
                return;
            }
        }
    }

    else if (location == "shelf_1" || location == "shelf_2" || location == "bins")
    {
        if (gantry_location_ == "ssi1")
        {
            goToPresetLocation(rail_1_);
        }
        if (gantry_location_ == "rail_1")
        {
            goToPresetLocation(agv1_90_);
        }
        if (gantry_location_ == "agv1_90" || gantry_location_ == "agv1")
        {
            goToPresetLocation(aisle1_90_);
        }
        if (gantry_location_ == "aisle1_90" || gantry_location_ == "aisle1")
        {
            goToPresetLocation(start_90_);
        }
        if (gantry_location_ == "ssi4")
        {
            goToPresetLocation(rail_2_);
        }
        if (gantry_location_ == "rail_2")
        {
            goToPresetLocation(agv2_90_);
        }
        if (gantry_location_ == "agv2_90" || gantry_location_ == "agv2")
        {
            goToPresetLocation(aisle2_90_);
        }
        if (gantry_location_ == "aisle2_90" || gantry_location_ == "aisle2")
        {
            goToPresetLocation(start_90_);
        }
        if (gantry_location_ == "bins" || gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "start" || gantry_location_ == "start_90")
        {
            if (location == "shelf_1")
            {
                goToPresetLocation(shelf1_);
                if (free_arm == "any" || free_arm == "left")
                {
                    reachPartShelfLeftArm(product.p);
                    FKLeftArm({0, -2.13, 1.49, -2.48, -1.57, 0});
                    pickPartLeftArm(product.p);
                    FKLeftArm(shelf1_.left_arm);
                    goToPresetLocation(shelf1_);
                }
                else
                {
                    reachPartShelfRightArm(product.p);
                    FKRightArm({PI, -2.13, 1.49, -2.48, -1.57, 0});
                    pickPartRightArm(product.p);
                    FKRightArm(shelf1_.right_arm);
                    goToPresetLocation(shelf1_);
                }
            }
            else if (location == "shelf_2")
            {
                goToPresetLocation(shelf2_);
                if (free_arm == "any" || free_arm == "left")
                {
                    reachPartShelfLeftArm(product.p);
                    FKLeftArm({0, -2.13, 1.49, -2.48, -1.57, 0});
                    pickPartLeftArm(product.p);
                    FKLeftArm(shelf2_.left_arm);
                    goToPresetLocation(shelf2_);
                }
                else
                {
                    reachPartShelfRightArm(product.p);
                    FKRightArm({PI, -2.13, 1.49, -2.48, -1.57, 0});
                    pickPartRightArm(product.p);
                    FKRightArm(shelf2_.right_arm);
                    goToPresetLocation(shelf2_);
                }
            }
            else if (location == "bins")
            {
                goToPresetLocation(bins_);
                if (free_arm == "any" || free_arm == "left")
                {
                    goToPresetLocation(bins_);
                    reachPartBinLeftArm(product.p);
                    gantry_location_ = "bins";
                    ros::Duration(0.5).sleep();
                    pickPartLeftArm(product.p);
                    ros::Duration(1).sleep();
                }
                else
                {
                    goToPresetLocation(bins_);
                    gantry_location_ = "bins";
                    reachPartBinRightArm(product.p);
                    ros::Duration(0.5).sleep();
                    pickPartRightArm(product.p);
                    ros::Duration(1).sleep();
                }
                goToPresetLocation(bins_);
                gantry_location_ = "bins";
            }
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
    tf2::Quaternion q_part_orientation(
        target.orientation.x,
        target.orientation.y,
        target.orientation.z,
        target.orientation.w);
    double part_roll, part_pitch, part_yaw;
    tf2::Matrix3x3(q_part_orientation).getRPY(part_roll, part_pitch, part_yaw);
    ROS_INFO_STREAM("[pickPart] Part pose in tray frame (rpy): "
                    << "[" << part_roll
                    << " " << part_pitch
                    << " " << part_yaw << "]");

    tf2::Quaternion adjusted_yaw;
    // multiply yaw by -1
    // Create this quaternion from roll/pitch/yaw (in radians)
    adjusted_yaw.setRPY(part_roll, part_pitch, part_yaw);
    adjusted_yaw.normalize();

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
    transformStamped.transform.rotation.x = adjusted_yaw.x();
    transformStamped.transform.rotation.y = adjusted_yaw.y();
    transformStamped.transform.rotation.z = adjusted_yaw.z();
    transformStamped.transform.rotation.w = adjusted_yaw.w();

    for (int i{0}; i < 15; ++i)
        br.sendTransform(transformStamped);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // ros::Rate rate(10);
    ros::Duration timeout(5.0);

    geometry_msgs::TransformStamped world_target_tf;
    geometry_msgs::TransformStamped ee_left_target_tf;
    geometry_msgs::TransformStamped ee_right_target_tf;
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

    /**
 *ee_target_tf and  world_target_tf are expressed in the same frame: "world"
 We want to find the relative rotation, q_r, to go from ee_target_tf to world_target_tf
 q_r = world_target_tf*ee_target_tf_inverse
 * 
 */

    tf2::Quaternion part_in_tray(world_target_tf.transform.rotation.x,
                                 world_target_tf.transform.rotation.y,
                                 world_target_tf.transform.rotation.z,
                                 world_target_tf.transform.rotation.w);

    geometry_msgs::Pose world_target{target};
    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = world_target_tf.transform.rotation.x;
    world_target.orientation.y = world_target_tf.transform.rotation.y;
    world_target.orientation.z = world_target_tf.transform.rotation.z;
    world_target.orientation.w = world_target_tf.transform.rotation.w;

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
    if (part.bin_location == "top")
    {
        rotateTorso(-PI);
    }
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

    if (part.bin_location == "bottom")
    {
        rotateTorso(-PI);
    }
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
        offset_y = 1.55 - dy;

        rotateTorso(L_LEFT_ARM);
    }
    else
    {
        offset_y = -1.55 - dy;
        rotateTorso(R_LEFT_ARM);
    }
    // FKLeftArm({0, -2.13, 1.49, -2.48, -1.57, 0});
    geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;

    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;

    // full_robot_group_.setJointValueTarget(joint_group_positions_);

    // moveit::planning_interface::MoveGroupInterface::Plan move_x;
    // bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success)
    //     full_robot_group_.move();

    joint_group_positions_.at(1) += offset_y;
    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_y;
    bool success = (full_robot_group_.plan(move_y) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
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
        offset_y = 1.55 - dy;

        rotateTorso(L_RIGHT_ARM);
        //--left arm
    }
    else
    {
        offset_y = -1.55 - dy;
        rotateTorso(R_RIGHT_ARM);
        //--left arm
    }
    // FKRightArm({PI, -2.13, 1.49, -2.48, -1.57, 0});
    geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;

    tf2::Quaternion q(currentArmPose.orientation.x,
                      currentArmPose.orientation.y,
                      currentArmPose.orientation.z,
                      currentArmPose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("GRIPPER ROLL ANGLE: " << roll);

    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;

    // full_robot_group_.setJointValueTarget(joint_group_positions_);

    // moveit::planning_interface::MoveGroupInterface::Plan move_x;
    // bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success)
    //     full_robot_group_.move();

    joint_group_positions_.at(1) += offset_y;
    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_y;
    bool success = (full_robot_group_.plan(move_y) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
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

    tf2::Quaternion q_arm(currentPose.orientation.x,
                          currentPose.orientation.y,
                          currentPose.orientation.z,
                          currentPose.orientation.w);

    tf2::Quaternion q_world_part(part.pose.orientation.x,
                                 part.pose.orientation.y,
                                 part.pose.orientation.z,
                                 part.pose.orientation.w);

    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON + 0.007;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;

    tf2::Quaternion q_world_left_ee_link(currentPose.orientation.x,
                                         currentPose.orientation.y,
                                         currentPose.orientation.z,
                                         currentPose.orientation.w);

    q_left_ee_link_part = q_world_left_ee_link.inverse() * q_world_part;

    q_left_ee_link_part.normalize();

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
                // ros::Duration(0.5).sleep();
                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();

                activateGripper("left_arm");
                // ros::Duration(0.5).sleep();
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

    tf2::Quaternion q_arm(currentPose.orientation.x,
                          currentPose.orientation.y,
                          currentPose.orientation.z,
                          currentPose.orientation.w);

    tf2::Quaternion q_world_part(part.pose.orientation.x,
                                 part.pose.orientation.y,
                                 part.pose.orientation.z,
                                 part.pose.orientation.w);

    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON + 0.007;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;

    tf2::Quaternion q_world_right_ee_link(currentPose.orientation.x,
                                          currentPose.orientation.y,
                                          currentPose.orientation.z,
                                          currentPose.orientation.w);

    q_right_ee_link_part = q_world_right_ee_link.inverse() * q_world_part;

    q_right_ee_link_part.normalize();

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
                // ros::Duration(0.5).sleep();
                right_arm_group_.setPoseTarget(part.pose);
                right_arm_group_.move();

                activateGripper("right_arm");
                // ros::Duration(0.5).sleep();
                current_attempt++;
            }
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
 * @brief A method to correct the pose of the part in the tray using left arm
 * 
 * @param part Part whose pose has to be adjusted 
 * @param target Target Pose to which the part is to be moved
 * @param agv_id AGV ID on which the part is present
 * @return true 
 * @return false 
 */

bool GantryControl::correctPosePartLeftArm(Part part, geometry_msgs::Pose target, std::string agv_id)
{
    if (agv_id.compare("agv2") == 0 || agv_id.compare("any") == 0)
    {
        // goToPresetLocation(agv2_);
        if (target.position.x >= 0)
        {
            goToPresetLocation(tray2_left_positive_);
            ros::Duration(0.5).sleep();
            ROS_WARN_STREAM("IN_TRAY POSITIONING");
        }
        else
        {
            goToPresetLocation(tray2_left_negative_);
            ros::Duration(0.5).sleep();
        }
    }
    else
    {
        // goToPresetLocation(agv1_);
        if (-target.position.x >= 0)
        {
            goToPresetLocation(tray1_left_positive_);
            ros::Duration(0.5).sleep();
        }
        else
        {
            goToPresetLocation(tray1_left_negative_);
            ros::Duration(0.5).sleep();
        }
    }

    geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;

    tf::Quaternion q1(part.pose.orientation.x, part.pose.orientation.y, part.pose.orientation.z, part.pose.orientation.w);
    tf::Quaternion q2(target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);

    tf::Matrix3x3 m1(q1);
    tf::Matrix3x3 m2(q2);

    double r1, r2, p1, p2, y1, y2;
    m1.getRPY(r1, p1, y1);
    m2.getRPY(r2, p2, y2);

    float o_xDiff = (r2 - r1);
    float o_yDiff = (p2 - p1);
    float o_zDiff = (y2 - y1);

    float p_xDiff = (target.position.x - currentArmPose.position.x);
    float p_yDiff = (target.position.y - currentArmPose.position.y);

    // ROS_INFO_STREAM("Current Arm Pose :"<< r1 <<","<<p1<<","<<y1);
    // ROS_INFO_STREAM("Target Pose :"<< r2 <<","<<p2<<","<<y2);
    // ROS_INFO_STREAM("Move arm (x,y,theta)"<<p_xDiff<<","<<p_yDiff<<","<<o_zDiff*(180/3.14));

    joint_group_positions_.at(0) += p_xDiff;
    joint_group_positions_.at(1) -= p_yDiff;
    joint_group_positions_.at(8) -= o_zDiff;
    // joint_group_positions_.at(8) += 1.57;
    full_robot_group_.setJointValueTarget(joint_group_positions_);
    full_robot_group_.move();
    ros::Duration(1).sleep();

    currentArmPose = left_arm_group_.getCurrentPose().pose;
    currentArmPose.position.z -= 0.2;
    left_arm_group_.setPoseTarget(currentArmPose);
    left_arm_group_.move();
    ros::Duration(1).sleep();

    deactivateGripper("left_arm");
}

/**
 * @brief A method to pick parts from the tray using left arm
 * 
 * @param part Part tobe picked
 * @param agv_id AGV ID of the part
 * @return true 
 * @return false 
 */
bool GantryControl::pickPartFromTrayLeftArm(part part, std::string agv_id)
{
    if (agv_id.compare("agv2") == 0 || agv_id.compare("any") == 0)
    {
        if (part.pose.position.x >= 0)
        {
            goToPresetLocation(tray2_left_positive_);
            ros::Duration(0.5).sleep();
        }
        else
        {
            goToPresetLocation(tray2_left_negative_);
            ros::Duration(0.5).sleep();
        }
    }
    else
    {
        // goToPresetLocation(agv1_);
        if (-part.pose.position.x >= 0)
        {
            goToPresetLocation(tray1_left_positive_);
            ros::Duration(0.5).sleep();
        }
        else
        {
            goToPresetLocation(tray1_left_negative_);
            ros::Duration(0.5).sleep();
        }
    }

    ROS_INFO_STREAM("[" << part.type << "]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);
    geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;

    const double offset_y = part.pose.position.y - currentArmPose.position.y;
    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;
    joint_group_positions_.at(1) -= offset_y;

    full_robot_group_.setJointValueTarget(joint_group_positions_);
    full_robot_group_.move();

    ros::Duration(0.5).sleep();
    part.pose.position.z = 0.73 + 0.021;
    pickPartLeftArm(part);
    activateGripper("left_arm");
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

    ROS_INFO_STREAM("[Pick " << part.type << " Left Arm]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);
    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;

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
                ros::Duration(1).sleep();
                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();

                activateGripper("left_arm");
                ros::Duration(1).sleep();
                current_attempt++;
            }
            part.picked_status = true;

            left_arm_group_.setPoseTarget(currentPose);
            left_arm_group_.move();
        }
    }
    ros::Duration(0.5).sleep();
    std::vector<double> retrieve{joint_group_positions_.at(0) -= offset_x, joint_group_positions_.at(1) += offset_y};
    FKGantry(retrieve);
    ros::Duration(1).sleep();
}

/**
 * @brief A method to correct the pose of the part in the tray
 * 
 * @param part Part whose pose has to be adjusted 
 * @param target Target Pose to which the part is to be moved
 * @param agv_id AGV ID on which the part is present
 * @return true 
 * @return false 
 */
bool GantryControl::correctPosePartRightArm(Part part, geometry_msgs::Pose target, std::string agv_id)
{
    if (agv_id.compare("agv2") == 0 || agv_id.compare("any") == 0)
    {
        // goToPresetLocation(agv2_);
        if (target.position.x >= 0)
        {
            goToPresetLocation(tray2_right_positive_);
            ros::Duration(0.5).sleep();
        }
        else
        {
            goToPresetLocation(tray2_right_negative_);
            ros::Duration(0.5).sleep();
        }
    }
    else
    {
        // goToPresetLocation(agv1_);
        if (-target.position.x >= 0)
        {
            goToPresetLocation(tray1_right_positive_);
            ros::Duration(0.5).sleep();
        }
        else
        {
            goToPresetLocation(tray1_right_negative_);
            ros::Duration(0.5).sleep();
        }
    }

    geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;

    tf::Quaternion q1(part.pose.orientation.x, part.pose.orientation.y, part.pose.orientation.z, part.pose.orientation.w);
    tf::Quaternion q2(target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);

    tf::Matrix3x3 m1(q1);
    tf::Matrix3x3 m2(q2);

    double r1, r2, p1, p2, y1, y2;
    m1.getRPY(r1, p1, y1);
    m2.getRPY(r2, p2, y2);

    float o_xDiff = (r2 - r1);
    float o_yDiff = (p2 - p1);
    float o_zDiff = (y2 - y1);

    float p_xDiff = (target.position.x - currentArmPose.position.x);
    float p_yDiff = (target.position.y - currentArmPose.position.y);

    // ROS_INFO_STREAM("Current Arm Pose :"<< r1 <<","<<p1<<","<<y1);
    // ROS_INFO_STREAM("Target Pose :"<< r2 <<","<<p2<<","<<y2);
    // ROS_INFO_STREAM("Move arm (x,y,theta)"<<p_xDiff<<","<<p_yDiff<<","<<o_zDiff*(180/3.14));

    joint_group_positions_.at(0) += p_xDiff;
    joint_group_positions_.at(1) -= p_yDiff;
    joint_group_positions_.at(14) -= o_zDiff;
    full_robot_group_.setJointValueTarget(joint_group_positions_);
    full_robot_group_.move();
    ros::Duration(1).sleep();

    currentArmPose = right_arm_group_.getCurrentPose().pose;
    currentArmPose.position.z -= 0.2;
    right_arm_group_.setPoseTarget(currentArmPose);
    right_arm_group_.move();
    ros::Duration(1).sleep();

    deactivateGripper("right_arm");
}

/**
 * @brief A method to pick parts from the tray from right arm
 * 
 * @param part Part tobe picked
 * @param agv_id AGV ID of the part
 * @return true 
 * @return false 
 */
bool GantryControl::pickPartFromTrayRightArm(part part, std::string agv_id)
{
    if (agv_id.compare("agv2") == 0 || agv_id.compare("any") == 0)
    {
        if (part.pose.position.x >= 0)
        {
            goToPresetLocation(tray2_right_positive_);
            ros::Duration(0.5).sleep();
        }
        else
        {
            goToPresetLocation(tray2_right_negative_);
            ros::Duration(0.5).sleep();
        }
    }
    else
    {
        // goToPresetLocation(agv1_);
        if (-part.pose.position.x >= 0)
        {
            goToPresetLocation(tray1_right_positive_);
            ros::Duration(0.5).sleep();
        }
        else
        {
            goToPresetLocation(tray1_right_negative_);
            ros::Duration(0.5).sleep();
        }
    }

    ROS_INFO_STREAM("[" << part.type << "]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);
    geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;

    const double offset_y = part.pose.position.y - currentArmPose.position.y;
    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;
    joint_group_positions_.at(1) -= offset_y;

    full_robot_group_.setJointValueTarget(joint_group_positions_);
    full_robot_group_.move();

    ros::Duration(0.5).sleep();
    part.pose.position.z = 0.73 + 0.021;
    pickPartRightArm(part);
    activateGripper("right_arm");
    geometry_msgs::Pose currentPose = right_arm_group_.getCurrentPose().pose;

    ROS_INFO_STREAM("[Pick " << part.type << " right Arm]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);
    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;

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
                ros::Duration(1).sleep();
                right_arm_group_.setPoseTarget(part.pose);
                right_arm_group_.move();

                activateGripper("right_arm");
                ros::Duration(1).sleep();
                current_attempt++;
            }
            part.picked_status = true;

            right_arm_group_.setPoseTarget(currentPose);
            right_arm_group_.move();
        }
    }
    ros::Duration(0.5).sleep();
    std::vector<double> retrieve{joint_group_positions_.at(0) -= offset_x, joint_group_positions_.at(1) += offset_y};
    FKGantry(retrieve);
    ros::Duration(1).sleep();
}

/**
 * @brief Throw part from left hand if faulty
 * 
 * @param part Part object of the faulty part
 * @param ptype Type of the part obtained from the orders list.
 * @return true 
 * @return false 
 */
bool GantryControl::throwPartLeft(part part)
{

    ROS_WARN_STREAM("Faulty Part: " << part.type);
    ROS_WARN_STREAM("FAULTY POSE Z: " << part.pose.position.z);

    part.pose.position.z += 0.015;
    ROS_WARN_STREAM("PART LOCATION: " << part.location);
    if (part.location == "agv_2")
    {
        goToPresetLocation(agv2_);
        if (part.pose.position.y < -7 && part.pose.position.x > 0){
            goToPresetLocation(tray2_left_negative_);
        }
        else if (part.pose.position.x >= 0)
        {
            goToPresetLocation(tray2_left_positive_);
        }
        else
        {
            goToPresetLocation(tray2_left_negative_);
        }
    }
    else if (part.location == "agv_1" || part.location == "any")
    {
        goToPresetLocation(agv1_);
        ROS_WARN_STREAM("X POSE: " << part.pose.position.x);
        ROS_WARN_STREAM("Y POSE: " << part.pose.position.y);
        if (part.pose.position.y > 7 && part.pose.position.x < 0){
            ROS_WARN_STREAM("CONDITION BEING CALLED");
            goToPresetLocation(tray1_left_negative_);
        }
        else if (-part.pose.position.x >= 0)
        {
            goToPresetLocation(tray1_left_positive_);
        }
        else
        {
            goToPresetLocation(tray1_left_negative_);
        }
    }

    part.pose.orientation.x = 0;
    part.pose.orientation.y = 0.707;
    part.pose.orientation.z = 0;
    part.pose.orientation.w = 0.707;

    geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;

    // // //--TODO: Consider agv1 too

    ROS_WARN_STREAM("TYPE: " << part.type);

    const double offset_y = part.pose.position.y - currentArmPose.position.y;

    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    if (currentArmPose.position.y > 0)
    {
        joint_group_positions_.at(1) -= offset_y - 0.1;
    }
    else
    {
        joint_group_positions_.at(1) -= offset_y + 0.1;
    }

    joint_group_positions_.at(0) += offset_x;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_x;
    bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();

    ros::Duration(1).sleep();
    pickPartLeftArm(part);
    ros::Duration(1).sleep();

    // check if part is in products_to_flip_ an erase it from the vector

    if (products_to_flip_.empty() != 1)
    {
        for (int i = 0; i < products_to_flip_.size(); i++)
        {
            if (part.pose.position.x + 0.1 >= products_to_flip_.at(i).p.pose.position.x &&
                part.pose.position.x - 0.1 <= products_to_flip_.at(i).p.pose.position.x &&
                part.pose.position.y + 0.1 >= products_to_flip_.at(i).p.pose.position.y &&
                part.pose.position.y - 0.1 <= products_to_flip_.at(i).p.pose.position.y)
            {

                products_to_flip_.erase(products_to_flip_.begin() + i);
            }
        }
    }

    auto state = getGripperState("left_arm");
    if (state.attached)
    {
        if (part.location.compare("agv_2") == 0)
        {
            goToPresetLocation(agv2_);
        }
        else
        {
            goToPresetLocation(agv1_);
        }

        deactivateGripper("left_arm");
    }

    if (part.location.compare("agv_2") == 0)
    {
        for (int j = 0; j < products_kit_tray_2_.size(); ++j)
        {
            if (products_kit_tray_2_.at(j).p.id == part.id)
            {
                products_kit_tray_2_.erase(products_kit_tray_2_.begin() + j);
            }
        }
    }
    else
    {
        for (int j = 0; j < products_kit_tray_1_.size(); ++j)
        {
            if (products_kit_tray_1_.at(j).p.id == part.id)
            {
                products_kit_tray_1_.erase(products_kit_tray_1_.begin() + j);
            }
        }
    }
}

/**
 * @brief Throw part from right hand if faulty
 * 
 * @param part Part object of the faulty part
 * @return true 
 * @return false 
 */
bool GantryControl::throwPartRight(part part)
{
    ROS_WARN_STREAM("Faulty Part: " << part.type);
    ROS_WARN_STREAM("FAULTY POSE Z: " << part.pose.position.z);

    part.pose.position.z += 0.015;
    ROS_WARN_STREAM("PART LOCATION: " << part.location);
    if (part.location == "agv_2")
    {
        goToPresetLocation(agv2_);
        if (part.pose.position.x >= 0)
        {
            goToPresetLocation(tray2_right_positive_);
        }
        else
        {
            goToPresetLocation(tray2_right_negative_);
        }
    }
    else if (part.location == "agv_1" || part.location == "any")
    {
        goToPresetLocation(agv1_);
        if (-part.pose.position.x >= 0)
        {
            goToPresetLocation(tray1_right_positive_);
        }
        else
        {
            goToPresetLocation(tray1_right_negative_);
        }
    }

    part.pose.orientation.x = 0;
    part.pose.orientation.y = 0.707;
    part.pose.orientation.z = 0;
    part.pose.orientation.w = 0.707;

    geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;

    // // //--TODO: Consider agv1 too

    ROS_WARN_STREAM("TYPE: " << part.type);

    const double offset_y = part.pose.position.y - currentArmPose.position.y;

    const double offset_x = part.pose.position.x - currentArmPose.position.x;

    if (currentArmPose.position.y > 0)
    {
        joint_group_positions_.at(1) -= offset_y - 0.1;
    }
    else
    {
        joint_group_positions_.at(1) -= offset_y + 0.1;
    }

    joint_group_positions_.at(0) += offset_x;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan move_x;
    bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();

    ros::Duration(1).sleep();
    pickPartLeftArm(part);
    ros::Duration(1).sleep();

    // check if part is in products_to_flip_ an erase it from the vector

    if (products_to_flip_.empty() != 1)
    {
        for (int i = 0; i < products_to_flip_.size(); i++)
        {
            if (part.pose.position.x + 0.1 >= products_to_flip_.at(i).p.pose.position.x &&
                part.pose.position.x - 0.1 <= products_to_flip_.at(i).p.pose.position.x &&
                part.pose.position.y + 0.1 >= products_to_flip_.at(i).p.pose.position.y &&
                part.pose.position.y - 0.1 <= products_to_flip_.at(i).p.pose.position.y)
            {

                products_to_flip_.erase(products_to_flip_.begin() + i);
            }
        }
    }

    auto state = getGripperState("right_arm");
    if (state.attached)
    {
        if (part.location.compare("agv_2") == 0)
        {
            goToPresetLocation(agv2_);
        }
        else
        {
            goToPresetLocation(agv1_);
        }

        deactivateGripper("right_arm");
    }

    if (part.location.compare("agv_2") == 0)
    {
        for (int j = 0; j < products_kit_tray_2_.size(); ++j)
        {
            if (products_kit_tray_2_.at(j).p.id == part.id)
            {
                products_kit_tray_2_.erase(products_kit_tray_2_.begin() + j);
            }
        }
    }
    else
    {
        for (int j = 0; j < products_kit_tray_1_.size(); ++j)
        {
            if (products_kit_tray_1_.at(j).p.id == part.id)
            {
                products_kit_tray_1_.erase(products_kit_tray_1_.begin() + j);
            }
        }
    }
}

/**
 * @brief Empty parts from left arm to corresponding AGV
 * 
 */
void GantryControl::placePartLeftArm()
{

    if (gantry_location_ == "ssi1")
    {
        goToPresetLocation(rail_1_);
    }
    if (gantry_location_ == "rail_1")
    {
        goToPresetLocation(agv1_90_);
    }
    if (gantry_location_ == "agv1_90" || gantry_location_ == "agv1")
    {
        goToPresetLocation(aisle1_90_);
    }
    if (gantry_location_ == "aisle1_90" || gantry_location_ == "aisle1")
    {
        goToPresetLocation(start_90_);
    }
    if (gantry_location_ == "ssi4")
    {
        goToPresetLocation(rail_2_);
    }
    if (gantry_location_ == "rail_2")
    {
        goToPresetLocation(agv2_90_);
    }
    if (gantry_location_ == "agv2_90" || gantry_location_ == "agv2")
    {
        goToPresetLocation(aisle2_90_);
    }
    if (gantry_location_ == "aisle2_90" || gantry_location_ == "aisle2" || gantry_location_ == "bins" || gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "start" || gantry_location_ == "start_90")
    {
        goToPresetLocation(start_90_);
    }
    if (product_left_arm_.agv_id.compare("agv2") == 0 || product_left_arm_.agv_id.compare("any") == 0)
    {
        goToPresetLocation(agv2_90_);
        // goToPresetLocation(agv2_);
        if (product_left_arm_.pose.position.x >= 0)
        {
            goToPresetLocation(tray2_left_positive_);
        }
        else
        {
            goToPresetLocation(tray2_left_negative_);
        }
    }
    else
    {
        goToPresetLocation(agv1_90_);
        // goToPresetLocation(agv1_);
        if (product_left_arm_.pose.position.x >= 0)
        {
            goToPresetLocation(tray1_left_positive_);
        }
        else
        {
            goToPresetLocation(tray1_left_negative_);
        }
    }

    ROS_INFO_STREAM("Product left arm " << product_left_arm_.type << std::endl);
    auto target_pose_in_tray = getTargetWorldPose(product_left_arm_.pose, product_left_arm_.agv_id);

    tf2::Quaternion q_world_tray_pose(target_pose_in_tray.orientation.x,
                                      target_pose_in_tray.orientation.y,
                                      target_pose_in_tray.orientation.z,
                                      target_pose_in_tray.orientation.w);

    geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;

    // // //--TODO: Consider agv1 too

    double actual_z = target_pose_in_tray.position.z + 0.007;
    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[product_left_arm_.type]);

    tf2::Quaternion q_rotation = q_world_tray_pose * q_left_ee_link_part.inverse();
    q_rotation.normalize();

    target_pose_in_tray.orientation.x = q_rotation.getX();
    target_pose_in_tray.orientation.y = q_rotation.getY();
    target_pose_in_tray.orientation.z = q_rotation.getZ();
    target_pose_in_tray.orientation.w = q_rotation.getW();

    product_left_arm_.p.pose.position.x = target_pose_in_tray.position.x;
    product_left_arm_.p.pose.position.y = target_pose_in_tray.position.y;
    product_left_arm_.p.pose.position.z = actual_z;

    double part_roll, part_pitch, part_yaw;
    tf2::Matrix3x3(q_world_tray_pose).getRPY(part_roll, part_pitch, part_yaw);

    int pos_t;
    std::string type;
    pos_t = product_left_arm_.type.find("_");
    type = product_left_arm_.type.substr(0, pos_t);

    if (part_roll - 0.1 <= -3.14 && part_roll + 0.1 >= -3.14 && type.compare("pulley") == 0)
    {
        target_pose_in_tray.orientation.x = currentArmPose.orientation.x;
        target_pose_in_tray.orientation.y = currentArmPose.orientation.y;
        target_pose_in_tray.orientation.z = currentArmPose.orientation.z;
        target_pose_in_tray.orientation.w = currentArmPose.orientation.w;
        tf2::Quaternion q_rotation_flip(currentArmPose.orientation.x,
                                        currentArmPose.orientation.y,
                                        currentArmPose.orientation.z,
                                        currentArmPose.orientation.w);
        q_world_tray_pose = q_rotation_flip * q_left_ee_link_part;
        q_rotation_flip.normalize();

        product_left_arm_.p.pose.orientation.x = q_world_tray_pose.getX();
        product_left_arm_.p.pose.orientation.y = q_world_tray_pose.getY();
        product_left_arm_.p.pose.orientation.z = q_world_tray_pose.getZ();
        product_left_arm_.p.pose.orientation.w = q_world_tray_pose.getW();

        products_to_flip_.push_back(product_left_arm_);
    }

    const double offset_y = target_pose_in_tray.position.y - currentArmPose.position.y;

    const double offset_x = target_pose_in_tray.position.x - currentArmPose.position.x;

    if (currentArmPose.position.y > 0)
    {
        joint_group_positions_.at(1) -= offset_y - 0.2;
    }
    else
    {
        joint_group_positions_.at(1) -= offset_y + 0.2;
    }

    joint_group_positions_.at(0) += offset_x;

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
    if (product_left_arm_.agv_id.compare("agv2") == 0)
    {
        product_left_arm_.p.location = "agv_2";
        products_kit_tray_2_.push_back(product_left_arm_);
    }
    else
    {
        product_left_arm_.p.location = "agv_1";
        products_kit_tray_1_.push_back(product_left_arm_);
    }

    std::vector<double> retrieve{joint_group_positions_.at(0) -= offset_x, joint_group_positions_.at(1) += offset_y};

    FKGantry(retrieve);
}

/**
 * @brief Empty parts from right arm to corresponding AGV
 * 
 */
void GantryControl::placePartRightArm()
{
    if (gantry_location_ == "ssi1")
    {
        goToPresetLocation(rail_1_);
    }
    if (gantry_location_ == "rail_1")
    {
        goToPresetLocation(agv1_90_);
    }
    if (gantry_location_ == "agv1_90" || gantry_location_ == "agv1")
    {
        goToPresetLocation(aisle1_90_);
    }
    if (gantry_location_ == "aisle1_90" || gantry_location_ == "aisle1")
    {
        goToPresetLocation(start_90_);
    }
    if (gantry_location_ == "ssi4")
    {
        goToPresetLocation(rail_2_);
    }
    if (gantry_location_ == "rail_2")
    {
        goToPresetLocation(agv2_90_);
    }
    if (gantry_location_ == "agv2_90" || gantry_location_ == "agv2")
    {
        goToPresetLocation(aisle2_90_);
    }
    if (gantry_location_ == "aisle2_90" || gantry_location_ == "aisle2" || gantry_location_ == "bins" || gantry_location_ == "shelf_1" || gantry_location_ == "shelf_2" || gantry_location_ == "start" || gantry_location_ == "start_90")
    {
        goToPresetLocation(start_90_);
    }
    if (product_right_arm_.agv_id.compare("agv2") == 0 || product_right_arm_.agv_id.compare("any") == 0)
    {
        goToPresetLocation(agv2_90_);
        // goToPresetLocation(agv2_);
        if (product_right_arm_.pose.position.x >= 0)
        {
            goToPresetLocation(tray2_right_positive_);
        }
        else
        {
            goToPresetLocation(tray2_right_negative_);
        }
    }
    else
    {
        goToPresetLocation(agv1_90_);
        // goToPresetLocation(agv1_);
        if (product_right_arm_.pose.position.x >= 0)
        {
            goToPresetLocation(tray1_right_positive_);
        }
        else
        {
            goToPresetLocation(tray1_right_negative_);
        }
    }

    auto target_pose_in_tray = getTargetWorldPose(product_right_arm_.pose, product_right_arm_.agv_id);

    tf2::Quaternion q_world_tray_pose(target_pose_in_tray.orientation.x,
                                      target_pose_in_tray.orientation.y,
                                      target_pose_in_tray.orientation.z,
                                      target_pose_in_tray.orientation.w);

    geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;

    // // //--TODO: Consider agv1 too
    double actual_z = target_pose_in_tray.position.z + 0.007;
    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5 * model_height[product_right_arm_.type]);

    tf2::Quaternion q_rotation = q_world_tray_pose * q_right_ee_link_part.inverse();
    target_pose_in_tray.orientation.x = q_rotation.getX();
    target_pose_in_tray.orientation.y = q_rotation.getY();
    target_pose_in_tray.orientation.z = q_rotation.getZ();
    target_pose_in_tray.orientation.w = q_rotation.getW();

    product_right_arm_.p.pose.position.x = target_pose_in_tray.position.x;
    product_right_arm_.p.pose.position.y = target_pose_in_tray.position.y;
    product_right_arm_.p.pose.position.z = actual_z;

    double part_roll, part_pitch, part_yaw;
    tf2::Matrix3x3(q_world_tray_pose).getRPY(part_roll, part_pitch, part_yaw);

    int pos_t;
    std::string type;
    pos_t = product_right_arm_.type.find("_");
    type = product_right_arm_.type.substr(0, pos_t);
    bool flip_process{false};

    ros::param::get("/flip_process", flip_process);

    if (part_roll - 0.1 <= -3.14 && part_roll + 0.1 >= -3.14 && type.compare("pulley") == 0 && flip_process == 0)
    {
        target_pose_in_tray.orientation.x = currentArmPose.orientation.x;
        target_pose_in_tray.orientation.y = currentArmPose.orientation.y;
        target_pose_in_tray.orientation.z = currentArmPose.orientation.z;
        target_pose_in_tray.orientation.w = currentArmPose.orientation.w;
        tf2::Quaternion q_rotation_flip(currentArmPose.orientation.x,
                                        currentArmPose.orientation.y,
                                        currentArmPose.orientation.z,
                                        currentArmPose.orientation.w);
        q_world_tray_pose = q_rotation_flip * q_right_ee_link_part;

        product_right_arm_.p.pose.orientation.x = q_world_tray_pose.getX();
        product_right_arm_.p.pose.orientation.y = q_world_tray_pose.getY();
        product_right_arm_.p.pose.orientation.z = q_world_tray_pose.getZ();
        product_right_arm_.p.pose.orientation.w = q_world_tray_pose.getW();

        products_to_flip_.push_back(product_right_arm_);
    }

    const double offset_y = target_pose_in_tray.position.y - currentArmPose.position.y;

    const double offset_x = target_pose_in_tray.position.x - currentArmPose.position.x;

    if (currentArmPose.position.y > 0)
    {
        joint_group_positions_.at(1) -= offset_y - 0.2;
    }
    else
    {
        joint_group_positions_.at(1) -= offset_y + 0.2;
    }

    joint_group_positions_.at(0) += offset_x;

    full_robot_group_.setJointValueTarget(joint_group_positions_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (full_robot_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();

    ros::Duration(0.5).sleep();

    right_arm_group_.setPoseTarget(target_pose_in_tray);
    right_arm_group_.move();

    deactivateGripper("right_arm");

    ros::Duration(0.5).sleep();

    if (product_right_arm_.agv_id.compare("agv2") == 0)
    {
        product_right_arm_.p.location = "agv_2";
        products_kit_tray_2_.push_back(product_right_arm_);
    }
    else if (product_right_arm_.agv_id.compare("agv1") == 0 || product_right_arm_.agv_id.compare("any") == 0)
    {
        product_right_arm_.p.location = "agv_1";
        products_kit_tray_1_.push_back(product_right_arm_);
    }

    std::vector<double> retrieve{joint_group_positions_.at(0) -= offset_x, joint_group_positions_.at(1) += offset_y};

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
    full_robot_group_.setGoalTolerance(0.000001);

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

// void GantryControl::getProductsToFlip(std::vector<Part> partsToFlip)
// {
//     bool is_no_prod_right{false};

//     ros::param::get("/no_prod_right", is_no_prod_right);

//     auto target_pose_in_tray_left = getTargetWorldPose(product_left_arm_.pose, product_left_arm_.agv_id);

//     double x_prod_r{0};
//     double y_prod_r{0};

//     double roll_right, pitch_right, yaw_right{0};
//     if (is_no_prod_right != 1)
//     {
//         auto target_pose_in_tray_right = getTargetWorldPose(product_right_arm_.pose, product_right_arm_.agv_id);
//         x_prod_r = target_pose_in_tray_right.position.x;
//         y_prod_r = target_pose_in_tray_right.position.y;

//         tf2::Quaternion q_right(
//             product_right_arm_.pose.orientation.x,
//             product_right_arm_.pose.orientation.y,
//             product_right_arm_.pose.orientation.z,
//             product_right_arm_.pose.orientation.w);
//         tf2::Matrix3x3 m_right(q_right);
//         m_right.getRPY(roll_right, pitch_right, yaw_right);
//     }

//     double x_prod_l{target_pose_in_tray_left.position.x};
//     double y_prod_l{target_pose_in_tray_left.position.y};

//     double y_part{};
//     double x_part{};

//     tf2::Quaternion q_left(
//         product_left_arm_.pose.orientation.x,
//         product_left_arm_.pose.orientation.y,
//         product_left_arm_.pose.orientation.z,
//         product_left_arm_.pose.orientation.w);
//     tf2::Matrix3x3 m_left(q_left);
//     double roll_left, pitch_left, yaw_left{0};
//     ;
//     m_left.getRPY(roll_left, pitch_left, yaw_left);

//     if (partsToFlip.empty() != 1)
//     {
//         for (int i = 0; i < partsToFlip.size(); i++)
//         {
//             x_part = partsToFlip.at(i).pose.position.x;
//             y_part = partsToFlip.at(i).pose.position.y;

//             tf2::Quaternion q_part(
//                 partsToFlip.at(i).pose.orientation.x,
//                 partsToFlip.at(i).pose.orientation.y,
//                 partsToFlip.at(i).pose.orientation.z,
//                 partsToFlip.at(i).pose.orientation.w);
//             tf2::Matrix3x3 m_part(q_part);
//             double roll_part, pitch_part, yaw_part;
//             m_part.getRPY(roll_part, pitch_part, yaw_part);

//             if (-1 <= roll_part && 1 >= roll_part)
//             {
//                 if ((x_prod_l - 1) <= x_part && (x_prod_l + 1) >= x_part && (y_prod_l - 1) <= y_part && (y_prod_l + 1) >= y_part && (((PI - 1) <= roll_left && (PI + 1) >= roll_left) || ((-PI - 1) <= roll_left && (-PI + 1) >= roll_left)))
//                 {
//                     product_left_arm_.p = partsToFlip.at(i);
//                     products_to_flip_.push_back(product_left_arm_);
//                 }
//                 else if ((x_prod_r - 1) <= x_part && (x_prod_r + 1) >= x_part && (y_prod_r - 1) <= y_part && (y_prod_r + 1) >= y_part && (((PI - 1) <= roll_right && (PI + 1) >= roll_right) || ((-PI - 1) <= roll_right && (-PI + 1) >= roll_right)) && is_no_prod_right != 1)
//                 {
//                     product_right_arm_.p = partsToFlip.at(i);
//                     products_to_flip_.push_back(product_right_arm_);
//                 }
//             }
//         }
//     }
// }

void GantryControl::flipProductsAGV(std::vector<Part> checkPartsToFlip)
{
    ros::param::set("/flip_process", true);

    bool parts_to_flip_in_trays{}; // will equal false when there is sesor blackout so that we do not flip parts

    if (checkPartsToFlip.empty() != 0 || products_to_flip_.empty() != 0)
    {
        parts_to_flip_in_trays = false;
    }
    else
    {
        ROS_WARN_STREAM("CHECK PARTS TO FLIP SIZE: " << checkPartsToFlip.size());
        ROS_WARN_STREAM("PRODUCTS TO FLIP SIZE: " << products_to_flip_.size());
        for (int i = 0; i < products_to_flip_.size(); i++)
        { // it can have pulleys that have already been flipped
            parts_to_flip_in_trays = false;
            for (int j = 0; j < checkPartsToFlip.size(); j++)
            {
                if (checkPartsToFlip.at(j).pose.position.x + 0.1 >= products_to_flip_.at(i).p.pose.position.x &&
                    checkPartsToFlip.at(j).pose.position.x - 0.1 <= products_to_flip_.at(i).p.pose.position.x &&
                    checkPartsToFlip.at(j).pose.position.y + 0.1 >= products_to_flip_.at(i).p.pose.position.y &&
                    checkPartsToFlip.at(j).pose.position.y - 0.1 <= products_to_flip_.at(i).p.pose.position.y)
                {
                    parts_to_flip_in_trays = true;
                    break;
                }
            }
            if (parts_to_flip_in_trays == false)
            {
                break;
            }
        }
    }
    if (products_to_flip_.empty() == 0 && parts_to_flip_in_trays)
    {
        for (int i = 0; i < products_to_flip_.size(); i++)
        {
            products_to_flip_.at(i).p.pose.position.z += 0.01;
            if (products_to_flip_.at(i).agv_id.compare("agv2") == 0 || products_to_flip_.at(i).agv_id.compare("any") == 0)
            {
                goToPresetLocation(agv2_);
                if (products_to_flip_.at(i).p.pose.position.x >= 0)
                {
                    goToPresetLocation(tray2_left_positive_);
                }
                else
                {
                    goToPresetLocation(tray2_left_negative_);
                }
            }
            else
            {
                goToPresetLocation(agv1_);
                if (-products_to_flip_.at(i).p.pose.position.x >= 0)
                {
                    goToPresetLocation(tray1_left_positive_);
                }
                else
                {
                    goToPresetLocation(tray1_left_negative_);
                }
            }

            geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;

            const double offset_y = products_to_flip_.at(i).p.pose.position.y - currentArmPose.position.y;
            const double offset_x = products_to_flip_.at(i).p.pose.position.x - currentArmPose.position.x;

            if (currentArmPose.position.y > 0)
            {
                joint_group_positions_.at(1) -= offset_y - 0.2;
            }
            else
            {
                joint_group_positions_.at(1) -= offset_y + 0.2;
            }
            joint_group_positions_.at(0) += offset_x;

            full_robot_group_.setJointValueTarget(joint_group_positions_);

            moveit::planning_interface::MoveGroupInterface::Plan move_x;
            bool success = (full_robot_group_.plan(move_x) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
                full_robot_group_.move();

            ros::Duration(1).sleep();
            pickPartLeftArm(products_to_flip_.at(i).p);
            ros::Duration(1).sleep();

            std::vector<double> retrieve{joint_group_positions_.at(0) -= offset_x, joint_group_positions_.at(1) += offset_y};

            FKGantry(retrieve);

            if (products_to_flip_.at(i).agv_id.compare("agv2") == 0 || products_to_flip_.at(i).agv_id.compare("any") == 0)
            {
                goToPresetLocation(agv2_);
                for (int j = 0; j < products_kit_tray_2_.size(); ++j)
                {
                    if (products_kit_tray_2_.at(j).p.id == products_to_flip_.at(i).p.id)
                    {
                        products_kit_tray_2_.erase(products_kit_tray_2_.begin() + j);
                    }
                }
            }
            else
            {
                goToPresetLocation(agv1_);
                for (int j = 0; j < products_kit_tray_1_.size(); ++j)
                {
                    if (products_kit_tray_1_.at(j).p.id == products_to_flip_.at(i).p.id)
                    {
                        products_kit_tray_1_.erase(products_kit_tray_1_.begin() + j);
                    }
                }
            }

            FKLeftArm({1.29, -0.25, 1.82, 0.0, 1.59, 0.0}); //position left arm

            FKRightArm({1.19, -0.12, 1.7, 0.0, 1.57, 0.0}); //position right arm

            FKRightArm({1.19, 0.01, 1.57, 0.0, 1.57, 0.0}); //approximate to part right arm

            activateGripper("right_arm");

            ros::Duration(1).sleep();

            geometry_msgs::Pose currentPoseLeftArm = left_arm_group_.getCurrentPose().pose;
            geometry_msgs::Pose currentPoseRightArm = right_arm_group_.getCurrentPose().pose;

            tf2::Quaternion q_arm_left(currentPoseLeftArm.orientation.x,
                                       currentPoseLeftArm.orientation.y,
                                       currentPoseLeftArm.orientation.z,
                                       currentPoseLeftArm.orientation.w);

            tf2::Quaternion q_arm_right(currentPoseRightArm.orientation.x,
                                        currentPoseRightArm.orientation.y,
                                        currentPoseRightArm.orientation.z,
                                        currentPoseRightArm.orientation.w);

            tf2::Quaternion q_world_part;

            q_world_part = q_arm_left * q_left_ee_link_part;
            q_world_part.normalize();

            q_right_ee_link_part = q_arm_right.inverse() * q_world_part;
            q_right_ee_link_part.normalize();

            deactivateGripper("left_arm");

            product_right_arm_ = products_to_flip_.at(i);

            ros::Duration(0.5).sleep();

            placePartRightArm();
        }

        products_to_flip_.clear();
        ros::param::set("/flip_process", false);
    }
}

/**
 * @brief flow to get a part from the conveyorbelt with the left arm
 * 
 * @param product which product to get
 * @return true 
 * @return false 
 */
bool GantryControl::getPartConveyorLeftArm(product product)
{
    double current_y;
    double original_z;
    double original_y;
    bool picked{false};
    double offset_y;
    double offset_x;
    geometry_msgs::Pose currentArmPose;

    original_z = product.p.pose.position.z;
    original_y = product.p.pose.position.y - 0.2 * (ros::Time::now().toSec() - product.p.time_stamp.toSec());

    gantry_location_ = "conveyor";
    rotateTorso(-PI / 2);
    int sum{0};
    while (sum < 4)
    {
        currentArmPose = left_arm_group_.getCurrentPose().pose;

        current_y = product.p.pose.position.y - 0.2 * (ros::Time::now().toSec() - product.p.time_stamp.toSec());
        ROS_WARN_STREAM("SUM: " << sum);
        ROS_WARN_STREAM("CURRENT Y ARM POSE: " << currentArmPose.position.y);
        ROS_WARN_STREAM("CURRENT Y PART POSE: " << current_y);
        offset_y = current_y - currentArmPose.position.y;
        offset_x = product.p.pose.position.x - currentArmPose.position.x;

        joint_group_positions_.at(0) += offset_x;
        joint_group_positions_.at(1) -= offset_y + 0.07;
        gantry_group_.setJointValueTarget({joint_group_positions_.at(0), joint_group_positions_.at(1), joint_group_positions_.at(2)});
        gantry_group_.move();

        if (sum == 0)
        {
            currentArmPose = left_arm_group_.getCurrentPose().pose;
            product.p.pose.position.z = product.p.pose.position.z + model_height.at(product.p.type) + GRIPPER_HEIGHT - EPSILON + 0.02;
            product.p.pose.position.y = currentArmPose.position.y;
            product.p.pose.orientation.x = currentArmPose.orientation.x;
            product.p.pose.orientation.y = currentArmPose.orientation.y;
            product.p.pose.orientation.z = currentArmPose.orientation.z;
            product.p.pose.orientation.w = currentArmPose.orientation.w;
            left_arm_group_.setPoseTarget(product.p.pose);
            left_arm_group_.move();
        }

        sum += 1;
    }

    currentArmPose = left_arm_group_.getCurrentPose().pose;
    current_y = product.p.pose.position.y - 0.2 * (ros::Time::now().toSec() - product.p.time_stamp.toSec());
    offset_y = current_y - currentArmPose.position.y;
    offset_x = product.p.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;
    joint_group_positions_.at(1) -= offset_y + 0.07;

    gantry_group_.setJointValueTarget({joint_group_positions_.at(0), joint_group_positions_.at(1), joint_group_positions_.at(2)});
    gantry_group_.move();

    product.p.pose.position.y = current_y + 0.07;
    product.p.pose.position.z = original_z + 0.01;
    activateGripper("left_arm");

    product.p.pose.position.z = product.p.pose.position.z + model_height.at(product.p.type) + GRIPPER_HEIGHT - EPSILON;
    product.p.pose.orientation.x = currentArmPose.orientation.x;
    product.p.pose.orientation.y = currentArmPose.orientation.y;
    product.p.pose.orientation.z = currentArmPose.orientation.z;
    product.p.pose.orientation.w = currentArmPose.orientation.w;
    left_arm_group_.setPoseTarget(product.p.pose);
    left_arm_group_.move();

    joint_group_positions_.at(1) += 0.8;

    full_robot_group_.setJointValueTarget({joint_group_positions_.at(0), joint_group_positions_.at(1), joint_group_positions_.at(2),
                                           -0.19, -0.74, 1.49, -0.75, 1.39, 0, joint_group_positions_.at(9), joint_group_positions_.at(10), joint_group_positions_.at(11),
                                           joint_group_positions_.at(12), joint_group_positions_.at(13), joint_group_positions_.at(14)});
    full_robot_group_.move();
    ros::Duration(0.5).sleep();
    goToPresetLocation(start_);
    gantry_location_ = "start";

    picked = true;

    product_left_arm_ = product;

    return picked;
}

/**
 * @brief flow to get a part from the conveyorbelt with the right arm
 * 
 * @param product which product to get
 * @return true 
 * @return false 
 */
bool GantryControl::getPartConveyorRightArm(product product)
{
    double current_y;
    double original_z;
    double original_y;
    bool picked{false};
    double offset_y;
    double offset_x;
    geometry_msgs::Pose currentArmPose;

    original_z = product.p.pose.position.z;
    original_y = product.p.pose.position.y - 0.2 * (ros::Time::now().toSec() - product.p.time_stamp.toSec());

    gantry_location_ = "conveyor";
    rotateTorso(PI / 2);
    int sum{0};
    while (sum < 4)
    {
        currentArmPose = right_arm_group_.getCurrentPose().pose;
        current_y = original_y - 0.2 * (ros::Time::now().toSec() - product.p.time_stamp.toSec());
        offset_y = current_y - currentArmPose.position.y;
        offset_x = product.p.pose.position.x - currentArmPose.position.x;

        joint_group_positions_.at(0) += offset_x;
        joint_group_positions_.at(1) -= offset_y - 0.25;
        gantry_group_.setJointValueTarget({joint_group_positions_.at(0), joint_group_positions_.at(1), joint_group_positions_.at(2)});
        gantry_group_.move();

        if (sum == 0)
        {
            currentArmPose = right_arm_group_.getCurrentPose().pose;
            product.p.pose.position.z = product.p.pose.position.z + model_height.at(product.p.type) + GRIPPER_HEIGHT - EPSILON + 0.02;
            product.p.pose.position.y = currentArmPose.position.y;
            product.p.pose.orientation.x = currentArmPose.orientation.x;
            product.p.pose.orientation.y = currentArmPose.orientation.y;
            product.p.pose.orientation.z = currentArmPose.orientation.z;
            product.p.pose.orientation.w = currentArmPose.orientation.w;
            right_arm_group_.setPoseTarget(product.p.pose);
            right_arm_group_.move();
        }

        sum += 1;
    }

    currentArmPose = right_arm_group_.getCurrentPose().pose;
    current_y = original_y - 0.2 * (ros::Time::now().toSec() - product.p.time_stamp.toSec());
    offset_y = current_y - currentArmPose.position.y;
    offset_x = product.p.pose.position.x - currentArmPose.position.x;

    joint_group_positions_.at(0) += offset_x;
    joint_group_positions_.at(1) -= offset_y - 0.35;

    gantry_group_.setJointValueTarget({joint_group_positions_.at(0), joint_group_positions_.at(1), joint_group_positions_.at(2)});
    gantry_group_.move();

    product.p.pose.position.y = current_y - 0.35;
    product.p.pose.position.z = original_z + 0.01;
    activateGripper("right_arm");

    product.p.pose.position.z = product.p.pose.position.z + model_height.at(product.p.type) + GRIPPER_HEIGHT - EPSILON;
    product.p.pose.orientation.x = currentArmPose.orientation.x;
    product.p.pose.orientation.y = currentArmPose.orientation.y;
    product.p.pose.orientation.z = currentArmPose.orientation.z;
    product.p.pose.orientation.w = currentArmPose.orientation.w;
    right_arm_group_.setPoseTarget(product.p.pose);
    right_arm_group_.move();

    joint_group_positions_.at(1) += 0.8;

    full_robot_group_.setJointValueTarget({joint_group_positions_.at(0), joint_group_positions_.at(1), joint_group_positions_.at(2),
                                           -0.19, -0.74, 1.49, -0.75, 1.39, 0, joint_group_positions_.at(9), joint_group_positions_.at(10), joint_group_positions_.at(11),
                                           joint_group_positions_.at(12), joint_group_positions_.at(13), joint_group_positions_.at(14)});
    full_robot_group_.move();
    ros::Duration(0.5).sleep();
    goToPresetLocation(start_);

    gantry_location_ = "start";

    picked = true;

    product_right_arm_ = product;

    return picked;
}

void GantryControl::getLeftArmRoll()
{
    geometry_msgs::Pose currentArmPose = left_arm_group_.getCurrentPose().pose;
    tf2::Quaternion q_p(currentArmPose.orientation.x,
                        currentArmPose.orientation.y,
                        currentArmPose.orientation.z,
                        currentArmPose.orientation.w);
    tf2::Matrix3x3 m_p(q_p);
    double roll_p, pitch_p, yaw_p;
    m_p.getRPY(roll_p, pitch_p, yaw_p);
    ROS_WARN_STREAM("LEFT GRIPPER ROLL ANGLE: " << roll_p);
    ROS_WARN_STREAM("LEFT GRIPPER PITCH ANGLE: " << pitch_p);
    ROS_WARN_STREAM("LEFT GRIPPER YAW ANGLE: " << yaw_p);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // ros::Rate rate(10);
    ros::Duration timeout(5.0);

    geometry_msgs::TransformStamped world_left_arm;
    for (int i = 0; i < 10; i++)
    {
        try
        {
            world_left_arm = tfBuffer.lookupTransform("world", "left_ee_link",
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

    tf2::Quaternion q(world_left_arm.transform.rotation.x,
                      world_left_arm.transform.rotation.y,
                      world_left_arm.transform.rotation.z,
                      world_left_arm.transform.rotation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("LEFT GRIPPER ROLL ANGLE: " << roll);
    ROS_WARN_STREAM("LEFT GRIPPER PITCH ANGLE: " << pitch);
    ROS_WARN_STREAM("LEFT GRIPPER YAW ANGLE: " << yaw);
}

void GantryControl::getRightArmRoll()
{
    geometry_msgs::Pose currentArmPose = right_arm_group_.getCurrentPose().pose;
    tf2::Quaternion q(currentArmPose.orientation.x,
                      currentArmPose.orientation.y,
                      currentArmPose.orientation.z,
                      currentArmPose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_WARN_STREAM("RIGHT GRIPPER ROLL ANGLE: " << roll);
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
