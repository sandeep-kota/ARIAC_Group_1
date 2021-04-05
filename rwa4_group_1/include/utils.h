#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <unordered_map>
#include <string>

#include <ros/ros.h>

#include <nist_gear/VacuumGripperState.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <queue>

#define NUM_LOGICAL_CAMERAS 16
#define NUM_BELT_CAMERAS 1

typedef struct Shipment shipment; // forward declarations
typedef struct Order order;
typedef struct Product product;

const double PI = 3.141592; // TODO correct!

// Logical cameras
const int MAX_NUMBER_OF_CAMERAS = 17;
const int NUM_QUALITY_SENSORS = 2;
const std::string LEFT_ARM = "left_arm";
const std::string RIGHT_ARM = "right_arm";

const int MAX_PICKING_ATTEMPTS = 3; // for pickup
const double ABOVE_TARGET = 0.2; // above target z pos when picking/placing part
const double PICK_TIMEOUT = 4.0;
const double RETRIEVE_TIMEOUT = 2.0;

const double BELT_SPEED = 0.2; // m/s

const double GRIPPER_HEIGHT = 0.01;
const double EPSILON = 0.008; // for the gripper to firmly touch

const double BIN_HEIGHT = 0.724;
const double TRAY_HEIGHT = 0.755;
const double RAIL_HEIGHT = 0.95;

const double PLANNING_TIME = 20; // for move_group
const int MAX_EXCHANGE_ATTEMPTS = 6; // Pulley flip

const double TL_LEFT_ARM = PI/4;
const double TR_LEFT_ARM = -PI/4;
const double BL_LEFT_ARM = 3*PI/4;
const double BR_LEFT_ARM = -3*PI/4;

const double TL_RIGHT_ARM = -3*PI/4;
const double TR_RIGHT_ARM = 3*PI/4;
const double BL_RIGHT_ARM = -PI/4;
const double BR_RIGHT_ARM = PI/4;

const double L_LEFT_ARM = PI/2;
const double R_LEFT_ARM = -PI/2;
const double L_RIGHT_ARM = -PI/2;
const double R_RIGHT_ARM = PI/2;

const std::string AGV1 = "agv_1";
const std::string AGV2 = "agv_2";

const std::string AGV1_ID = "agv1";
const std::string AGV2_ID = "agv2";

const std::string AGV1_TRAY = "kit_tray_1";
const std::string AGV2_TRAY = "kit_tray_2";

extern std::string action_state_name[];
extern std::unordered_map<std::string, double> model_height;

enum PartStates {FREE, BOOKED, UNREACHABLE, ON_TRAY, GRIPPED, GOING_HOME,
  REMOVE_FROM_TRAY, LOST};


/**
 * @brief Struct for preset locations
 * @todo Add new preset locations here
 * 
 */
typedef struct PresetLocation {
    std::string location;
    std::vector<double> gantry;
    std::vector<double> left_arm;
    std::vector<double> right_arm;
} start, bins, shelf1, shelf2, aisle1 , aisle2, agv1, agv1_left, agv1_right, agv2, agv2_left, agv2_right, tray1_left_positive, 
tray1_left_negative, tray1_right_positive, tray1_right_negative, tray2_left_positive, 
tray2_left_negative, tray2_right_positive, tray2_right_negative;


/**
 * @brief Struct to store part information
 * 
 */
typedef struct Part {
  std::string type; // model type
  geometry_msgs::Pose pose; // model pose (in frame)
  geometry_msgs::Pose save_pose;
  std::string frame; // model frame (e.g., "logical_camera_1_frame")
  ros::Time time_stamp;
  std::string id;
  std::string location;
//   bool faulty;
  bool picked_status;
} part;




/**
 * @brief Struct to store joint positions for each group
 * 
 */
typedef struct Position {
    std::vector<double> gantry;
    std::vector<double> left;
    std::vector<double> right;
} position;



/**
 * @brief Struct to store shipments
 * 
 */
typedef struct Shipment {
    std::string shipment_type;
    std::string agv_id;
    std::vector<Product> products;
} shipment;




/**
 * @brief Struct to store products
 * 
 */
typedef struct Product {
    std::string type;
    geometry_msgs::Pose pose;
    part p; // NEW here!
    // std::string frame_of_origin;
    geometry_msgs::Pose actual_pose;
    std::string actual_pose_frame;
    std::string agv_id;
    std::string tray;
    std::string arm_name;
} product;



/**
 * @brief struct to parse and store orders published on /ariac/orders
 * 
 */
typedef struct Order {
    std::string order_id;
    std::vector<Shipment> shipments;
} order;

inline std::ostream& operator<<(std::ostream& os, const Part& part)
{
    os << "Part Details:\n"<< "model type: "<< part.type << 
    "\nID: "<<part.id<<
    "\nLocation: "<<part.location<<
     std::fixed<< std::setprecision(2)<<
    "\nPose Point: "<<part.pose.position.x<< " "<< part.pose.position.y<< " "<< part.pose.position.z<<"\n" ;
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Position& pos)
{
    os << "Position Details:\n"<< 
    "Gantry:";
    for(auto i: pos.gantry) {
      os<<i<<" ";
    }
    os<<"\nleft: ";
    for(auto i: pos.left) {
      os<<i<<" ";
    }
    os<<"\nright: ";
    for(auto i: pos.right) {
      os<<i<<" ";
    }
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Product& prod)
{
    os << "Product Details:\n"<< 
    "model type: "<< prod.type << 
    "\nAGV ID: "<<prod.agv_id<<
    "\nArm Name: "<<prod.arm_name<<
    "\nTray Name: "<<prod.tray<<
    "\nActual Pose Frame: "<<prod.actual_pose_frame<<
     std::fixed<< std::setprecision(2)<<
    "\nPose Point: "<<prod.pose.position.x<< " "<< prod.pose.position.y<< " "<< prod.pose.position.z <<
    "\nActual Pose Point: "<<prod.actual_pose.position.x<< " "<< prod.actual_pose.position.y<< " "<< prod.actual_pose.position.z <<
    std::endl<<"Product "<<prod.p;
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Shipment& ship)
{
    os << "Shipment Details:\n"<< 
    "Shipment type: "<< ship.shipment_type << 
    "\nShipment AGV ID: "<<ship.agv_id<<
    "\nProduct List:\n";
    for(int i =0; i< ship.products.size(); ++i) {
      os<<"Product "<<i+1<<"\n"<<ship.products[i]<<std::endl;
    }
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Order& ord)
{
    os << "Order Details:\n"<< 
    "Order ID: "<< ord.order_id <<std::endl;
    for(int i=0; i<ord.shipments.size(); ++i) {
      os<<"Shipment "<<i+1<<"\n"<<ord.shipments[i]<<std::endl;
    }
    
    return os;
}

#endif