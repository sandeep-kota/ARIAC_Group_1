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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3

#include "competition.h"
#include "utils.h"
#include "gantry_control.h"
#include "agv_control.h"
#include "sensor_control.h"

#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    GantryControl gantry(node);
    gantry.init();

    SensorControl sensors(node);
    sensors.init();

    ros::Duration(5).sleep();

    gantry.goToPresetLocation(gantry.start_);

    //--1-Read order
    while(comp.processOrder())
    {
        std::vector<Shipment> list_of_shipments = comp.get_shipment_list();
        std::vector<Product> list_of_products = comp.get_product_list();

        ROS_INFO_STREAM("Order processed");

        for (int p = 0; p < list_of_products.size(); p++)
        {
            Product current_product = list_of_products.at(p);

            ROS_INFO_STREAM(current_product.type);

            //--2-Look for parts in this order
            current_product.p = sensors.findPart(current_product.type);

            if (gantry.checkFreeGripper() == "none")
            {
                if (gantry.getGantryLocation() == "aisle_1")
                {
                    gantry.goToPresetLocation(gantry.aisle1_);
                } else if (gantry.getGantryLocation() == "aisle_2")
                {
                    gantry.goToPresetLocation(gantry.aisle2_);
                }

                gantry.goToPresetLocation(gantry.start_);
                
                break;

            } else {

                gantry.getProduct(current_product.p);
            }            

            // gantry.goToPresetLocation(gantry.start_);
        }
        
    }
    
    
    
    
    //--We go to this bin because a camera above
    //--this bin found one of the parts in the order
    // gantry.goToPresetLocation(gantry.bins1_);
    // gantry.goToPresetLocation(gantry.bins2_);
    // gantry.goToPresetLocation(gantry.bins3_);
    // gantry.goToPresetLocation(gantry.bins4_);
    // gantry.rotateTorso(TR_RIGHT_ARM);
    // gantry.rotateTorso(TL_RIGHT_ARM);
    // gantry.rotateTorso(BR_RIGHT_ARM);
    // gantry.rotateTorso(BL_RIGHT_ARM);

    // gantry.rotateTorso(TR_LEFT_ARM);
    // gantry.rotateTorso(TL_LEFT_ARM);
    // gantry.rotateTorso(BR_LEFT_ARM);
    // gantry.rotateTorso(BL_LEFT_ARM);

    // gantry.goToPresetLocation(gantry.start_);
    
    //--TODO: Parse each product in list_of_products
    //--TODO: For each product in list_of_product find the part in the environment using cameras
    //--TODO: Choose one part and pick it up
    //--Assume the following part was found by a camera
    // part my_part1;
    // my_part1.type = "pulley_part_red";
    // my_part1.location = "shelf_5";
    // my_part1.pose.position.x = -13.522081;
    // my_part1.pose.position.y = 3.446263;
    // my_part1.pose.position.z = 1.398754;
    // my_part1.pose.orientation.x = 0.012;
    // my_part1.pose.orientation.y = -0.004;
    // my_part1.pose.orientation.z = 0.002;
    // my_part1.pose.orientation.w = 1.000;
    // tf2::Quaternion myQuaternion;
    // myQuaternion.setRPY(0.029299, 0.000598, 0.000003);
    // part my_part2;
    // my_part2.type = "pulley_part_blue";
    // my_part2.location = "shelf_5";
    // my_part2.pose.position.x = -14.522080;
    // my_part2.pose.position.y = 3.446263;
    // my_part2.pose.position.z = 1.398766;
    // my_part2.pose.orientation.x = myQuaternion.x();
    // my_part2.pose.orientation.y = myQuaternion.y();
    // my_part2.pose.orientation.z = myQuaternion.z();
    // my_part2.pose.orientation.w = myQuaternion.w();

    //--Where to place the part in tmyQuaternion.y()he tray?
    //--TODO: Get this information from /ariac/orders (list_of_products in this case)
    // part part_in_tray;
    // part_in_tray.type = "pulley_part_red";
    // part_in_tray.pose.position.x = -0.1;
    // part_in_tray.pose.position.y = -0.2;
    // part_in_tray.pose.position.z = 0.0;
    // myQuaternion.setRPY(0.032322, 0.003832, 0.010435);
    // part_in_tray.pose.orientation.x = myQuaternion.x();;
    // part_in_tray.pose.orientation.y = myQuaternion.y();
    // part_in_tray.pose.orientation.z = myQuaternion.z();
    // part_in_tray.pose.orientation.w = myQuaternion.w();
    

    // gantry.getPart(my_part1);
    // gantry.getPart(my_part2);
    // gantry.getPart(my_part1);
    // gantry.retriveFromBottomShelf();
    // gantry.goToPresetLocation(gantry.aisle1_);
    // gantry.goToPresetLocation(gantry.start_);
    // gantry.goToPresetLocation(gantry.agv2_);

    // ros::Duration(2).sleep();
    // gantry.printPartOrient();
    // gantry.goToPresetLocation(gantry.bins2_);
    // gantry.rotateTorso(BR_RIGHT_ARM);
    // gantry.pickPartRightArm(my_part2);

    // gantry.reachPartShelfRightArm(my_part2);
    // gantry.pickPartRightArm(my_part2);
    // gantry.retriveFromBottomShelf();
    // gantry.goToPresetLocation(gantry.aisle1_);
    // gantry.goToPresetLocation(gantry.start_);
    //--Go pick the part
    // if (!gantry.pickPart(my_part1)){
    //     gantry.goToPresetLocation(gantry.start_);
    //     spinner.stop();
    //     ros::shutdown();
    // }
    
    //--Go place the part
    //--TODO: agv2 should be retrieved from /ariac/orders (list_of_products in this case)
    // gantry.placePart(part_in_tray, "agv2");

    // AGVControl agv_control(node);
    //--TODO: get the following arguments from the order
    // agv_control.sendAGV("order_0_shipment_0", "kit_tray_2");
    // comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}