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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    std::vector<Product> list_of_products;  // list of products to retrieve in order
    std::vector<Shipment> list_of_shipments; // list of shipments to complete in order
    Product current_product;

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    GantryControl gantry(node);
    gantry.init();

    SensorControl sensors(node);
    sensors.init();                 // initialize the sensor callbacks of the environment

    gantry.goToPresetLocation(gantry.start_); // start the trial from start position

    
    while(comp.processOrder() && sensors.read_all_sensors_) //--1-Read order until no more found
    // while (1)
    {
        list_of_shipments = comp.get_shipment_list(); // get list of shipments of current order in priority order
        list_of_products = comp.get_product_list();   // get list of products of current order in priority order  

        for (int p = 0; p < list_of_products.size(); p++)   // loop all the products to be retrieve from current order
        // for (int p = 0; p = 1; p++)   // loop all the products to be retrieve from current order
        {

            current_product = list_of_products.at(p);

            ROS_WARN_STREAM(current_product.type); 

            current_product.p = sensors.findPart(current_product.type); //--2-Look for parts in this order

            ROS_WARN_STREAM(current_product.p.location);

            if(current_product.p.type.empty())  // no parts of desired product found
            {
                ROS_WARN_STREAM("NO PART FOUND");
            }

            if (gantry.checkFreeGripper().compare("none") == 0)     // if none of the grippers are free place both products in grippers
            {
                if (gantry.getGantryLocation().compare("aisle_1") == 0)     // go to start location from current gantry location
                {
                    gantry.goToPresetLocation(gantry.aisle1_);
                }
                else if (gantry.getGantryLocation().compare("aisle_2") == 0)
                {
                    gantry.goToPresetLocation(gantry.aisle2_);
                }

                gantry.goToPresetLocation(gantry.start_);   

                
                gantry.placePartLeftArm();  // Place product of left arm in agv
                ROS_WARN_STREAM("FAULTY PARTS :" << sensors.faulty_parts_);
                gantry.throwLastPartLeft();

                // gantry.placePartRightArm(); // Place product of right arm in agv
                // ROS_WARN_STREAM("FAULTY PARTS :" << sensors.faulty_parts_);

                gantry.goToPresetLocation(gantry.start_); // go back to start position
            }
            
            if (p < list_of_products.size())    // get product not called in last iteration
                gantry.getProduct(current_product); // get product after placing in agv
            
        }
    }

    // Place in agv the two last retrieved products
    if (gantry.checkFreeGripper().compare("none") == 0)     // if none of the grippers are free place both products in grippers
            {
                if (gantry.getGantryLocation().compare("aisle_1") == 0)     // go to start location from current gantry location
                {
                    gantry.goToPresetLocation(gantry.aisle1_);
                }
                else if (gantry.getGantryLocation().compare("aisle_2") == 0)
                {
                    gantry.goToPresetLocation(gantry.aisle2_);
                }

                gantry.goToPresetLocation(gantry.start_);   

                
                gantry.placePartLeftArm();  // Place product of left arm in agv

                gantry.placePartRightArm(); // Place product of right arm in agv

                gantry.goToPresetLocation(gantry.start_); // go back to start position
            }

    spinner.stop();
    ros::shutdown();
    return 0;
}
