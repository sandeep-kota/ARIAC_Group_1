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

/**
 * @brief Update Orders list
 * 
 * @param list_of_products List of products
 * @param faultyProduct list of faulty products
 */
void updateOrderProductList(std::vector<Product> list_of_products, Product faultyProduct)
{
    Product productToBeReplaced;
    productToBeReplaced.type = faultyProduct.type;
    productToBeReplaced.p = faultyProduct.p;
    productToBeReplaced.type = faultyProduct.type;
    productToBeReplaced.pose = faultyProduct.pose;
    productToBeReplaced.actual_pose_frame = faultyProduct.actual_pose_frame;
    productToBeReplaced.agv_id = faultyProduct.agv_id;

    list_of_products.push_back(productToBeReplaced);
    ROS_INFO_STREAM("Product list updated with the product to be replaced. NewSize: " << list_of_products.size() << " Type: " << faultyProduct.type);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rwa4_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    std::vector<Product> list_of_products;   // list of products to retrieve in order
    std::vector<Shipment> list_of_shipments; // list of shipments to complete in order
    Product current_product;

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    GantryControl gantry(node);
    gantry.init();

    SensorControl sensors(node);
    sensors.init(); // initialize the sensor callbacks of the environment
    ros::param::set("/activate_quality", false);

    gantry.goToPresetLocation(gantry.start_); // start the trial from start position

    AGVControl agvControl(node);

    while (comp.processOrder() && sensors.read_all_sensors_) //--1-Read order until no more found
    {
        list_of_shipments = comp.get_shipment_list(); // get list of shipments of current order in priority order
        list_of_products = comp.get_product_list();   // get list of products of current order in priority order

        for (int p = 0; p < list_of_products.size(); p++) // loop all the products to be retrieve from current order
        {

            current_product = list_of_products.at(p);

            ROS_WARN_STREAM(current_product.type);

            current_product.p = sensors.findPart(current_product.type); //--2-Look for parts in this order

            ROS_WARN_STREAM(current_product.p.location);

            if (current_product.p.type.empty()) // no parts of desired product found
            {
                ROS_WARN_STREAM("NO PART FOUND");
            }

            if (gantry.checkFreeGripper().compare("none") == 0) // if none of the grippers are free place both products in trays
            {
                if (gantry.getGantryLocation().compare("aisle_1") == 0) // go to start location from current gantry location
                {
                    gantry.goToPresetLocation(gantry.aisle1_);
                }
                else if (gantry.getGantryLocation().compare("aisle_2") == 0)
                {
                    gantry.goToPresetLocation(gantry.aisle2_);
                }

                gantry.goToPresetLocation(gantry.start_);

                gantry.placePartLeftArm(); // Place product of left arm in agv

                //Check for faulty product
                ros::Duration(1).sleep();
                ros::param::set("/activate_quality", true);
                ros::Duration(1).sleep();

                bool activate_quality;
                ros::param::get("/activate_quality", activate_quality);
                ROS_WARN_STREAM("ACTIVATE QUALITY: " << activate_quality);
                if (sensors.isFaultyPartDetected())
                {
                    std::vector<Product> faultyProducts = sensors.getFaultyProducts();
                    ROS_WARN_STREAM("FAULTY PARTS :" << faultyProducts[0].p.location);

                    gantry.throwLastPartLeft(faultyProducts.front().p);
                    updateOrderProductList(list_of_products, faultyProducts.front());

                    sensors.clearFaultyProducts();
                    sensors.setFaultyPartDetectedFlag(false);
                }
                ros::param::set("/activate_quality", false);

                gantry.placePartRightArm(); // Place product of right arm in agv

                //Check for faulty product
                ros::Duration(1).sleep();
                ros::param::set("/activate_quality", true);
                ros::Duration(1).sleep();

                if (sensors.isFaultyPartDetected())
                {
                    std::vector<Product> faultyProducts = sensors.getFaultyProducts();

                    gantry.throwLastPartRight(faultyProducts.front().p);

                    updateOrderProductList(list_of_products, faultyProducts.front());

                    sensors.clearFaultyProducts();
                    sensors.setFaultyPartDetectedFlag(false);
                }

                ros::param::set("/activate_quality", false);

                ros::param::set("/check_flipped", true);
                ros::Duration(2).sleep();

                gantry.getProductsToFlip(sensors.getPartsToFlip());
                gantry.flipProductsAGV();
                sensors.clearPartsToFlip();

                gantry.goToPresetLocation(gantry.start_); // go back to start position
            }

            if (p < list_of_products.size())        // get product not called in last iteration
                gantry.getProduct(current_product); // get product after placing in agv
        }
    }

    // Place in agv the two last retrieved products
    if (gantry.checkFreeGripper().compare("none") == 0 || gantry.checkFreeGripper().compare("right") == 0) // if none of the grippers are free place both products in grippers
    {
        if (gantry.getGantryLocation().compare("aisle_1") == 0) // go to start location from current gantry location
        {
            gantry.goToPresetLocation(gantry.aisle1_);
        }
        else if (gantry.getGantryLocation().compare("aisle_2") == 0)
        {
            gantry.goToPresetLocation(gantry.aisle2_);
        }

        gantry.goToPresetLocation(gantry.start_);

        gantry.placePartLeftArm(); // Place product of left arm in agv

        //Check for faulty product
        ros::Duration(1).sleep();
        ros::param::set("/activate_quality", true);
        if (sensors.isFaultyPartDetected())
        {
            std::vector<Product> faultyProducts = sensors.getFaultyProducts();
            ROS_WARN_STREAM("FAULTY PARTS :" << faultyProducts[0].p.location);

            gantry.throwLastPartLeft(faultyProducts.front().p);
            updateOrderProductList(list_of_products, faultyProducts.front());

            sensors.clearFaultyProducts();
            sensors.setFaultyPartDetectedFlag(false);
        }
        ros::param::set("/activate_quality", false);

        ROS_WARN_STREAM("STARTING PLACE PART RIGHT");
        std::string free_gripper = gantry.checkFreeGripper();
        if (free_gripper.compare("right") != 0)
        {
            ROS_WARN_STREAM("PLACE PART RIGHT");
            gantry.placePartRightArm(); // Place product of right arm in agv

            //Check for faulty product
            ros::Duration(1).sleep();
            ros::param::set("/activate_quality", true);
            if (sensors.isFaultyPartDetected())
            {
                std::vector<Product> faultyProducts = sensors.getFaultyProducts();

                gantry.throwLastPartRight(faultyProducts.front().p);

                updateOrderProductList(list_of_products, faultyProducts.front());

                sensors.clearFaultyProducts();
                sensors.setFaultyPartDetectedFlag(false);
            }
            ros::param::set("/activate_quality", false);
        } else {
            ros::param::set("/no_prod_right", true);
        }
        
        ros::param::set("/check_flipped", true);
        ros::Duration(1).sleep();
    
        if (sensors.getPartsToFlip().empty() != 1)
        {
            gantry.getProductsToFlip(sensors.getPartsToFlip());
            gantry.flipProductsAGV();
            sensors.clearPartsToFlip();
        }
        gantry.goToPresetLocation(gantry.start_); // go back to start position
    }

    ROS_WARN_STREAM("COMPLETED TEST, NOW SEND AGV");
    if (agvControl.isAGVReady(AGV1_TRAY))
    {
        std::string shipmentType = comp.agvToShipmentMap.at(AGV1_ID).front();
        comp.agvToShipmentMap.at(AGV1_ID).pop();
        agvControl.sendAGV(shipmentType, AGV1_TRAY);
    }

    if (agvControl.isAGVReady(AGV2_TRAY))
    {
        std::string shipmentType = comp.agvToShipmentMap.at(AGV2_ID).front();
        comp.agvToShipmentMap.at(AGV2_ID).pop();
        agvControl.sendAGV(shipmentType, AGV2_TRAY);
    }
    ROS_WARN_STREAM("AGV SEND");
    spinner.stop();
    ros::shutdown();
    return 0;
}