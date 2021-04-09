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
 * @brief 
 * 
 * @param gantry 
 * @param sensors 
 * @param arm 
 */
void faultyPartsProcess(GantryControl &gantry, SensorControl &sensors, std::string arm)
{
    //Check for faulty product
    ros::Duration(1).sleep();
    ros::param::set("/activate_quality", true);
    ros::Duration(1).sleep();

    bool activate_quality;
    bool get_product_from_conveyor {false};
    bool pickedConveyor;
    double time {0.0};
    Product repick_product;
    Product initial_product_left_arm = gantry.getProductLeftArm();
    Product initial_product_right_arm = gantry.getProductRightArm();
    std::vector<Part> partsConveyor;

    ros::param::get("/activate_quality", activate_quality);
    ROS_WARN_STREAM("ACTIVATE QUALITY: " << activate_quality);
    while (sensors.isFaultyPartDetected())
    {
        std::vector<Product> faultyProducts = sensors.getFaultyProducts();
        ROS_WARN_STREAM("FAULTY PARTS :" << faultyProducts[0].p.location);
        if (arm.compare("left") == 0)
        {
            gantry.throwLastPartLeft(faultyProducts.front().p);
        } else
        {
            gantry.throwLastPartRight(faultyProducts.front().p);
        }
        

        sensors.clearFaultyProducts();
        sensors.setFaultyPartDetectedFlag(false);
        ros::param::set("/activate_quality", false);

        gantry.goToPresetLocation(gantry.start_); // go to start location
        gantry.setGantryLocation("start");

        if (arm.compare("left") == 0)
        {
            repick_product = gantry.getProductLeftArm();
            repick_product.p = sensors.findPart(gantry.getProductLeftArm().type); // find new part in the env
        } else 
        {
            repick_product = gantry.getProductRightArm();
            repick_product.p = sensors.findPart(gantry.getProductRightArm().type); // find new part in the env
        }
        

        if (repick_product.p.type.empty()) // no parts of desired product found go to conveyor
        {
            get_product_from_conveyor = true;
            ROS_WARN_STREAM("GET PART CONVEYOR");
        } else {
            get_product_from_conveyor = false;
        }
        // get product in the env
        if (get_product_from_conveyor)
        {
            double startig_time = ros::Time::now().toSec();
            while(time <= 120)
            {
                partsConveyor = sensors.getPartsConveyor();
                if (partsConveyor.empty() != 1)
                {
                    double original_y;
                    for (int prt = 0; prt < partsConveyor.size(); prt++)
                    {
                        original_y = partsConveyor.at(prt).pose.position.y - 0.2*(ros::Time::now().toSec() - partsConveyor.at(prt).time_stamp.toSec());
                        if (partsConveyor.at(prt).type.compare(repick_product.type) == 0 && original_y >= 3)
                        {
                            repick_product.p = partsConveyor.at(prt);

                            if (gantry.checkFreeGripper().compare("left") == 0 || gantry.checkFreeGripper().compare("any") == 0)
                            {
                                pickedConveyor = gantry.getPartConveyorLeftArm(repick_product);
                            } else if (gantry.checkFreeGripper().compare("right") == 0){
                                pickedConveyor = gantry.getPartConveyorRightArm(repick_product);
                            }
                                sensors.erasePartConveyor(prt);
                                break;
                        } else if (original_y < 3.0)
                        {
                            sensors.erasePartConveyor(prt);
                        }

                    }
                    if (pickedConveyor == 1)
                    {
                        break;
                    }
                 time = ros::Time::now().toSec() - startig_time;                   
                }
            }
        } else 
        {
            ROS_WARN_STREAM("GO TO GET NEW GASKET");
            gantry.getProduct(repick_product); // get product after placing in agv
        }

        if (gantry.getGantryLocation().compare("aisle_1") == 0) // go to start location from current gantry location
        {
            gantry.goToPresetLocation(gantry.aisle1_);
        } else if (gantry.getGantryLocation().compare("aisle_2") == 0)
        {
            gantry.goToPresetLocation(gantry.aisle2_);
        }

        gantry.goToPresetLocation(gantry.start_);

        gantry.placePartLeftArm(); // Place product of left arm in agv

        ros::Duration(1).sleep();
        ros::param::set("/activate_quality", true);
        ros::Duration(1).sleep();

    }
        ros::param::set("/activate_quality", false);

    gantry.set_product_left_arm_(initial_product_left_arm);
    gantry.set_product_right_arm_(initial_product_right_arm);
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
    ros::param::set("/ariac/new_part_conveyor", false);

    gantry.goToPresetLocation(gantry.start_); // start the trial from start position

    AGVControl agvControl(node);

    std::vector<Part> partsConveyor;
    double startig_time = ros::Time::now().toSec();
    double time {0.0};
    bool get_product_from_conveyor {false};
    bool pickedConveyor;

    bool transitionDone = false;
    
    while (comp.areOrdersLeft() && sensors.read_all_sensors_) //--1-Read order until no more found
    {
        
        ROS_WARN_STREAM("Starting building kit for the new order");
        list_of_shipments = comp.get_shipment_list(); // get list of shipments of current order in priority order
        list_of_products = comp.get_product_list();   // get list of products of current order in priority order

        int currentOrderCount = comp.getTotalReceivedOrdersCount();
        comp.setNewOrderAlert(false);
        transitionDone = false;

        for (int p = 0; p < list_of_products.size(); p++) // loop all the products to be retrieve from current order
        {

            ROS_WARN_STREAM("Picking next product: " << p+1 << "/" << list_of_products.size() << " for AGV: " << list_of_products.at(p).agv_id);
            if (currentOrderCount != comp.getTotalReceivedOrdersCount()) {
                ROS_WARN_STREAM("New Order Received. We need to stop current assembly.");
                /** new high priority order received;
                 *  need to stop the current process and process the new order
                 *  take care of the pending products
                 *  break the current iteration
                 *  
                 */
                
                comp.orderTransition(list_of_shipments, gantry);
                comp.setNewOrderAlert(true);
                transitionDone = true;
                
                break;

            }

            current_product = list_of_products.at(p);

            ROS_WARN_STREAM(current_product.type);

            current_product.p = sensors.findPart(current_product.type); //--2-Look for parts in this order

            ROS_WARN_STREAM(current_product.p.location);

            if (current_product.p.type.empty()) // no parts of desired product found
            {
                get_product_from_conveyor = true;
            } else {
                get_product_from_conveyor = false;
            }

            if (gantry.checkFreeGripper().compare("none") == 0 && !comp.newOrderAlert()) // if none of the grippers are free place both products in trays
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

                faultyPartsProcess(gantry, sensors, "left");

                // TODO: @Sandeep, check the part oriention for the left arm 
                gantry.placePartRightArm(); // Place product of right arm in agv
                
                faultyPartsProcess(gantry, sensors, "right");

                ros::param::set("/check_flipped", true);
                ros::Duration(2).sleep();

                gantry.getProductsToFlip(sensors.getPartsToFlip());
                gantry.flipProductsAGV();
                sensors.clearPartsToFlip();

                gantry.goToPresetLocation(gantry.start_); // go back to start position
                
                // the last two products that were picked have been placed.
                
                comp.updateAGVProductMap(list_of_products.at(p-1).agv_id, list_of_products.at(p-1));
                comp.updateAGVProductMap(list_of_products.at(p-2).agv_id, list_of_products.at(p-2));

                ROS_WARN_STREAM("Product placed on AGV: " << list_of_products.at(p-1).agv_id << " ID");
            }

            if (p < list_of_products.size() && !comp.newOrderAlert())        // get product not called in last iteration
            {
                if (get_product_from_conveyor)
                {
                    time = 0.0;
                    while(time <= 120)
                    {
                        partsConveyor = sensors.getPartsConveyor();
                        if (partsConveyor.empty() != 1)
                        {
                            double original_y;
                            for (int prt = 0; prt < partsConveyor.size(); prt++)
                            {
                                original_y = partsConveyor.at(prt).pose.position.y - 0.2*(ros::Time::now().toSec() - partsConveyor.at(prt).time_stamp.toSec());
                                if (partsConveyor.at(prt).type.compare(current_product.type) == 0 && original_y >= 3)
                                {
                                    current_product.p = partsConveyor.at(prt);

                                    if (gantry.checkFreeGripper().compare("left") == 0 || gantry.checkFreeGripper().compare("any") == 0)
                                    {
                                        pickedConveyor = gantry.getPartConveyorLeftArm(current_product);
                                    } else if (gantry.checkFreeGripper().compare("right") == 0){
                                        pickedConveyor = gantry.getPartConveyorRightArm(current_product);
                                    }
                                    
                                    sensors.erasePartConveyor(prt);
                                    break;
                                } else if (original_y < 3)
                                {
                                    sensors.erasePartConveyor(prt);
                                }
                            }
                            if (pickedConveyor == 1)
                            {
                                break;
                            }
                        }
                        time = ros::Time::now().toSec() - startig_time;
                    }
                    ROS_WARN_STREAM("Picked from conveyor belt");
                } else {
                    gantry.getProduct(current_product); // get product after placing in agv
                    ROS_WARN_STREAM("Picked from a bin or shelf");
                }
            }        
        }
        // Place in agv the two last retrieved products
        // this piece of code has been brought inside the while loop
        if (!comp.newOrderAlert()) {
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

                faultyPartsProcess(gantry, sensors, "left");

                std::string free_gripper = gantry.checkFreeGripper();
                if (free_gripper.compare("right") != 0)
                {
                    gantry.placePartRightArm(); // Place product of right arm in agv

                    faultyPartsProcess(gantry, sensors, "right");
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

                // the last two products that were picked have been placed.
                // take the last two from the list itself instead of using index p;
                int totalProducts = list_of_products.size();
                comp.updateAGVProductMap(list_of_products.at(totalProducts-1).agv_id, list_of_products.at(totalProducts-1));
                
                // 2nd last also placed
                if (totalProducts % 2 == 0) {
                    comp.updateAGVProductMap(list_of_products.at(totalProducts-2).agv_id, list_of_products.at(totalProducts-2));
                }

            }

            if (!comp.newOrderAlert()) {
                // time to send the agv:
                ROS_INFO_STREAM("Product Placed, NOW SEND AGV");
                
                std::string shipmentAGV = list_of_shipments.at(0).agv_id; // will work for rwa4 only - one order-one shipment
                std::string shipmentTray = agvTrayMap.at(shipmentAGV);
                ROS_INFO_STREAM("Product Placed, NOW SEND AGV: " << shipmentAGV << "Tray: " << shipmentTray);

                if (agvControl.isAGVReady(shipmentTray))
                {
                    std::string shipmentType = list_of_shipments.at(0).shipment_type;
                    agvControl.sendAGV(shipmentType, shipmentTray);
                }
                else {
                    ROS_INFO_STREAM("Tray not ready: " << shipmentTray);
                }

                ROS_INFO_STREAM("AGV SENT");
            }
            

        }
        if (comp.newOrderAlert() && !transitionDone) {
            ROS_WARN_STREAM("New Order Received. We need to stop current assembly.");
            /** new high priority order received;
             *  need to stop the current process and process the new order
             *  take care of the pending products
             *  break the current iteration
             *  
             */
            comp.orderTransition(list_of_shipments, gantry);
        }
        
    }

    // Not needed..
    /**
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

        faultyPartsProcess(gantry, sensors, "left");

        std::string free_gripper = gantry.checkFreeGripper();
        if (free_gripper.compare("right") != 0)
        {
            gantry.placePartRightArm(); // Place product of right arm in agv

            faultyPartsProcess(gantry, sensors, "right");
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

    */
    spinner.stop();
    ros::shutdown();
    return 0;
}
