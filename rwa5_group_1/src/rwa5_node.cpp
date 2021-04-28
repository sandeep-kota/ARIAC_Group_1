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
 * @brief Check for sensor blackout 
 * 
 * @param sensorNum Logical Camera ID : 0 or 1
 * @param numProductsToBeSent Product size from orders
 * @param sensors Sensor control object
 * @return true 
 * @return false 
 */
bool checkBlackout(int sensorNum, int numProductsToBeSent, SensorControl &sensors) {
    ros::param::set(ACTIVATE_LOG_CAM, sensorNum);
    ros::Duration(1).sleep();
    ros::param::set(ACTIVATE_LOG_CAM, -1);
    // ROS_INFO_STREAM("Checking the products placed on AGV.");
    int numProductsDetected = sensors.getLogicalCameraNumProducts(sensorNum);
    // ROS_INFO_STREAM("Number of parts detected :" << numProductsDetected);
    
    if (numProductsDetected == 0) {
        
        ROS_INFO_STREAM("Number of products mismatched := Products on AGV : " << numProductsDetected << ", Expected : " << numProductsToBeSent );
        return true;
    }
    return false;
}

/**
 * @brief function that processes the flow when a faulty part is detected
 * 
 * @param gantry gantry class passed by reference
 * @param sensors sensor class passed by reference
 * @param arm arm that placed the detected faulty part
 */
void faultyPartsProcess(GantryControl &gantry, SensorControl &sensors)
{
    sensors.resetLogicCallQuality();
    //Check for faulty product
    ros::Duration(1).sleep();
    ros::param::set("/activate_quality", true);
    ros::Duration(1).sleep();
    ros::param::set("/activate_quality", false);

    bool activate_quality;
    bool get_product_from_conveyor{false};
    bool pickedConveyor;
    double time{0.0};
    Product repick_product;
    std::vector<Part> partsConveyor;
    std::vector<Part> faultyParts{};
    std::vector<Product> products_tray_1;
    std::vector<Product> products_tray_2;
    std::vector<Product> faultyProducts{};
    double d_x;
    double d_y;

    // ROS_WARN_STREAM("FAULTY PRODUCTS DETECTED: " << sensors.isFaultyPartDetected());
    // ROS_WARN_STREAM("FAULTY PARTD LIST SIZE: " << faultyParts.size());
    // ROS_WARN_STREAM("FAULTY PRODUCTS LIST SIZE: " << faultyProducts.size());
    faultyParts = sensors.getFaultyParts();
    while (faultyParts.size() > 0)
    {
        ROS_WARN_STREAM("PROCESS FAULTY PART");
        faultyParts = sensors.getFaultyParts();
        products_tray_1 = gantry.getProductsTray1();
        products_tray_2 = gantry.getProductsTray2();
        ROS_WARN_STREAM("SIZE FAULTY PARTS DETECTED: " << faultyParts.size());

        // create a faulty product list with type and id from quality sensor data
        for (int f = 0; f < faultyParts.size(); f++)
        {
            if (faultyParts.at(f).location == "agv_1")
            {
                for (int p = 0; p < products_tray_1.size(); p++)
                {
                    d_x = abs(faultyParts.at(f).pose.position.x - products_tray_1.at(p).p.pose.position.x);
                    d_y = abs(faultyParts.at(f).pose.position.y - products_tray_1.at(p).p.pose.position.y);
                
                    if (d_x < 0.1 && d_y < 0.1)
                    {
                        faultyProducts.push_back(products_tray_1.at(p));
                        ROS_WARN_STREAM("FAULTY PRODUCT DETECT TRAY 1: " << products_tray_1.at(p).type);
                        break;
                    }
                }
            }
            else
            {
                for (int p = 0; p < products_tray_2.size(); p++)
                {
                    d_x = abs(faultyParts.at(f).pose.position.x - products_tray_2.at(p).p.pose.position.x);
                    d_y = abs(faultyParts.at(f).pose.position.y - products_tray_2.at(p).p.pose.position.y);

                    if (d_x < 0.1 && d_y < 0.1)
                    {
                        faultyProducts.push_back(products_tray_2.at(p));
                        ROS_WARN_STREAM("FAULTY PRODUCT DETECT TRAY 2: " << products_tray_2.at(p).type);
                        break;
                    }
                }
            }
        }

        ROS_WARN_STREAM("SIZE FAULTY PRODUCTS TO BE REMOVED: " << faultyProducts.size());

        for (int i = 0; i < faultyProducts.size(); i += 2)
        {
            if ((i + 1) >= faultyProducts.size())
            {
                //end of the list only pick and replace with left arm
                gantry.throwPartLeft(faultyProducts.at(i).p); // update parts in tray vectors
                
            }
            else
            {
                //both arms
                gantry.throwPartLeft(faultyProducts.at(i).p);
                gantry.throwPartLeft(faultyProducts.at(i + 1).p);
            }

            gantry.goToPresetLocation(gantry.start_90_); // go to start location
            gantry.setGantryLocation("start_90");

            faultyProducts.at(i).p = sensors.findPart(faultyProducts.at(i).type); // find a new part in the env o replace

            if (faultyProducts.at(i).p.type.empty()) // no parts of desired product found go to conveyor
            {
                get_product_from_conveyor = true;
            }
            else
            {
                get_product_from_conveyor = false;
            }
            // get product in the env
            if (get_product_from_conveyor)
            {
                double startig_time = ros::Time::now().toSec();
                while (time <= 120)
                {
                    partsConveyor = sensors.getPartsConveyor();
                    if (partsConveyor.empty() != 1)
                    {
                        double original_y;
                        for (int prt = 0; prt < partsConveyor.size(); prt++)
                        {
                            original_y = partsConveyor.at(prt).pose.position.y - 0.2 * (ros::Time::now().toSec() - partsConveyor.at(prt).time_stamp.toSec());
                            if (partsConveyor.at(prt).type.compare(faultyProducts.at(i).type) == 0 && original_y >= 3)
                            {
                                faultyProducts.at(i).p = partsConveyor.at(prt);

                                if (gantry.checkFreeGripper().compare("left") == 0 || gantry.checkFreeGripper().compare("any") == 0)
                                {
                                    pickedConveyor = gantry.getPartConveyorLeftArm(faultyProducts.at(i));
                                }
                                else if (gantry.checkFreeGripper().compare("right") == 0)
                                {
                                    pickedConveyor = gantry.getPartConveyorRightArm(faultyProducts.at(i));
                                }
                                sensors.erasePartConveyor(prt);
                                break;
                            }
                            else if (original_y < 3.0)
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
            }
            else
            {
                gantry.getProduct(faultyProducts.at(i)); // get product after placing in agv
            }

            // ROS_WARN_STREAM("NEW PRODUCT PICKED LEFT ARM");
            if ((i + 1) < faultyProducts.size()) // if the elements in the faulty product list are even
            {
                // ROS_WARN_STREAM("TYPE PART TO REPLACE: " << faultyProducts.at(i + 1).p.type);
                faultyProducts.at(i + 1).p = sensors.findPart(faultyProducts.at(i + 1).type); // find a new part in the env to replace
                // ROS_WARN_STREAM("TYPE PART TO REPLACE: " << faultyProducts.at(i + 1).p.type);

                if (faultyProducts.at(i + 1).p.type.empty()) // no parts of desired product found go to conveyor
                {
                    get_product_from_conveyor = true;
                }
                else
                {
                    get_product_from_conveyor = false;
                    // ROS_WARN_STREAM("GO PICK PART RICHT ARM");
                }
                // get product in the env
                if (get_product_from_conveyor)
                {
                    double startig_time = ros::Time::now().toSec();
                    while (time <= 120)
                    {
                        partsConveyor = sensors.getPartsConveyor();
                        if (partsConveyor.empty() != 1)
                        {
                            double original_y;
                            for (int prt = 0; prt < partsConveyor.size(); prt++)
                            {
                                original_y = partsConveyor.at(prt).pose.position.y - 0.2 * (ros::Time::now().toSec() - partsConveyor.at(prt).time_stamp.toSec());
                                if (partsConveyor.at(prt).type.compare(faultyProducts.at(i + 1).type) == 0 && original_y >= 3)
                                {
                                    faultyProducts.at(i + 1).p = partsConveyor.at(prt);

                                    if (gantry.checkFreeGripper().compare("left") == 0 || gantry.checkFreeGripper().compare("any") == 0)
                                    {
                                        pickedConveyor = gantry.getPartConveyorLeftArm(faultyProducts.at(i + 1));
                                    }
                                    else if (gantry.checkFreeGripper().compare("right") == 0)
                                    {
                                        pickedConveyor = gantry.getPartConveyorRightArm(faultyProducts.at(i + 1));
                                    }
                                    sensors.erasePartConveyor(prt);
                                    break;
                                }
                                else if (original_y < 3.0)
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
                }
                else
                {
                    // ROS_WARN_STREAM("GO PICK PART RICHT ARM");
                    gantry.getProduct(faultyProducts.at(i + 1)); // get product after placing in agv
                }
            }

            // TODO Account for the new getProduct() @SANDEEP
            // if (gantry.getGantryLocation().compare("aisle_1") == 0) // go to start location from current gantry location
            // {
            //     gantry.goToPresetLocation(gantry.aisle1_);
            // }
            // else if (gantry.getGantryLocation().compare("aisle_2") == 0)
            // {
            //     gantry.goToPresetLocation(gantry.aisle2_);
            // }

            // gantry.goToPresetLocation(gantry.start_90_);

            gantry.placePartLeftArm();

            if ((i + 1) < faultyProducts.size()) // if the elements in the faulty product list are even
            {
                gantry.placePartRightArm();
            }
        }

        // empty all faulty product vectors to search for new
        if (faultyParts.empty() != 1)
        {
            faultyParts.clear();
        }
        if (faultyProducts.empty() != 1)
        {
            faultyProducts.clear();
        }
        sensors.clearFaultyProducts();
        sensors.resetLogicCallQuality();
        ros::param::set("/activate_quality", true);
        ros::Duration(1).sleep();
        ros::param::set("/activate_quality", false);
        faultyParts = sensors.getFaultyParts();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rwa5_node");
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
    ros::param::set("/update_agv_parts", false);
    ros::param::set("/check_parts_to_flip", false);

    std::array<std::array<std::vector<part>, 3>, 5> p_agv1;
    std::array<std::array<std::vector<part>, 3>, 5> p_agv2;
    gantry.goToPresetLocation(gantry.start_); // start the trial from start position

    AGVControl agvControl(node);

    std::vector<Part> partsConveyor;
    double startig_time = ros::Time::now().toSec();
    double time{0.0};
    bool get_product_from_conveyor{false};
    bool pickedConveyor;

    bool transitionDone = false;

    while (true) {
        while (comp.areOrdersLeft() && sensors.read_all_sensors_) //--1-Read order until no more found
        {

            while (!agvControl.isAGVReady(AGV1_TRAY) && !agvControl.isAGVReady(AGV2_TRAY));
            
            // ROS_WARN_STREAM("Starting building kit for the new order");
            list_of_shipments = comp.get_shipment_list(); // get list of shipments of current order in priority order
            //list_of_products = comp.get_product_list();   // get list of products of current order in priority order

            int currentOrderCount = comp.getTotalReceivedOrdersCount();
            comp.setNewOrderAlert(false);
            transitionDone = false;

            Shipment current_shipment;
            for (int n_ship = 0; n_ship < list_of_shipments.size() && !comp.newOrderAlert(); n_ship++)
            {
                current_shipment = list_of_shipments.at(n_ship);
                comp.setAgvInUse(current_shipment.agv_id);
                // for (auto& current_shipment: list_of_shipments) {
                list_of_products = comp.get_product_list_from_shipment(current_shipment);

                for (int p = 0; p < list_of_products.size(); p++) // loop all the products to be retrieve from current order
                {

                    // ROS_WARN_STREAM("Picking next product: " << p + 1 << "/" << list_of_products.size() << " for AGV: " << list_of_products.at(p).agv_id);
                    if (currentOrderCount != comp.getTotalReceivedOrdersCount())
                    {
                        // ROS_WARN_STREAM("New Order Received. We need to stop current assembly.");
                        /** new high priority order received;
                         *  need to stop the current process and process the new order
                         *  take care of the pending products
                         *  break the current iteration
                         *  
                         */

                        comp.orderTransition(list_of_shipments, sensors, gantry);
                        comp.setNewOrderAlert(true);
                        transitionDone = true;

                        break;
                    }

                    current_product = list_of_products.at(p);
                    current_product.agv_id = comp.getAgvInUse();

                    ROS_WARN_STREAM(current_product.type);

                    current_product.p = sensors.findPart(current_product.type); //--2-Look for parts in this order

                    ROS_WARN_STREAM(current_product.p.location);

                    if (current_product.p.type.empty()) // no parts of desired product found go to conveyor
                    {
                        get_product_from_conveyor = true;
                        ROS_WARN_STREAM("PRODUCT NOT FOUND");
                    }
                    else
                    {
                        get_product_from_conveyor = false;
                    }

                    ROS_WARN_STREAM("GET PRODICT FROM CONVEYOR: " << get_product_from_conveyor);

                    if (gantry.checkFreeGripper().compare("none") == 0 && !comp.newOrderAlert()) // if none of the grippers are free place both products in trays
                    {
                        // if (gantry.getGantryLocation().compare("aisle_1") == 0) // go to start location from current gantry location
                        // {
                        //     gantry.goToPresetLocation(gantry.aisle1_);
                        // }
                        // else if (gantry.getGantryLocation().compare("aisle_2") == 0)
                        // {
                        //     gantry.goToPresetLocation(gantry.aisle2_);
                        // }

                        // gantry.goToPresetLocation(gantry.start_);

                        gantry.placePartLeftArm(); // Place product of left arm in agv

                        // TODO: @Sandeep, check the part oriention for the left arm
                        ////!------ Check part orientation Left with order -------!
                        // Get parts from AGV logical cameras sensors

                        // ros::Duration(1).sleep(); // Delay to update the parts list
                        // ros::param::set("/update_agv_parts", true);
                        // ros::Duration(1).sleep(); // Delay to update the parts list
                        // ros::param::set("/update_agv_parts", false);
                        // ros::Duration(1).sleep(); // Delay to update the parts list

                        // Part target_part;
                        // target_part.type = gantry.getProductLeftArm().type;
                        // target_part.pose = gantry.getProductLeftArm().pose;

                        // tf2_ros::Buffer tfBuffer;
                        // tf2_ros::TransformListener tfListener(tfBuffer);
                        // ros::Duration timeout(1.0);

                        // if (gantry.getProductLeftArm().agv_id == "agv1")
                        // {
                        //     geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("world", "kit_tray_1", ros::Time(0), timeout);
                        //     tf2::doTransform(target_part.pose, target_part.pose, transformStamped);
                        // }

                        // else if (gantry.getProductLeftArm().agv_id == "agv2")
                        // {
                        //     geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("world", "kit_tray_2", ros::Time(0), timeout);
                        //     tf2::doTransform(target_part.pose, target_part.pose, transformStamped);
                        // }

                        // ROS_WARN_STREAM("PRINT TARGET PART: " << target_part.type);

                        // if (sensors.isPartPoseAGVCorrect(target_part, gantry.getProductLeftArm().agv_id) == false)
                        // {
                        //     if (gantry.getProductLeftArm().agv_id == "agv1")
                        //     {
                        //         gantry.pickPartFromTrayLeftArm(sensors.incorrect_part_agv1, gantry.getProductLeftArm().agv_id);
                        //         ROS_WARN_STREAM("PICK PART TRAY INCORRECT");
                        //         gantry.correctPosePartLeftArm(sensors.incorrect_part_agv1, target_part.pose, gantry.getProductLeftArm().agv_id);
                        //         gantry.goToPresetLocation(gantry.agv1_);
                        //         ros::Duration(0.5).sleep(); // Delay to update the parts list
                        //         ros::param::set("/update_agv_parts", true);
                        //         ros::Duration(0.5).sleep(); // Delay to update the parts list
                        //         ros::param::set("/update_agv_parts", false);
                        //         ros::Duration(0.5).sleep(); // Delay to update the parts list
                        //         sensors.isPartPoseAGVCorrect(target_part, gantry.getProductLeftArm().agv_id);
                        //         ROS_WARN_STREAM("FINISH REPOSSITION");
                        //     }
                        //     else if (gantry.getProductLeftArm().agv_id == "agv2")
                        //     {
                        //         gantry.pickPartFromTrayLeftArm(sensors.incorrect_part_agv2, gantry.getProductLeftArm().agv_id);
                        //         gantry.correctPosePartLeftArm(sensors.incorrect_part_agv2, target_part.pose, gantry.getProductLeftArm().agv_id);
                        //         gantry.goToPresetLocation(gantry.agv2_);
                        //         ros::param::set("/update_agv_parts", true);
                        //         ros::Duration(0.1).sleep(); // Delay to update the parts list
                        //         ros::param::set("/update_agv_parts", false);
                        //         sensors.isPartPoseAGVCorrect(target_part, gantry.getProductLeftArm().agv_id);
                        //     }
                        // }
                        gantry.placePartRightArm();
                

                        // Place product of right arm in agv
                        ////!------ Check part orientation Rgght with order -------!
                        // Get parts from AGV logical cameras sensors
                        // ros::Duration(1).sleep(); // Delay to update the parts list
                        // ros::param::set("/update_agv_parts", true);
                        // ros::Duration(1).sleep(); // Delay to update the parts list
                        // ros::param::set("/update_agv_parts", false);
                        // ros::Duration(1).sleep(); // Delay to update the parts list

                        // if (sensors.isPartPoseAGVCorrect(target_part, gantry.getProductRightArm().agv_id) == false)
                        // {
                        //     if (gantry.getProductRightArm().agv_id == "agv1")
                        //     {
                        //         gantry.pickPartFromTrayRightArm(sensors.incorrect_part_agv1, gantry.getProductRightArm().agv_id);
                        //         gantry.correctPosePartRightArm(sensors.incorrect_part_agv1, target_part.pose, gantry.getProductRightArm().agv_id);
                        //         gantry.goToPresetLocation(gantry.agv1_);
                        //         ros::Duration(0.5).sleep(); // Delay to update the parts list
                        //         ros::param::set("/update_agv_parts", true);
                        //         ros::Duration(0.5).sleep(); // Delay to update the parts list
                        //         ros::param::set("/update_agv_parts", false);
                        //         ros::Duration(0.5).sleep(); // Delay to update the parts list
                        //         sensors.isPartPoseAGVCorrect(target_part, gantry.getProductRightArm().agv_id);
                        //     }
                        //     else if (gantry.getProductRightArm().agv_id == "agv2")
                        //     {
                        //         gantry.pickPartFromTrayRightArm(sensors.incorrect_part_agv2, gantry.getProductRightArm().agv_id);
                        //         gantry.correctPosePartRightArm(sensors.incorrect_part_agv2, target_part.pose, gantry.getProductRightArm().agv_id);
                        //         gantry.goToPresetLocation(gantry.agv2_);
                        //         ros::param::set("/update_agv_parts", true);
                        //         ros::Duration(0.1).sleep(); // Delay to update the parts list
                        //         ros::param::set("/update_agv_parts", false);
                        //         sensors.isPartPoseAGVCorrect(target_part, gantry.getProductRightArm().agv_id);
                        //     }
                        // }

                        // ros::param::set("/check_flipped", true);
                        // ros::Duration(2).sleep();

                        // gantry.getProductsToFlip(sensors.getPartsToFlip());
                        faultyPartsProcess(gantry, sensors);
                        ros::param::set("/check_parts_to_flip", true);
                        ros::Duration(1).sleep();
                        gantry.flipProductsAGV(sensors.getcheckPartsToFlip());
                        sensors.clearcheckPartsToFlip();
                        // sensors.clearPartsToFlip();

                        gantry.goToPresetLocation(gantry.start_90_); // go back to start position

                        // the last two products that were picked have been placed.
                        comp.updateAGVProductMap(list_of_products.at(p - 1).agv_id, list_of_products.at(p - 1));
                        comp.updateAGVProductMap(list_of_products.at(p - 2).agv_id, list_of_products.at(p - 2));

                        ////!------ Check part orientation Rgght with order -------!
                        // Get parts from AGV logical cameras sensors
                        // ros::Duration(1).sleep(); // Delay to update the parts list
                        // ros::param::set("/update_agv_parts", true);
                        // ros::Duration(1).sleep(); // Delay to update the parts list
                        // ros::param::set("/update_agv_parts", false);
                        // ros::Duration(1).sleep(); // Delay to update the parts list

                        // if (sensors.isPartPoseAGVCorrect(target_part, gantry.getProductRightArm().agv_id) == false)
                        // {
                        //     if (gantry.getProductRightArm().agv_id == "agv1")
                        //     {
                        //         gantry.pickPartFromTrayRightArm(sensors.incorrect_part_agv1, gantry.getProductRightArm().agv_id);
                        //         gantry.correctPosePartRightArm(sensors.incorrect_part_agv1, target_part.pose, gantry.getProductRightArm().agv_id);
                        //         gantry.goToPresetLocation(gantry.agv1_);
                        //         ros::Duration(0.5).sleep(); // Delay to update the parts list
                        //         ros::param::set("/update_agv_parts", true);
                        //         ros::Duration(0.5).sleep(); // Delay to update the parts list
                        //         ros::param::set("/update_agv_parts", false);
                        //         ros::Duration(0.5).sleep(); // Delay to update the parts list
                        //         sensors.isPartPoseAGVCorrect(target_part, gantry.getProductRightArm().agv_id);
                        //     }
                        //     else if (gantry.getProductRightArm().agv_id == "agv2")
                        //     {
                        //         gantry.pickPartFromTrayRightArm(sensors.incorrect_part_agv2, gantry.getProductRightArm().agv_id);
                        //         gantry.correctPosePartRightArm(sensors.incorrect_part_agv2, target_part.pose, gantry.getProductRightArm().agv_id);
                        //         gantry.goToPresetLocation(gantry.agv2_);
                        //         ros::param::set("/update_agv_parts", true);
                        //         ros::Duration(0.1).sleep(); // Delay to update the parts list
                        //         ros::param::set("/update_agv_parts", false);
                        //         sensors.isPartPoseAGVCorrect(target_part, gantry.getProductRightArm().agv_id);
                        //     }
                        // }

                        // ros::param::set("/check_flipped", true);
                        // ros::Duration(2).sleep();

                        // gantry.getProductsToFlip(sensors.getPartsToFlip());

                        // faultyPartsProcess(gantry, sensors);
                        // ros::param::set("/check_parts_to_flip", true);
                        // ros::Duration(1).sleep();
                        // gantry.flipProductsAGV(sensors.getcheckPartsToFlip());
                        // sensors.clearcheckPartsToFlip();
                        // // sensors.clearPartsToFlip();

                        // gantry.goToPresetLocation(gantry.start_90_); // go back to start position

                        // the last two products that were picked have been placed.

                        // comp.updateAGVProductMap(list_of_products.at(p - 1).agv_id, list_of_products.at(p - 1));
                        // comp.updateAGVProductMap(list_of_products.at(p - 2).agv_id, list_of_products.at(p - 2));

                        // ROS_WARN_STREAM("Product placed on AGV: " << list_of_products.at(p - 1).agv_id << " ID");
                    }

                    if (p < list_of_products.size() && !comp.newOrderAlert()) // get product not called in last iteration
                    {
                        if (get_product_from_conveyor)
                        {
                            // gantry.goToPresetLocation(gantry.conveyor_left_);
                            time = 0.0;
                            startig_time = ros::Time::now().toSec();
                            pickedConveyor == 0;
                            while(sensors.conveyorPartsList.size()<1)
                            {
                            }
                            if(sensors.conveyorPartsList.size()>0)
                            {
                                float distance  = sensors.conveyorPartsList.at(0).pose.position.y - (0.2*(ros::Time().now().toSec() - sensors.conveyorPartsTime.at(0)));

                            }
                            while (time <= 120)
                            {
                                partsConveyor = sensors.getPartsConveyor();
                                if (partsConveyor.empty() != 1)
                                {
                                    double original_y;
                                    for (int prt = 0; prt < partsConveyor.size(); prt++)
                                    {
                                        if (partsConveyor.at(prt).type.compare(current_product.type) == 0 && original_y >= 0)
                                        {
                                            current_product.p = partsConveyor.at(prt);

                                            if (gantry.checkFreeGripper().compare("left") == 0 || gantry.checkFreeGripper().compare("any") == 0)
                                            {
                                                pickedConveyor = gantry.getPartConveyorLeftArm(current_product);
                                            }
                                            else if (gantry.checkFreeGripper().compare("right") == 0)
                                            {
                                                pickedConveyor = gantry.getPartConveyorRightArm(current_product);
                                            }

                                            sensors.erasePartConveyor(prt);
                                            break;
                                        }
                                        else if (original_y < 0)
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
                            // ROS_WARN_STREAM("Picked from conveyor belt");
                        }
                        else
                        {
                            gantry.getProduct(current_product); // get product after placing in agv
                            // ROS_WARN_STREAM("Picked from a bin or shelf");
                        }
                        // ROS_WARN_STREAM("Picked from conveyor belt");
                    }
                    else if (!comp.newOrderAlert())
                    {
                        gantry.getProduct(current_product); // get product after placing in agv
                        // ROS_WARN_STREAM("Picked from a bin or shelf");
                    }
                }
                // }
                // Place in agv the two last retrieved products
                // this piece of code has been brought inside the while loop
                if (!comp.newOrderAlert())
                {
                    if (gantry.checkFreeGripper().compare("none") == 0 || gantry.checkFreeGripper().compare("right") == 0) // if none of the grippers are free place both products in grippers
                    {
                        // if (gantry.getGantryLocation().compare("aisle_1") == 0) // go to start location from current gantry location
                        // {
                        //     gantry.goToPresetLocation(gantry.aisle1_);
                        // }
                        // else if (gantry.getGantryLocation().compare("aisle_2") == 0)
                        // {
                        //     gantry.goToPresetLocation(gantry.aisle2_);
                        // }

                        // gantry.goToPresetLocation(gantry.start_);

                        gantry.placePartLeftArm(); // Place product of left arm in agv

                        std::string free_gripper = gantry.checkFreeGripper();
                        if (list_of_products.size()%2 == 0)
                        {
                            // ROS_WARN_STREAM("LAST PART RIGHT ARM PLACING");
                            gantry.placePartRightArm(); // Place product of right arm in agv
                        }
                        else
                        {
                            ros::param::set("/no_prod_right", true);
                        }

                        // ros::param::set("/check_flipped", true);
                        // ros::Duration(1).sleep();

                        // if (sensors.getPartsToFlip().empty() != 1)
                        // {
                        //     // gantry.getProductsToFlip(sensors.getPartsToFlip());
                        ROS_WARN_STREAM("FIRST FAULTY PARTS CALLED");
                        faultyPartsProcess(gantry, sensors);
                        ROS_WARN_STREAM("FIRST FAULTY PARTS FINISHED");
                        ros::param::set("/check_parts_to_flip", true);
                        ros::Duration(1).sleep();
                        gantry.flipProductsAGV(sensors.getcheckPartsToFlip());
                        sensors.clearcheckPartsToFlip();
                        //     sensors.clearPartsToFlip();
                        // }
                        gantry.goToPresetLocation(gantry.start_90_); // go back to start position

                        // the last two products that were picked have been placed.
                        // take the last two from the list itself instead of using index p;
                        int totalProducts = list_of_products.size();
                        comp.updateAGVProductMap(list_of_products.at(totalProducts - 1).agv_id, list_of_products.at(totalProducts - 1));

                        // 2nd last also placed
                        if (totalProducts % 2 == 0)
                        {
                            comp.updateAGVProductMap(list_of_products.at(totalProducts - 2).agv_id, list_of_products.at(totalProducts - 2));
                        }
                    }

                    if (!comp.newOrderAlert())
                    {
                        // time to send the agv:

                        bool blackout;

                        std::string shipmentAGV = current_shipment.agv_id;
                        int sensorNum = sensorNumMap.at(shipmentAGV);
                        
                        blackout = checkBlackout(sensorNum, list_of_products.size(), sensors);
                        if (blackout)
                        {
                            // ROS_WARN_STREAM("Sensor blackout is there. Waiting for it to get over..");
                            // wait till blackout is over
                            while (blackout)
                            {
                                blackout = checkBlackout(sensorNum, list_of_products.size(), sensors);
                            }
                            // ROS_WARN_STREAM("Sensor blackout over..");

                        }
                        
                        // check the faulty parts again and the parts that are to be flipped
                        if (!comp.newOrderAlert()) {
                            ROS_INFO_STREAM("Checking the faulty parts before sending AGV");
                            faultyPartsProcess(gantry, sensors);
                            ros::param::set("/check_parts_to_flip", true);
                            ros::Duration(1).sleep();
                            gantry.flipProductsAGV(sensors.getcheckPartsToFlip());
                            sensors.clearcheckPartsToFlip();

                            
                            std::string shipmentTray = agvTrayMap.at(shipmentAGV);
                            ROS_INFO_STREAM("Product Placed, NOW SEND AGV: " << shipmentAGV << "Tray: " << shipmentTray);

                            if (agvControl.isAGVReady(shipmentTray))
                            {
                                std::string shipmentType = current_shipment.shipment_type;
                                agvControl.sendAGV(shipmentType, shipmentTray);
                            }
                            else
                            {
                                ROS_INFO_STREAM("Tray not ready: " << shipmentTray);
                            }

                            ROS_INFO_STREAM("AGV SENT");
                        }
                    }
                }

                if (comp.newOrderAlert() && !transitionDone)
                {
                    // ROS_WARN_STREAM("New Order Received. We need to stop current assembly.");
                    /** new high priority order received;
                     *  need to stop the current process and process the new order
                     *  take care of the pending products
                     *  break the current iteration
                     *  
                     */
                    comp.orderTransition(list_of_shipments, sensors, gantry);
                }
            }
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
