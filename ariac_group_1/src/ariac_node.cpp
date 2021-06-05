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

void placePartsEmptyBins(GantryControl &gantry, SensorControl &sensors)
{
    double b_x;
    double b_y;
    std::vector<EmptyBin> emptybins;
    emptybins = sensors.getEmptyBins();
    if (gantry.checkFreeGripper().compare("left") != 0)
    {

        if (emptybins.at(0).empty_locations.at(0) == 0)
        {
            b_x = emptybins.at(0).pose.position.x + 0.15;
            b_y = emptybins.at(0).pose.position.y + 0.15;
            emptybins.at(0).empty_locations.at(0) = true;
        }
        else if (emptybins.at(0).empty_locations.at(1) == 0)
        {
            b_x = emptybins.at(0).pose.position.x + 0.15;
            b_y = emptybins.at(0).pose.position.y - 0.15;
            emptybins.at(0).empty_locations.at(1) = true;
        }
        else if (emptybins.at(0).empty_locations.at(2) == 0)
        {
            b_x = emptybins.at(0).pose.position.x - 0.15;
            b_y = emptybins.at(0).pose.position.y - 0.15;
            emptybins.at(0).empty_locations.at(2) = true;
        }
        else if (emptybins.at(0).empty_locations.at(3) == 0)
        {
            b_x = emptybins.at(0).pose.position.x - 0.15;
            b_y = emptybins.at(0).pose.position.y + 0.15;
            emptybins.at(0).empty_locations.at(3) = true;
            emptybins.erase(emptybins.begin() + 0);
        }

        gantry.product_left_arm_.pose.position.x = b_x;
        gantry.product_left_arm_.pose.position.y = b_y;

        gantry.placeProductLeftArmBin();
        sensors.updatePartDataStruct(gantry.product_left_arm_);
    }

    if (gantry.checkFreeGripper().compare("right") != 0)
    {

        if (emptybins.at(0).empty_locations.at(0) == 0)
        {
            b_x = emptybins.at(0).pose.position.x + 0.15;
            b_y = emptybins.at(0).pose.position.y + 0.15;
            emptybins.at(0).empty_locations.at(0) = true;
        }
        else if (emptybins.at(0).empty_locations.at(1) == 0)
        {
            b_x = emptybins.at(0).pose.position.x + 0.15;
            b_y = emptybins.at(0).pose.position.y - 0.15;
            emptybins.at(0).empty_locations.at(1) = true;
        }
        else if (emptybins.at(0).empty_locations.at(2) == 0)
        {
            b_x = emptybins.at(0).pose.position.x - 0.15;
            b_y = emptybins.at(0).pose.position.y - 0.15;
            emptybins.at(0).empty_locations.at(2) = true;
        }
        else if (emptybins.at(0).empty_locations.at(3) == 0)
        {
            b_x = emptybins.at(0).pose.position.x - 0.15;
            b_y = emptybins.at(0).pose.position.y + 0.15;
            emptybins.at(0).empty_locations.at(3) = true;
            emptybins.erase(emptybins.begin() + 0);
        }

        gantry.product_right_arm_.pose.position.x = b_x;
        gantry.product_right_arm_.pose.position.y = b_y;

        gantry.placeProductRightArmBin();
        sensors.updatePartDataStruct(gantry.product_right_arm_);
    }

    sensors.updateEmptyBins(emptybins);
}

void correctPose(GantryControl &gantry, SensorControl &sensors)
{
    sensors.resetLogicCallTray();
    ros::Duration(1).sleep();
    ros::param::set("/update_tray_parts", true);
    ros::Duration(1).sleep();
    ros::param::set("/update_tray_parts", false);

    std::vector<Part> parts_tray_1;  // parts in tray 1 detected by logical sensor
    std::vector<Part> parts_tray_2;  // parts in tray 2 detected by logical sensor
    std::vector<Product> products_tray_1; // parts in tray 1 saved when placing
    std::vector<Product> products_tray_2;   // parts in tray 2 saved when placing

    std::vector<Part> incorrect_pose_tray_1;
    std::vector<Part> incorrect_pose_tray_2;
    std::vector<Part> correct_pose_tray_1;
    std::vector<Part> correct_pose_tray_2;
    std::vector<Part> possible_pose_tray_1;
    std::vector<Part> possible_pose_tray_2;

    parts_tray_1 = sensors.getPartsTray1();
    parts_tray_2 = sensors.getPartsTray2();

    products_tray_1 = gantry.getProductsTray1();
    products_tray_2 = gantry.getProductsTray2();

    ROS_WARN_STREAM("PARTS LOGIC CAMERA SIZE: " << parts_tray_1.size());
    ROS_WARN_STREAM("PRODUCTS TRAY SIZE: " << products_tray_1.size());

    bool not_correct;
    bool not_possible;
    double part_roll, part_pitch, part_yaw;
    double product_roll, product_pitch, product_yaw;
    double d_x, d_y, d_yaw;

    for (int prt=0; prt< parts_tray_1.size();prt++) // iterate through parts pose detected by logical camera
    {
        not_correct = true;
        for (int prod=0;prod<products_tray_1.size();prod++) // check with pose of each product saved in placed tray list 
        {
            d_x = abs(parts_tray_1.at(prt).pose.position.x - products_tray_1.at(prod).p.pose.position.x);
            d_y = abs(parts_tray_1.at(prt).pose.position.y - products_tray_1.at(prod).p.pose.position.y);
            tf2::Quaternion part_q(parts_tray_1.at(prt).pose.orientation.x,
                                      parts_tray_1.at(prt).pose.orientation.y,
                                      parts_tray_1.at(prt).pose.orientation.z,
                                      parts_tray_1.at(prt).pose.orientation.w);
            tf2::Matrix3x3(part_q).getRPY(part_roll, part_pitch, part_yaw);
            tf2::Quaternion product_q(products_tray_1.at(prod).p.pose.orientation.x,
                                      products_tray_1.at(prod).p.pose.orientation.y,
                                      products_tray_1.at(prod).p.pose.orientation.z,
                                      products_tray_1.at(prod).p.pose.orientation.w);
            tf2::Matrix3x3(product_q).getRPY(product_roll, product_pitch, product_yaw);

            d_yaw = abs(product_yaw-part_yaw);

            ROS_WARN_STREAM("Part Logic Camera: " << parts_tray_1.at(prt).type);
            ROS_WARN_STREAM("x" << parts_tray_1.at(prt).pose.position.x);
            ROS_WARN_STREAM("y: " << parts_tray_1.at(prt).pose.position.y);
            ROS_WARN_STREAM("yaw" << part_yaw);
            ROS_WARN_STREAM("Product: " << products_tray_1.at(prod).type);
            ROS_WARN_STREAM("x" << products_tray_1.at(prod).p.pose.position.x);
            ROS_WARN_STREAM("y: " << products_tray_1.at(prod).p.pose.position.y);
            ROS_WARN_STREAM("yaw" << product_yaw);
            if (d_x < 0.1 && d_y < 0.1 && (d_yaw < 0.4 || d_yaw > 3))
            {
                not_correct = false;
                break;
            }  
        }

        if (not_correct)
        {
            incorrect_pose_tray_1.push_back(parts_tray_1.at(prt));
        } else {
            ROS_WARN_STREAM("differences: " << d_x <<","<< d_y <<","<< d_yaw );
            ROS_WARN_STREAM("correct part pose" << parts_tray_1.at(prt).type);
            correct_pose_tray_1.push_back(parts_tray_1.at(prt));
        }
    }

    for (int prt=0; prt< parts_tray_2.size();prt++) // iterate through parts pose detected by logical camera
    {
        not_correct = true;
        for (int prod=0;prod<products_tray_2.size();prod++) // check with pose of each product saved in placed tray list 
        {
            d_x = abs(parts_tray_2.at(prt).pose.position.x - products_tray_2.at(prod).p.pose.position.x);
            d_y = abs(parts_tray_2.at(prt).pose.position.y - products_tray_2.at(prod).p.pose.position.y);
            tf2::Quaternion part_q(parts_tray_2.at(prt).pose.orientation.x,
                                      parts_tray_2.at(prt).pose.orientation.y,
                                      parts_tray_2.at(prt).pose.orientation.z,
                                      parts_tray_2.at(prt).pose.orientation.w);
            tf2::Matrix3x3(part_q).getRPY(part_roll, part_pitch, part_yaw);
            tf2::Quaternion product_q(products_tray_2.at(prod).p.pose.orientation.x,
                                      products_tray_2.at(prod).p.pose.orientation.y,
                                      products_tray_2.at(prod).p.pose.orientation.z,
                                      products_tray_2.at(prod).p.pose.orientation.w);
            tf2::Matrix3x3(product_q).getRPY(product_roll, product_pitch, product_yaw);

            d_yaw = abs(product_yaw-part_yaw);
            if (d_x < 0.1 && d_y < 0.1 && d_yaw < 0.4)
            {
                not_correct = false;
                break;
            }
        }

        if (not_correct)
        {
            incorrect_pose_tray_2.push_back(parts_tray_2.at(prt));
        } else {
            correct_pose_tray_2.push_back(parts_tray_2.at(prt));
        }
    }

    // check for possible incorrect candidates in products_tray_ vector
    for (int i=0; i<products_tray_1.size();i++)
    {
        not_possible = true;
        for (int j=0; j<correct_pose_tray_1.size();j++)
        {
            d_x = abs(correct_pose_tray_1.at(j).pose.position.x - products_tray_1.at(i).p.pose.position.x);
            d_y = abs(correct_pose_tray_1.at(j).pose.position.y - products_tray_1.at(i).p.pose.position.y);
            tf2::Quaternion part_q(correct_pose_tray_1.at(j).pose.orientation.x,
                                      correct_pose_tray_1.at(j).pose.orientation.y,
                                      correct_pose_tray_1.at(j).pose.orientation.z,
                                      correct_pose_tray_1.at(j).pose.orientation.w);
            tf2::Matrix3x3(part_q).getRPY(part_roll, part_pitch, part_yaw);
            tf2::Quaternion product_q(products_tray_1.at(i).p.pose.orientation.x,
                                      products_tray_1.at(i).p.pose.orientation.y,
                                      products_tray_1.at(i).p.pose.orientation.z,
                                      products_tray_1.at(i).p.pose.orientation.w);
            tf2::Matrix3x3(product_q).getRPY(product_roll, product_pitch, product_yaw);

            d_yaw = abs(product_yaw-part_yaw);
            if (d_x < 0.1 && d_y < 0.1 && d_yaw < 0.4)
            {
                not_possible = false;
                break;
            }
        }
        if (not_possible)
        {
            possible_pose_tray_1.push_back(products_tray_1.at(i).p);
        }
    }

    // move part from incorrect pose to possible candidate pose of same part type
    if (incorrect_pose_tray_1.empty() != 1)
    {
        for (int inc=0;inc<incorrect_pose_tray_1.size();inc++)
        {
            for (int poss=0;poss<possible_pose_tray_1.size();poss++)
            {
                if(incorrect_pose_tray_1.at(inc).type == possible_pose_tray_1.at(poss).type)
                {

                    gantry.moveFaultyGripperPart(incorrect_pose_tray_1.at(inc), possible_pose_tray_1.at(poss));
                    possible_pose_tray_1.erase(possible_pose_tray_1.begin() + poss); // erase the possible pose candidate since we are correcting a part for that location                 
                    
                }
            }
        }

        incorrect_pose_tray_1.clear();
        correct_pose_tray_1.clear();
        if (possible_pose_tray_1.empty() != 1)
        {
            possible_pose_tray_1.clear();
        }        
    }
}



/**
 * @brief Check for sensor blackout 
 * 
 * @param sensorNum Logical Camera ID : 0 or 1
 * @param numProductsToBeSent Product size from orders
 * @param sensors Sensor control object
 * @return true 
 * @return false 
 */
bool checkBlackout(int sensorNum, int numProductsToBeSent, SensorControl &sensors)
{
    ros::param::set(ACTIVATE_LOG_CAM, sensorNum);
    ros::Duration(1).sleep();
    ros::param::set(ACTIVATE_LOG_CAM, -1);
    // ROS_INFO_STREAM("Checking the products placed on AGV.");
    int numProductsDetected = sensors.getLogicalCameraNumProducts(sensorNum);
    // ROS_INFO_STREAM("Number of parts detected :" << numProductsDetected);

    if (numProductsDetected == 0)
    {

        ROS_INFO_STREAM("Number of products mismatched := Products on AGV : " << numProductsDetected << ", Expected : " << numProductsToBeSent);
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
    while (true)
    {
        while (comp.areOrdersLeft() && sensors.read_all_sensors_) //--1-Read order until no more found
        {

            // ROS_WARN_STREAM("Starting building kit for the new order");
            list_of_shipments = comp.get_shipment_list(); // get list of shipments of current order in priority order
            //list_of_products = comp.get_product_list();   // get list of products of current order in priority order

            int currentOrderCount = comp.getTotalReceivedOrdersCount();
            comp.setNewOrderAlert(false);
            transitionDone = false;

            Shipment current_shipment;

            std::vector<Product> conveyor_products;
            for (int n_ship = 0; n_ship < list_of_shipments.size() && !comp.newOrderAlert(); n_ship++)
            {
                current_shipment = list_of_shipments.at(n_ship);
                // for (auto& current_shipment: list_of_shipments) {
                list_of_products = comp.get_product_list_from_shipment(current_shipment);

                for (int p = 0; p < list_of_products.size(); p++) // loop all the products to be retrieve from current order
                {

                    current_product = list_of_products.at(p);

                    ROS_WARN_STREAM(current_product.type);

                    current_product.p = sensors.findPart(current_product.type);
                    if (current_product.p.type.empty()) // no parts of desired product found go to conveyor
                    {
                        conveyor_products.push_back(current_product);
                    }
                }
            }

            for (int i = 0; i < 2; i++)
            {
                for (int c = 0; c < conveyor_products.size(); c++)
                {
                    time = 0.0;
                    startig_time = ros::Time::now().toSec();
                    pickedConveyor == 0;
                    while (time <= 120)
                    {
                        pickedConveyor = 0;
                        partsConveyor = sensors.getPartsConveyor();
                        if (partsConveyor.empty() != 1)
                        {
                            double original_y;
                            for (int prt = 0; prt < partsConveyor.size(); prt++)
                            {
                                original_y = partsConveyor.at(prt).pose.position.y - partsConveyor.at(prt).estimated_velocity * (ros::Time::now().toSec() - partsConveyor.at(prt).time_stamp.toSec());
                                if (partsConveyor.at(prt).type.compare(conveyor_products.at(c).type) == 0 && original_y >= 2)
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
                                else if (original_y < 2)
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

                    if (c % 2 != 0 || (c + 1) == conveyor_products.size())
                    {
                        placePartsEmptyBins(gantry, sensors);
                    }
                }
            }
            sensors.clearPartsList();
            sensors.clearLogicalCallVector();
            sensors.read_all_sensors_ = false;
            ros::Duration(5).sleep();

            while (!agvControl.isAGVReady(AGV1_TRAY) && !agvControl.isAGVReady(AGV2_TRAY));

            for (int n_ship = 0; n_ship < list_of_shipments.size() && !comp.newOrderAlert(); n_ship++)
            {
                current_shipment = list_of_shipments.at(n_ship);
                comp.setAgvInUse(current_shipment.agv_id);
                // for (auto& current_shipment: list_of_shipments) {
                list_of_products = comp.get_product_list_from_shipment(current_shipment);

                for (int p = 0; p < list_of_products.size(); p++) // loop all the products to be retrieve from current order
                {

                    ROS_WARN_STREAM("Picking next product: " << p + 1 << "/" << list_of_products.size() << " for AGV: " << list_of_products.at(p).agv_id);
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

                        gantry.placePartLeftArm(); // Place product of left arm in agv
                        gantry.placePartRightArm();

                        if (!comp.newOrderAlert()){
                            ROS_WARN("CHECKING FIRST CORRECT POSE");
                            correctPose(gantry, sensors);
                            ROS_WARN("FINISHED CHECKING FIRST CORRECT POSE");
                            faultyPartsProcess(gantry, sensors);
                            ros::param::set("/check_parts_to_flip", true);
                            ros::Duration(1).sleep();
                            gantry.flipProductsAGV(sensors.getcheckPartsToFlip());
                            sensors.clearcheckPartsToFlip();
                        // sensors.clearPartsToFlip();
                        }

                        gantry.goToPresetLocation(gantry.start_90_); // go back to start position

                        // the last two products that were picked have been placed.
                        comp.updateAGVProductMap(list_of_products.at(p - 1).agv_id, list_of_products.at(p - 1));
                        comp.updateAGVProductMap(list_of_products.at(p - 2).agv_id, list_of_products.at(p - 2));
                    }

                    if (p < list_of_products.size() && !comp.newOrderAlert()) // get product not called in last iteration
                    {
                        if (get_product_from_conveyor)
                        {
                            // gantry.goToPresetLocation(gantry.conveyor_left_);
                            time = 0.0;
                            startig_time = ros::Time::now().toSec();
                            pickedConveyor == 0;
                            while (time <= 120)
                            {
                                partsConveyor = sensors.getPartsConveyor();
                                if (partsConveyor.empty() != 1)
                                {
                                    double original_y;
                                    for (int prt = 0; prt < partsConveyor.size(); prt++)
                                    {
                                        if (partsConveyor.at(prt).type.compare(current_product.type) == 0 && original_y >= 1)
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
                                        else if (original_y < 1)
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

                        gantry.placePartLeftArm(); // Place product of left arm in agv

                        std::string free_gripper = gantry.checkFreeGripper();
                        if (list_of_products.size() % 2 == 0)
                        {
                            // ROS_WARN_STREAM("LAST PART RIGHT ARM PLACING");
                            gantry.placePartRightArm(); // Place product of right arm in agv
                        }
                        else
                        {
                            ros::param::set("/no_prod_right", true);
                        }

                        if(!comp.newOrderAlert()){
                            ROS_WARN("CHECKING FIRST CORRECT POSE");
                            correctPose(gantry, sensors);
                            ROS_WARN("FINISHED CHECKING FIRST CORRECT POSE");
                            faultyPartsProcess(gantry, sensors);
                            ROS_WARN_STREAM("FIRST FAULTY PARTS FINISHED");
                            ros::param::set("/check_parts_to_flip", true);
                            ros::Duration(1).sleep();
                            gantry.flipProductsAGV(sensors.getcheckPartsToFlip());
                            sensors.clearcheckPartsToFlip();
                        }
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

                        // std::string shipmentAGV = current_shipment.agv_id;
                        std::string shipmentAGV = comp.getAgvInUse();
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
                        if (!comp.newOrderAlert())
                        {
                            ROS_WARN("CHECKING FIRST CORRECT POSE");
                            correctPose(gantry, sensors);
                            ROS_WARN("FINISHED CHECKING FIRST CORRECT POSE");
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

    spinner.stop();
    ros::shutdown();
    return 0;
}
