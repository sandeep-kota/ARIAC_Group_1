#ifndef COMPETITION_H
#define COMPETITION_H

#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include <nist_gear/Order.h>
#include <unordered_map> 
#include <unordered_set> 
#include <queue>

#include "utils.h"
#include "gantry_control.h"
#include "sensor_control.h"

/**
 * @brief Competition class
 * 
 */
class Competition
{
public:
    std::unordered_map<std::string, std::queue<std::string>> agvToShipmentMap;
    /**
     * @brief Construct a new Competition object
     * 
     * @param node Node handle
     */
    explicit Competition(ros::NodeHandle &node);
    /**
     * @brief Initialize components of the class (suscribers, publishers, etc)
     * 
     */
    void init();
    /**
     * @brief Start the competition through ROS service
     * 
     */
    void startCompetition();
    /**
     * @brief End competition through ROS service
     * 
     */
    void endCompetition();
    /**
     * @brief Get the state of the competition (init, end, etc)
     * 
     * @param msg 
     */
    void competition_state_callback(const std_msgs::String::ConstPtr &msg);
    /**
     * @brief Time since the competition started
     * 
     * @param msg 
     */
    void competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr &msg);
    /**
     * @brief Deal with received orders from /ariac/orders topic
     * 
     * @param msg 
     */
    void order_callback(const nist_gear::Order::ConstPtr &msg);
    /**
     * @brief Handle products in the order
     * 
     */
    bool processOrder();
    /**
     * @brief Get the Clock objectGet 
     * 
     * @return double 
     */
    double getClock();
    /**
     * @brief Get the Start Time object
     * 
     * @return double 
     */
    double getStartTime();
    /**
     * @brief Get the Competition State object
     * 
     * @return std::string 
     */
    std::string getCompetitionState();
    std::vector<Product> get_product_list();
    std::vector<Product> get_product_list_from_shipment(Shipment shipment);
    
    std::vector<Shipment> get_shipment_list()
    {
        shipment_list_ = orderStack.top();
        orderStack.pop();
        return shipment_list_;
    }

    int getTotalReceivedOrdersCount() const {
        return receivedOrdersCount;
    }

    int updateTotalOrderCount() {
        receivedOrdersCount++;
    }

    void updateAGVProductMap(std::string agvid, Product prod) {
        agvToProductsMap.at(agvid).emplace_back(prod);
    }

    bool areOrdersLeft() {
        return !orderStack.empty();
    }

    bool newOrderAlert() {
        return newOrderAlertFlag;
    }

    bool setNewOrderAlert(bool flag) {
        return newOrderAlertFlag = flag;
    }

    void orderTransition(std::vector<Shipment> prevShipments, SensorControl &sensors, GantryControl& gantry);
    void removePrevProductsFromAGV(std::string fromAGV, SensorControl &sensors, GantryControl &gantry);

private:
    ros::NodeHandle node_;                          /*!< node h_type: "ordeandle for this class */
    std::string competition_state_;                 /*!< state of the competition */
    double current_score_;                          /*!< current score during the trial */
    ros::Time competition_clock_;                   /*!< clock to check if we are close to the time limit */
    double competition_start_time_;                 /*!< wall time in second */
    std::vector<nist_gear::Order> received_orders_; /*!< vector to store orders */
    std::vector<Order> order_list_;                 /*!< list of orders */
    std::vector<Product> product_list_;
    std::vector<Shipment> shipment_list_;
    ros::Subscriber current_score_subscriber_;     /*!< subscriber to the topic /ariac/current_score */
    ros::Subscriber competition_state_subscriber_; /*!< subscriber to the topic /ariac/competition_state */
    ros::Subscriber competition_clock_subscriber_; /*!< subscriber to the topic /clock */
    ros::Subscriber orders_subscriber_;            /*!< subscriber to the topic /ariac/orders */

    std::unordered_map<std::string, std::vector<Product>> agvToProductsMap {{AGV1_ID, {}}, {AGV2_ID, {}}};
    std::stack<std::vector<Shipment>> orderStack;
    std::unordered_set<std::string> orderSet;

    bool newOrderAlertFlag = false;

    int receivedOrdersCount;

};

#endif
