#include "competition.h"
#include "utils.h"

#include <std_srvs/Trigger.h>

/**
 * @brief Construct a new Competition:: Competition object
 * 
 * @param node 
 */
Competition::Competition(ros::NodeHandle &node) : current_score_(0)
{
  node_ = node;
}

/**
 * @brief Constructor
 * 
 */
void Competition::init()
{
  // Subscribe to the '/ariac/current_score' topic.
  double time_called = ros::Time::now().toSec();
  competition_start_time_ = ros::Time::now().toSec();

  // Subscribe to the '/ariac/competition_state' topic.
  ROS_INFO("Subscribe to the /ariac/competition_state topic...");
  competition_state_subscriber_ = node_.subscribe(
      "/ariac/competition_state", 10, &Competition::competition_state_callback, this);

  // Subscribe to the '/clock' topic.
  ROS_INFO("Subscribe to the /clock...");
  competition_clock_subscriber_ = node_.subscribe(
      "/clock", 10, &Competition::competition_clock_callback, this);

  ROS_INFO("Subscribe to the /orders...");
  orders_subscriber_ = node_.subscribe(
      "/ariac/orders", 10, &Competition::order_callback, this);

  startCompetition();
}
/**
 * @brief Competition State Callback
 * 
 * @param msg Subscribed Message
 */
void Competition::competition_state_callback(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data == "done" && competition_state_ != "done")
  {
    ROS_INFO("Competition ended.");
  }
  competition_state_ = msg->data;
}

/**
 * @brief Order list callback
 * 
 * @param order_msg Subscribed Message
 */
void Competition::order_callback(const nist_gear::Order::ConstPtr &order_msg)
{
  ROS_INFO_STREAM("Received order:\n"
                  << *order_msg);
  received_orders_.push_back(*order_msg);

  //--order
  order new_order;
  new_order.order_id = order_msg->order_id;

  //--shipments
  for (int s = 0; s < order_msg->shipments.size(); s++)
  {
    shipment new_shipment;
    new_shipment.shipment_type = order_msg->shipments[s].shipment_type;
    new_shipment.agv_id = order_msg->shipments[s].agv_id;
    //--products
    for (int p = 0; p < order_msg->shipments[s].products.size(); p++)
    {
      product new_product;
      new_product.type = order_msg->shipments[s].products[p].type;
      new_product.pose = order_msg->shipments[s].products[p].pose;
      new_product.agv_id = order_msg->shipments[s].agv_id;

      if (new_product.agv_id == "agv1")
      {
        new_product.tray = "kit_tray_1";
      };
      if (new_product.agv_id == "agv2")
      {
        new_product.tray = "kit_tray_2";
      };
      if (new_product.agv_id == "any")
      {
        new_product.tray = "kit_tray_1";
      };
      new_shipment.products.push_back(new_product);
    }
    new_order.shipments.push_back(new_shipment);
  }
  order_list_.push_back(new_order);
}

void Competition::competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr &msg)
{
  competition_clock_ = msg->clock;
}

/**
 * @brief Process the order list
 * 
 * @return true 
 * @return false 
 */
bool Competition::processOrder()
{

  Order current_order;
  if (order_list_.empty())
  {
    // ROS_INFO_STREAM("Order empty");
    return false;
  }
  else
  {
    // ROS_INFO_STREAM("Order not empty");
    current_order = order_list_.front();
    ROS_INFO_STREAM("Current Order:\n"<<current_order);
    order_list_.erase(order_list_.begin());

    shipment_list_.clear();
    product_list_.clear();

    // ROS_INFO_STREAM("Clear shipment an dproduct lists");

    for (int s = 0; s < current_order.shipments.size(); s++)
    {
      shipment_list_.emplace_back(current_order.shipments.at(s));
      std::string agv_id = current_order.shipments.at(s).agv_id;
      std::string shipment_type = current_order.shipments.at(s).shipment_type;

      if (agvToShipmentMap.find(agv_id) == agvToShipmentMap.end())
      {
        std::queue<std::string> shipmentQueue;
        agvToShipmentMap.insert({agv_id, shipmentQueue});
      }
      agvToShipmentMap.at(agv_id).push(shipment_type);
      for (int p = 0; p < current_order.shipments.at(s).products.size(); p++)
      {
        product_list_.emplace_back(current_order.shipments.at(s).products.at(p));
      }
    }
    ROS_INFO_STREAM("SHIPMENT SIZE: " << shipment_list_.size());
    ROS_INFO_STREAM("PRODUCT SIZE: " << product_list_.size());
    return true;
  }
}

////////////////////////
void Competition::startCompetition()
{
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
      node_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists())
  {
    ROS_INFO("[competition][startCompetition] Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("[competition][startCompetition] Competition is now ready.");
  }
  ROS_INFO("[competition][startCompetition] Requesting competition start...");
  std_srvs::Trigger srv;
  start_client.call(srv);
  if (!srv.response.success)
  { // If not successful, print out why.
    ROS_ERROR_STREAM("[competition][startCompetition] Failed to start the competition: " << srv.response.message);
  }
  else
  {
    ROS_INFO("[competition][startCompetition] Competition started!");
  }
}

////////////////////////
void Competition::endCompetition()
{
  // Create a Service client for the correct service, i.e. '/ariac/end_competition'.
  ros::ServiceClient end_client =
      node_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!end_client.exists())
  {
    ROS_INFO("[competition][endCompetition] Waiting for the end_competition to be ready...");
    end_client.waitForExistence();
    ROS_INFO("[competition][endCompetition] end_competition is now ready.");
  }
  ROS_INFO("[competition][endCompetition] Requesting competition end...");
  std_srvs::Trigger srv;
  end_client.call(srv);
  if (!srv.response.success)
  { // If not successful, print out why.
    ROS_ERROR_STREAM("[competition][endCompetition] Failed to end the competition: " << srv.response.message);
  }
  else
  {
    ROS_INFO("[competition][endCompetition] Competition ended!");
  }
}

////////////////////////
double Competition::getStartTime()
{
  return competition_start_time_;
}

////////////////////////
double Competition::getClock()
{
  double time_spent = competition_clock_.toSec();
  ROS_INFO_STREAM("[competition][getClock] competition time spent (getClock()) =" << time_spent);
  return time_spent;
}

////////////////////////
std::string Competition::getCompetitionState()
{
  return competition_state_;
}
