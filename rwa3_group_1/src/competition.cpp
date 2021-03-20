#include "competition.h"
#include "utils.h"

#include <std_srvs/Trigger.h>


////////////////////////
Competition::Competition(ros::NodeHandle & node): current_score_(0)
{
  node_ = node;
}

////////////////////////
void Competition::init() {
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

////////////////////////
void Competition::competition_state_callback(const std_msgs::String::ConstPtr & msg) {
  if (msg->data == "done" && competition_state_ != "done")
  {
    ROS_INFO("Competition ended.");
  }
  competition_state_ = msg->data;
}

////////////////////////
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

////////////////////////
void Competition::competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr & msg) {
  competition_clock_ = msg->clock;
}

////////////////////////
void Competition::processOrder(){
  auto current_order = order_list_.front();
  auto current_shipment = current_order.shipments.front();//--change this line to handle multiple shipments
  auto product_list = current_shipment.products;

  for (const auto &product: product_list)
  {
    product_list.push_back(product);
  }
}


// static void fillOrderMsg()
// {
//   nist_gear::Order _msgOrder = received_orders_.front();

//   _msgOrder.order_id = _order.orderID;
//   for (const auto &shipment : _order.shipments)
//   {
//     nist_gear::Shipment msgShipment;
//     msgShipment.shipment_type = shipment.shipmentType;
//     msgShipment.agv_id = shipment.agv_id;
//     for (const auto &obj : shipment.products)
//     {
//       nist_gear::Product msgObj;
//       msgObj.type = obj.type;
//       msgObj.pose.position.x = obj.pose.Pos().X();
//       msgObj.pose.position.y = obj.pose.Pos().Y();
//       msgObj.pose.position.z = obj.pose.Pos().Z();
//       msgObj.pose.orientation.x = obj.pose.Rot().X();
//       msgObj.pose.orientation.y = obj.pose.Rot().Y();
//       msgObj.pose.orientation.z = obj.pose.Rot().Z();
//       msgObj.pose.orientation.w = obj.pose.Rot().W();

//       // Add the product to the shipment.
//       msgShipment.products.push_back(msgObj);
//     }
//     _msgOrder.shipments.push_back(msgShipment);
//   }
// }



////////////////////////
void Competition::startCompetition() {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("[competition][startCompetition] Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("[competition][startCompetition] Competition is now ready.");
  }
  ROS_INFO("[competition][startCompetition] Requesting competition start...");
  std_srvs::Trigger srv;
  start_client.call(srv);
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("[competition][startCompetition] Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("[competition][startCompetition] Competition started!");
  }
}

////////////////////////
void Competition::endCompetition() {
  // Create a Service client for the correct service, i.e. '/ariac/end_competition'.
  ros::ServiceClient end_client =
    node_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!end_client.exists()) {
    ROS_INFO("[competition][endCompetition] Waiting for the end_competition to be ready...");
    end_client.waitForExistence();
    ROS_INFO("[competition][endCompetition] end_competition is now ready.");
  }
  ROS_INFO("[competition][endCompetition] Requesting competition end...");
  std_srvs::Trigger srv;
  end_client.call(srv);
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("[competition][endCompetition] Failed to end the competition: " << srv.response.message);
  } else {
    ROS_INFO("[competition][endCompetition] Competition ended!");
  }
}


////////////////////////
double Competition::getStartTime() {
  return competition_start_time_;
}

////////////////////////
double Competition::getClock() {
  double time_spent = competition_clock_.toSec();
  ROS_INFO_STREAM("[competition][getClock] competition time spent (getClock()) =" << time_spent);
  return time_spent;
}

////////////////////////
std::string Competition::getCompetitionState() {
  return competition_state_;
}
