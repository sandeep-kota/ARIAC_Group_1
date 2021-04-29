#include "competition.h"
#include "utils.h"
//#include "constants.h"

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

  // dont process the callback if the same order has been received
  if (orderSet.find(new_order.order_id) != orderSet.end())
  {
    return;
  }

  orderSet.insert(new_order.order_id);
  updateTotalOrderCount();
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

      // needs to be checked
      if (new_product.agv_id == "any")
      {
        if (this->agvInUse.size() != 0)
        {
          new_product.tray = agvTrayMap.at(oppositeAGV.at(this->agvInUse));
        }
        else
        {
          new_product.tray = "kit_tray_2";
        }
      };
      new_product.isPlacedOnAGV = false;

      new_shipment.products.push_back(new_product);
    }

    new_order.shipments.push_back(new_shipment);
  }

  order_list_.push_back(new_order);

  processOrder();
}

/**
 * @brief Callback to get the clock
 * 
 * @param msg 
 */
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
    // ROS_INFO_STREAM("Got current order");
    order_list_.erase(order_list_.begin());
    // ROS_INFO_STREAM("Erase from order list");

    shipment_list_.clear();
    product_list_.clear();

    // ROS_INFO_STREAM("Clear shipment an dproduct lists");

    for (int s = 0; s < current_order.shipments.size(); s++)
    {
      shipment_list_.emplace_back(current_order.shipments.at(s));
      std::string agv_id = current_order.shipments.at(s).agv_id;
      std::string shipment_type = current_order.shipments.at(s).shipment_type;

      // if (agvToShipmentMap.find(agv_id) == agvToShipmentMap.end())
      // {
      //   std::queue<std::string> shipmentQueue;
      //   agvToShipmentMap.insert({agv_id, shipmentQueue});
      // }
      // agvToShipmentMap.at(agv_id).push(shipment_type);

      //for (int p = 0; p < current_order.shipments.at(s).products.size(); p++)
      //{
      //product_list_.emplace_back(current_order.shipments.at(s).products.at(p));
      //}
    }
    ROS_INFO_STREAM("SHIPMENT SIZE: " << shipment_list_.size());
    //ROS_INFO_STREAM("PRODUCT SIZE: " << product_list_.size());

    orderStack.emplace(shipment_list_);
    setNewOrderAlert(true);

    return true;
  }
}

/**
 * @brief Start the competition
 * 
 */
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

/**
 * @brief End the competition
 * 
 */
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

/**
 * @brief Get competition start time
 * 
 * @return double 
 */
double Competition::getStartTime()
{
  return competition_start_time_;
}

/**
 * @brief get the current clock time
 * 
 * @return double 
 */
double Competition::getClock()
{
  double time_spent = competition_clock_.toSec();
  ROS_INFO_STREAM("[competition][getClock] competition time spent (getClock()) =" << time_spent);
  return time_spent;
}

/**
 * @brief Get the state of the competition
 * 
 * @return std::string 
 */
std::string Competition::getCompetitionState()
{
  return competition_state_;
}

/**
 * @brief Transfer Products from one AGV tray to another
 * 
 * @param fromAGV From AGV ID
 * @param sensors Sensors object
 * @param gantry Gantry Object
 */
void Competition::removePrevProductsFromAGV(std::string fromAGV, SensorControl &sensors, GantryControl &gantry)
{
  ROS_WARN_STREAM("REMOVE CALLED");

  ROS_WARN_STREAM("AGV ID: " << fromAGV);
  if (fromAGV == "agv1") {
    sensors.detectPartsTray1();
    std::vector<Part> parts_tray_1 = sensors.getPartsTray1();
    for (Part& part_in_agv: parts_tray_1) {
      gantry.pickPartFromTrayLeftArm(part_in_agv, fromAGV);
      gantry.goToPresetLocation(gantry.start_90_);
      placePartsEmptyBins(gantry, sensors);
      gantry.goToPresetLocation(gantry.start_90_);
      gantry.clearProductsTray1();
    }
  } else if (fromAGV == "agv2") {
    sensors.detectPartsTray2();
    ros::Duration(2).sleep();
    std::vector<Part> parts_tray_2 = sensors.getPartsTray2();
    ROS_WARN_STREAM("parts tray 2 size: " << parts_tray_2.size());
    for (Part& part_in_agv: parts_tray_2) {
      gantry.pickPartFromTrayLeftArm(part_in_agv, fromAGV);
      gantry.goToPresetLocation(gantry.start_90_);
      placePartsEmptyBins(gantry, sensors);
      gantry.goToPresetLocation(gantry.start_90_);
      gantry.clearProductsTray2();
    }
  }

  sensors.clearPartsList();
  sensors.clearLogicalCallVector();
  sensors.read_all_sensors_ = false;
  ros::Duration(5).sleep();
  gantry.clearProductsToFlip(); 
  // REMOVE PRODUCTS TO FLIP
  // std::array<std::array<std::vector<part>, 3>, 5> parts_agv = sensors.getPartsAGV(fromAGV);
  // for (int i = 0; i < 5; i++)
  // {
  //   for (int j = 0; j < 3; j++)
  //   {
  //     int t_sum = 0;
  //     for (int k = 0; k < parts_agv.at(i).at(j).size(); k++)
  //     {
  //       Part current_part = parts_agv.at(i).at(j).at(k);

  //       if (k % 2 == 0 && t_sum < 2)
  //       {

  //         gantry.pickPartFromTrayLeftArm(current_part, fromAGV);
  //         if (fromAGV.compare("agv1") == 0)
  //         {
  //           gantry.product_left_arm_.agv_id = "agv2";
  //         }
  //         if (fromAGV.compare("agv2") == 0)
  //         {
  //           gantry.product_left_arm_.agv_id = "agv1";
  //         }
  //         t_sum += 1;
  //       }
  //       else if (k % 2 == 1 && t_sum < 2)
  //       {
  //         gantry.pickPartFromTrayRightArm(current_part, fromAGV);
  //         if (fromAGV.compare("agv1") == 0)
  //         {
  //           gantry.product_right_arm_.agv_id = "agv2";
  //         }
  //         if (fromAGV.compare("agv2") == 0)
  //         {
  //           gantry.product_right_arm_.agv_id = "agv1";
  //         }
  //         t_sum += 1;
  //       }
  //       if (t_sum == 2)
  //       {
  //         t_sum = 0;
  //         gantry.placePartLeftArm();
  //         gantry.placePartRightArm();
  //       }
  //     }
  //   }
  // }

  // std::string toAGV = oppositeAGV.at(fromAGV);
}

/**
 * @brief Process a new priority order
 * 
 * @param prevShipments shipment that is currently being build and needs to be stored
 * @param gantry Gantry class passed by reference
 */
void Competition::orderTransition(std::vector<Shipment> prevShipments, SensorControl &sensors, GantryControl &gantry)
{

  // Assuming there will be only one shipment in the new and the previous orders.
  // That is why collecting the element from index=0
  ROS_INFO_STREAM("New Order Received. Processing New Order Details. Will start assembling soon.");

  Shipment prevShipment = prevShipments.at(0);
  Shipment newShipment = orderStack.top().at(0); // getting the shipment from the recently added shipmentlist.
  orderStack.pop();

  // check for agvid:
  if (this->agvInUse.compare(newShipment.agv_id) == 0)
  {
    removePrevProductsFromAGV(this->agvInUse, sensors, gantry);
    agvToProductsMap.at(this->agvInUse).clear();
  }
  else
  {
    // update the pending items and done items for the previous AGV
    ROS_INFO_STREAM("New shipment to be placed on different agv.");
    ROS_INFO_STREAM("Previous AGV: " << prevShipment.agv_id << " New Shipment AGV: " << newShipment.agv_id);

    for (auto &productOnAGV : agvToProductsMap.at(this->agvInUse))
    {
      for (auto &shipmentProduct : prevShipment.products)
      {
        if (productOnAGV.type.compare(shipmentProduct.type) == 0 && !shipmentProduct.isPlacedOnAGV)
        {
          shipmentProduct.isPlacedOnAGV = true;
          break;
        }
      }
    }

    // placing the prodcucts with agv_id=any on the opposite agv.
    if (newShipment.agv_id.compare(ANY_AGV) == 0)
    {
      ROS_WARN_STREAM("New Shipment to be placed on any AGV");
      for (auto &product : newShipment.products)
      {
        //product.agv_id = oppositeAGV.at(prevShipment.agv_id);
        product.tray = agvTrayMap.at(oppositeAGV.at(this->agvInUse));
      }
    }
  }

  ROS_WARN_STREAM("Adding shipments to the stack.");

  prevShipments[0] = prevShipment;
  orderStack.emplace(prevShipments);
  orderStack.emplace(shipment_list_);

  ROS_WARN_STREAM("clearing AGV map");
  // agvToProductsMap.at(AGV1_ID).clear();
  // agvToProductsMap.at(AGV2_ID).clear();

  ROS_INFO_STREAM("Starting building kit for new order.");
}

/**
 * @brief getter to obtain the current list of products
 * 
 * @return std::vector<Product> list of products being processed
 */
std::vector<Product> Competition::get_product_list()
{
  product_list_.clear();

  for (int s = 0; s < shipment_list_.size(); s++)
  {
    for (int p = 0; p < shipment_list_.at(s).products.size(); p++)
    {
      if (!shipment_list_.at(s).products.at(p).isPlacedOnAGV)
      {
        product_list_.emplace_back(shipment_list_.at(s).products.at(p));
      }
    }
  }
  ROS_INFO_STREAM("PRODUCT SIZE: " << product_list_.size());
  return product_list_;
}

/**
 * @brief getter to obtain the current list of products from a particluar shipment
 * 
 * @return std::vector<Product> list of products being processed
 */
std::vector<Product> Competition::get_product_list_from_shipment(Shipment shipment)
{
  std::vector<Product> products_for_shipment;
  for (int p = 0; p < shipment.products.size(); p++)
  {
    if (!shipment.products.at(p).isPlacedOnAGV)
    {
      products_for_shipment.emplace_back(shipment.products.at(p));
    }
  }

  ROS_INFO_STREAM("PRODUCT SIZE: " << products_for_shipment.size());
  return products_for_shipment;
}

/**
 * @brief Check free AGV to place part
 * 
 * @param agv_id 
 */
void Competition::setAgvInUse(const std::string &agv_id)
{
  if (agv_id.compare(ANY_AGV) == 0)
  {
    agvInUse = agvInUse.size() == 0 || receivedOrdersCount < 2 ? AGV2_ID : oppositeAGV.at(agvInUse);
  }
  else
  {
    agvInUse = agv_id;
  }
}

void Competition::placePartsEmptyBins(GantryControl &gantry, SensorControl &sensors)
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