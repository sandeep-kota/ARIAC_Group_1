#include "agv_control.h"

AGVControl::AGVControl(ros::NodeHandle &node)
{
    agv1_client =
            node.serviceClient<nist_gear::AGVControl>("/ariac/agv1");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!agv1_client.exists()) {
        ROS_INFO("Waiting for the AGV1 to be ready...");
        agv1_client.waitForExistence();
        ROS_INFO("AGV1 is now ready.");
    }

    agv1_state_subscriber_ = node.subscribe(
            "ariac/agv1/state", 10, &AGVControl::agv1_state_callback, this);

    agv2_client =
            node.serviceClient<nist_gear::AGVControl>("/ariac/agv2");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!agv2_client.exists()) {
        ROS_INFO("Waiting for the AGV2 to be ready...");
        agv2_client.waitForExistence();
        ROS_INFO("AGV2 is now ready.");
    }

    agv2_state_subscriber_ = node.subscribe(
            "ariac/agv2/state", 10, &AGVControl::agv2_state_callback, this);

}

bool AGVControl::isAGVReady(std::string tray_name)
{
    if (tray_name == "kit_tray_1") return agv1_ready;
    if (tray_name == "kit_tray_2") return agv2_ready;
    return false;
}


bool AGVControl::sendAGV(std::string shipment_type, std::string tray_name) {


  nist_gear::AGVControl msg;
  msg.request.shipment_type = shipment_type;

  (tray_name == "kit_tray_1") ? agv1_client.call(msg) : agv2_client.call(msg);

  if (msg.response.success) {
    ROS_INFO_STREAM("[agv_control][sendAGV] AGV is taking order: " + msg.request.shipment_type);
    return true;
  }
  else {
    ROS_ERROR_STREAM("[agv_control][sendAGV] Failed to call AGV!");
    return false;
  }
}



void AGVControl::agv1_state_callback(const std_msgs::String &msg)
{
    if (!((msg.data).compare("ready_to_deliver")))
        agv1_ready = true;
    else agv1_ready = false;
}

void AGVControl::agv2_state_callback(const std_msgs::String &msg)
{
    if (!((msg.data).compare("ready_to_deliver")))
        agv2_ready = true;
    else agv2_ready = false;
}
