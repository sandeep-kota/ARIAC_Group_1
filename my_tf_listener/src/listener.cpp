#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10);
  ros::Duration timeout(5.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("world", "logical_camera_12_disk_part_red_9_frame",
                               ros::Time(0), timeout);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    tf2::Quaternion q(
      transformStamped.transform.rotation.x,
      transformStamped.transform.rotation.y,
      transformStamped.transform.rotation.z,
      transformStamped.transform.rotation.w);
   tf2::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);

   ROS_INFO("disk_part_red_9 in world frame: [%f,%f,%f] [%f,%f,%f]",transformStamped.transform.translation.x,
   transformStamped.transform.translation.y,
   transformStamped.transform.translation.z,
   roll,
   pitch,
   yaw);



    rate.sleep();
  }
  return 0;
};
