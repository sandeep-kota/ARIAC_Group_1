#ifndef DYNAMICOBS_H
#define DYNAMICOBS_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>
#include <cmath>


#include "utils.h"

#include <nist_gear/Proximity.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point32.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <ros/ros.h>
#include <ros/console.h>
#include "utils.h"
#include <tf/tf.h>

#define NUM_OBSTACLES 2
#define WAIT_TIME 7
#define MOVE_TIME 9

class DynamicObs {
 public:
 	explicit DynamicObs(ros::NodeHandle &node);
 	void break_beam_callback(const nist_gear::Proximity::ConstPtr &msg, int sensor);
	void blackout_subscriber_callback(const nist_gear::Proximity::ConstPtr &msg);

	bool isBlackout();
	void publish_data();
 private:

 	ros::NodeHandle node_;
 	ros::Subscriber break_beam_subscriber_[8];
 	ros::Subscriber blackout_subscriber_;
 	ros::Publisher dyn_obs_pos_pub_[NUM_OBSTACLES];

 	bool cur_reading_[2*NUM_OBSTACLES] = {false};
 	int reading_time_[2*NUM_OBSTACLES];
	float latest_time = -1.0;

 	geometry_msgs::Point32 obs_[NUM_OBSTACLES];
 	int num_obstacles_ = 0;

 	std::vector<std::vector<int>> sensor_id_;
 	std::unordered_map<int, double> y_locs = {
 		{1, -5.0},
 		{2, -1.57},
 		{3, 1.57},
 		{4, 5}
 	};
 	std::unordered_map<int, int> sensor_pos_;
};

#endif