#include "dynamic_obs.h"

DynamicObs::DynamicObs(ros::NodeHandle &node)
{
	node_ = node;

	for (int i=0; i< 8; ++i) {
		break_beam_subscriber_[i] = node_.subscribe<nist_gear::Proximity>
		("/ariac/breakbeam_"+std::to_string(i+1)+"_change", 1, 
			boost::bind(&DynamicObs::break_beam_callback, this, _1, i+1));
	}

	for (int i =0; i< NUM_OBSTACLES ; ++i) {
		dyn_obs_pos_pub_[i] = node_.advertise<geometry_msgs::Point32>
			("obstacle_" + std::to_string(i+1) + "_pos", 2);	
	}

	for(int i=0; i< NUM_OBSTACLES; ++i)
		obs_[i].z = -1;

	memset(reading_time_, 0, sizeof(reading_time_));

}

void DynamicObs::break_beam_callback(const nist_gear::Proximity::ConstPtr &msg, int sensor) {
	double time = msg->header.stamp.toSec();
	if(num_obstacles_<NUM_OBSTACLES) {
		std::vector<int> sensor_index = {num_obstacles_, num_obstacles_+NUM_OBSTACLES};
		sensor_id_.push_back(sensor_index);
		obs_[num_obstacles_].y = y_locs[sensor];
		sensor_pos_.insert(std::pair<int, int>(sensor, num_obstacles_));
		sensor_pos_.insert(std::pair<int, int>(sensor+4, num_obstacles_+NUM_OBSTACLES));


		++num_obstacles_;
	}
	
	cur_reading_[sensor_pos_[sensor]]= msg->object_detected;
	reading_time_[sensor_pos_[sensor]]= time;
}

bool DynamicObs::isBlackout() {

	double cur_time = ros::Time::now().toSec();
	int latest_time = -1;
	for(int i = 0; i< 2*NUM_OBSTACLES; ++i) {
		latest_time = latest_time>reading_time_[i]? latest_time:reading_time_[i];
	}
	if(cur_time - latest_time <= 8) return false;
	else return true;
}

void DynamicObs::publish_data() {

	bool blackout = isBlackout();
	ros::param::set("/ariac/sensor_blackout", blackout);
	if (blackout) {
		ROS_WARN_STREAM("Sensor blackout detected");
	}

	double cur_time = ros::Time::now().toSec();
	if(!blackout) {

		for(int i=0; i< num_obstacles_; ++i) {
			int sensor1 = sensor_id_[i][0];
			int sensor2 = sensor_id_[i][1];
			
			if(cur_reading_[sensor1]) {
				obs_[i].x = -1.6;
			} else if(cur_reading_[sensor2]) {
				obs_[i].x = -16.6;
			} else {
				if(reading_time_[sensor1]>reading_time_[sensor2]) {
					obs_[i].z =  -1;
				} else if (reading_time_[sensor1]<reading_time_[sensor2]) {
					obs_[i].z =  1;
				}

				if(obs_[i].z==1) {
					obs_[i].x = -16.6 + (15.0/MOVE_TIME)*(cur_time - reading_time_[sensor2]);
				} else {
					obs_[i].x = -1.6 - (15.0/MOVE_TIME)*(cur_time - reading_time_[sensor1]);
				}
			}
		}

	} else {
		// Made sure for for max operation both args are doubles not ints
		// fmod(a,b) = a%b... used for doubles instead of ints 

		for(int i=0; i< num_obstacles_; ++i) {
			int sensor1 = sensor_id_[i][0];
			int sensor2 = sensor_id_[i][1];

			if(cur_reading_[sensor1]) {
				double time_diff = std::fmod(cur_time - reading_time_[sensor1],2*(MOVE_TIME+WAIT_TIME));
				if( time_diff< MOVE_TIME+WAIT_TIME) {
					obs_[i].x = -1.6 - (15.0/MOVE_TIME)*std::max(0.0, time_diff - WAIT_TIME);
					obs_[i].z = time_diff>WAIT_TIME? -1:1;
				}
				else {
					obs_[i].x = -16.6 + (15.0/MOVE_TIME)*std::max(0.0,time_diff-2*WAIT_TIME-MOVE_TIME);
					obs_[i].z = time_diff>2*WAIT_TIME+MOVE_TIME? 1:-1;
				}

			} else if(cur_reading_[sensor2]) {
				double time_diff = std::fmod(cur_time - reading_time_[sensor2],2*(MOVE_TIME+WAIT_TIME));
				if( time_diff< MOVE_TIME+WAIT_TIME) {
					obs_[i].x = -16.6 + (15.0/MOVE_TIME)*std::max(0.0, time_diff - WAIT_TIME);
					obs_[i].z = time_diff>WAIT_TIME? 1:-1;
				}
				else {
					obs_[i].x = -1.6 - (15.0/MOVE_TIME)*std::max(0.0, time_diff-2*WAIT_TIME-MOVE_TIME);
					obs_[i].z = time_diff>2*WAIT_TIME+MOVE_TIME? -1:1;
				}
			} else {
				obs_[i].z = reading_time_[sensor1]>reading_time_[sensor2]? -1:1;
				double time_diff = std::fmod(cur_time - 
								std::max(reading_time_[sensor1],reading_time_[sensor2]),2*(MOVE_TIME+WAIT_TIME));
				
				if(time_diff<MOVE_TIME+WAIT_TIME && obs_[i].z == -1){
					obs_[i].x = std::max(-16.6, -1.6 - (15.0/MOVE_TIME)*time_diff);
				} else if (time_diff<MOVE_TIME+WAIT_TIME && obs_[i].z == 1) {
					obs_[i].x = std::min(-1.6, -16.6 + (15.0/MOVE_TIME)*time_diff);
				} else if(time_diff>MOVE_TIME+WAIT_TIME && obs_[i].z == -1) {
					obs_[i].x = std::min(-1.6, -16.6 + (15.0/MOVE_TIME)*(time_diff-MOVE_TIME-WAIT_TIME));
					obs_[i].z = 1;
				} else if(time_diff>MOVE_TIME+WAIT_TIME && obs_[i].z == 1) {
					obs_[i].x = std::max(-16.6, -1.6 - (15.0/MOVE_TIME)*(time_diff-MOVE_TIME-WAIT_TIME));
					obs_[i].z = -1;
				}
			}
		}	
	}
	for(int i=0; i< NUM_OBSTACLES; ++i) {
		dyn_obs_pos_pub_[i].publish(obs_[i]);
	}
	return;

}


int main(int argc, char **argv) {
	ros::init(argc, argv, "dyn_obs_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);

    DynamicObs dynamic_obs(node);

    while(ros::ok()) {
    	dynamic_obs.publish_data();
    	ros::spinOnce();
    	loop_rate.sleep();

    }
}

