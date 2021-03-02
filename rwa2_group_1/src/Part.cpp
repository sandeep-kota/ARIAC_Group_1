#include "../include/Part.hh"

std::vector<double> rwa2::Part::get_pose_world() {
	std::vector<double> r{std::round(pose_world_.pose.position.x*100.0)/100.0,
						std::round(pose_world_.pose.position.y*100.0)/100.0,
						std::round(pose_world_.pose.position.z*100.0)/100.0};
	return r;
}

void rwa2::Part::Print_Info() {

    // std::cout << "\n-------------------------" << std::endl;
	std::cout << "Part ID:" + part_id_ <<std::endl;
    std::cout << "Sensor: " + std::to_string(sensor_) <<std::endl;
    std::cout << "Type: " + type_ <<std::endl;
    std::cout << "Color: " + color_ <<std::endl;
    std::cout << "Position:\n\tx: " + std::to_string(pose_world_.pose.position.x) +"\n\ty: " +  
    std::to_string(pose_world_.pose.position.y) +"\n\tz: " +  
    std::to_string(pose_world_.pose.position.z)<<std::endl;
    std::cout << "`````"<<std::endl;
    std::cout << "Orientation:\n\tx: " + std::to_string(pose_world_.pose.orientation.x) +"\n\ty: " +  
    std::to_string(pose_world_.pose.orientation.y) +"\n\tz: " +  
    std::to_string(pose_world_.pose.orientation.z) + "\n\tw: " +
    std::to_string(pose_world_.pose.orientation.w)<<std::endl;

    tf2::Quaternion q(
        pose_world_.pose.orientation.x,
        pose_world_.pose.orientation.y,
        pose_world_.pose.orientation.z,
        pose_world_.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    std::cout << "\n\troll: " + std::to_string(roll) + "\n\tpitch: " + std::to_string(pitch) + 
    "\n\tyaw: " + std::to_string(yaw) << std::endl;
    std::cout << "````" << std::endl;
    std::cout << "-------------------------" << std::endl;

}