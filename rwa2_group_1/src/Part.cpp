#include "../include/Part.hh"

void rwa2::Part::Print_Info() {

    std::cout << "\n-------------------------" << std::endl;
    std::cout << "Sensor: " + std::to_string(sensor_) <<std::endl;
    std::cout << "Type: " + type_ <<std::endl;
    std::cout << "Color: " + color_ <<std::endl;
    std::cout << "Pose:\n\tx:  " + std::to_string(pose_world_.pose.position.x) +"\n\ty: " +  
    std::to_string(pose_world_.pose.position.y) +"\n\tz: " +  
    std::to_string(pose_world_.pose.position.z)<<std::endl;
    std::cout << "-------------------------" << std::endl;

}