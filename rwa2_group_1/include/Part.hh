
#ifndef PART_HH
#define PART_HH

#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * @brief Class that instanciates objects of type Part with logging methods
 */
namespace rwa2 {

    class Part {
    public:
        explicit Part(std::string part_id, std::string type,std::string color, int sensor,
         geometry_msgs::PoseStamped pose_world): part_id_{part_id}, type_{type}, color_{color}, sensor_{sensor}, 
         pose_world_{pose_world}{}

        /**
        * @brief Method to log the information of the part object
        */
        void Print_Info();
        
        /**
        * @brief Accesor of the type of the part object
        */
        std::string const get_type_() { return type_;}

        /**
        * @brief Accesor of the sensor attribute that has detected the object part
        */
        int const get_sensor() {return sensor_;}

        /**
        * @brief Accessor of the pose of the object part with respect to the world coordinates
        */
        std::vector<double> get_pose_world();

    private:
        std::string part_id_;
        std::string type_;
        std::string color_;
        int sensor_;
        geometry_msgs::PoseStamped pose_world_;
       
    };
}
#endif //RWA2_PART_INFO_HH