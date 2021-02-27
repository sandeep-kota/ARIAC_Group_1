
#ifndef PART_HH
#define PART_HH

#include <iostream>
#include <geometry_msgs/Pose.h>

namespace rwa2 {

    class Part {
    public:
        Part(std::string type,std::string color,std::string name_id, std::string sensor,
         geometry_msgs::Pose pose_world): type_{type}, color_{color}, name_id_{name_id}, 
         sensor_{sensor}, pose_world_{pose_world}{}

        void Print_Info();

    private:

        std::string type_;
        std::string color_;
        std::string name_id_;
        std::string sensor_;
        geometry_msgs::Pose pose_world_;
       
    };
}
#endif //RWA2_PART_INFO_HH