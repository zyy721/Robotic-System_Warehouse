#ifndef OBSTABLE_MANAGER_H
#define OBSTABLE_MANAGER_H


#include "ros/ros.h"
#include <topic_tools/MuxSelect.h>
#include <std_srvs/Empty.h>
#include <cstdlib>
#include <string>

namespace IRIM
{
    class Obstable_Manager
    {
    public:
        Obstable_Manager(ros::NodeHandle &nh)
        {
            Obstable_Manager_client = nh.serviceClient<topic_tools::MuxSelect>("/mux/select");
            Octomap_client = nh.serviceClient<std_srvs::Empty>("/clear_octomap");
        }
        
        void add_obstacle();
        void remove_obstacle(); 
    private:
        ros::ServiceClient Obstable_Manager_client;
        ros::ServiceClient Octomap_client;

    };
}


#endif /* OBSTABLE_MANAGER_H */