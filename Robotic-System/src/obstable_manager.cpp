#include "obstable_manager.h"

namespace IRIM
{
    void Obstable_Manager::add_obstacle()
    {
        std::cout << "add obstable " << std::endl;
        topic_tools::MuxSelect switch_mux;
        switch_mux.request.topic = "/kinect2/qhd/points";
        Obstable_Manager_client.call(switch_mux);
        sleep(1);

    }

    void Obstable_Manager::remove_obstacle()
    { 
        std::cout << "remove obstable " << std::endl;        
        topic_tools::MuxSelect switch_mux;
        std_srvs::Empty temp;
        switch_mux.request.topic = "/filtered_cloud";
        Obstable_Manager_client.call(switch_mux);
        Octomap_client.call(temp);
    }
}