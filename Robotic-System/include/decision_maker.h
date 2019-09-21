#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H   

#include <opencv2/core/core.hpp>
#include <ros/spinner.h>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>

typedef std::vector<std::pair<cv::Rect2f, std::string>> RecoResult;
typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;

namespace IRIM
{
    class DecisionMaker
    {
    public:
        void read_from_file(std::string filename);
        bool is_vertical_plan(std::string object_type);
        void save_a_picture();
        bool show_the_result();
        static void save_pic_callBack(const sensor_msgs::Image::ConstPtr& msg);
        bool help();

        RecoResult& get_result();
        std::vector<bool> get_plan_result();
        std::string object_type;
        NodeHandlePtr ptrNodeHandle;
        RecoResult out_Rect;
        geometry_msgs::Pose2D pose2d;

private:
        std::vector<std::string> string_split(std::string str, std::string pattern);
        std::vector<bool> desion_result;
        
    };

    class DeepLearningDM: public DecisionMaker
    {
    public:
        virtual void picture_recongnize();
        bool run();
    };

    class SVMDM:  private DecisionMaker
    {
        virtual void picture_recongnize()
        {

        };
    };


}

#endif /* DECISION_MAKER_H */