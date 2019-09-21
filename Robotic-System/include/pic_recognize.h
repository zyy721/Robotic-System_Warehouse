#ifndef PIC_RECOGNIZE_H
#define PIC_RECOGNIZE_H


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <atomic>
#include <geometry_msgs/Pose2D.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/RegionOfInterest.h>

extern std::atomic_int mouseX;
extern std::atomic_int mouseY;
extern std::atomic_int mouseBtnType;

extern ros::Publisher leftBtnPointPub;
 
void onMouse(int event, int x, int y, int flags, void* ustc);
void Origin_DcallBack(const sensor_msgs::RegionOfInterest::ConstPtr& msg);

void Origin_3DcallBack(const geometry_msgs::Pose2D::ConstPtr& msg);
void help(const std::string &path);

class Receiver
{
public:
  double offset_x,offset_y, offset_z;
  const double Upper_Bonudary = 0.9, Lower_Boundary = 0.6;

  std::string TARGET_NAME;
  std::string CAMERA_NAME;

private:
    std::mutex lock;
    const std::string topicColor, topicDepth;
    const int choseLocal_size = 1, choseLocal_size2 = 2;
  
    bool updateImage, updateCloud;
    bool running;
    size_t frame;
    const size_t queueSize;

    cv::Mat color, depth;
    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner;
    image_transport::ImageTransport it;
    image_transport::SubscriberFilter *subImageColor, *subImageDepth;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

    std::thread imageViewerThread;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    std::ostringstream oss;

 public:
    Receiver(const std::string &topicColor, const std::string &topicDepth)
    : topicColor(topicColor), topicDepth(topicDepth), 
        updateImage(false), updateCloud(false),running(false), frame(0), queueSize(5),
        nh("~"), spinner(0), it(nh)
    {
        offset_x = 0.f;
        offset_y = 0.f;
        offset_z = 0.f;
        TARGET_NAME = "target_position";
        CAMERA_NAME = "kinect2_link";
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    }
    ~Receiver()
    {
    }
    void run(void);
    tf::Transform find_the_nearest_transform(std::vector<tf::Transform> &trasform_list);
    tf::Transform find_center_point(double img_x, double img_y, cv::Mat &depth);

private:
    void start(void);
    void stop(void);
    void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
            const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);
    void cloudViewer(void);
    void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const;
    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const;
    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const;
    void createLookup(size_t width, size_t height);
};

#endif /* PIC_RECOGNIZE_H */ 