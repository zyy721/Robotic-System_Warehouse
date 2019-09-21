#include "pic_recognize.h"

using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
  
  leftBtnPointPub = nh.advertise<geometry_msgs::PointStamped>("/kinect2/click_point/left", 1);

  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;

  Receiver receiver(topicColor, topicDepth);

  receiver.run();

  ros::shutdown();
  return 0;
}