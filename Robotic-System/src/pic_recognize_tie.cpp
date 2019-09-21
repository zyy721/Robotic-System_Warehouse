
#include "pic_recognize.h"
#include <ro_control/RoiList.h>
#include "std_msgs/String.h"
#include <Eigen/Dense>


using namespace Eigen;

std::atomic_int mouseX;
std::atomic_int mouseY;
std::atomic_int mouseBtnType;
ros::Publisher leftBtnPointPub;

int img_x;
int img_y;
std::vector<geometry_msgs::Pose2D> temp_saved_pose2d_list;
std::vector<geometry_msgs::Pose2D> saved_pose2d_list;
tf::Transform Add_transform(tf::Transform &tf_c2t);
Matrix4f tf_2_matix(tf::Transform &input);
class Listener
{
public:
    void Origin_DcallBack(const ro_control::RoiList::ConstPtr &msg)
    {    
        saved_pose2d_list.resize(0);
        std::vector<sensor_msgs::RegionOfInterest> roi = msg->roi_list;
        
        for(int i = 0; i < roi.size(); i++)
        {
            
            geometry_msgs::Pose2D temp;
            if(roi.at(i).x_offset == 1 && roi.at(i).y_offset == 1)
            {
                continue;
            }
            temp.x = roi.at(i).x_offset + roi.at(i).width;
            temp.y = roi.at(i).y_offset + roi.at(i).height;   
            saved_pose2d_list.push_back(temp);
        }   
        
    }
};

void onMouse(int event, int x, int y, int flags, void* ustc) 
{
    mouseX  = x;
    mouseY  = y;
    mouseBtnType = event;
}

void Receiver::run(void)
{
    start();
    stop();
}
void Receiver::start(void)
{
    running = true;

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    image_transport::TransportHints hints(false ? "compressed" : "raw");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

    syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
    syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));

    spinner.start();

    std::chrono::milliseconds duration(1);
    while(!updateImage || !updateCloud)
    {
        std::this_thread::sleep_for(duration);
    }
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(this->color.cols, this->color.rows);

    cloudViewer();
}

void Receiver::stop()
{
    spinner.stop();

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;

    running = false;
}

void Receiver::callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
            const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
{
    cv::Mat color, depth;

    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    readImage(imageColor, color);
    readImage(imageDepth, depth);

    // IR image input
    if(color.type() == CV_16U)
    {
        cv::Mat tmp;
        color.convertTo(tmp, CV_8U, 0.02);
        cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = color;
    this->depth = depth;
    updateImage = true;
    updateCloud = true;
    lock.unlock();
}


void Receiver::cloudViewer()
 {
  cv::Mat color, depth;
  std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
  double fps = 0;
  size_t frameCount = 0;
  std::ostringstream oss;
  std::ostringstream ossXYZ, ossXYZ_c;
  const cv::Point pos(5, 15);
  const cv::Scalar colorText = CV_RGB(255, 0, 0);
  const double sizeText = 0.5;
  const int lineText = 1;
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  // 从全局变量获取当前鼠标坐标

  geometry_msgs::PointStamped ptMsg;
  geometry_msgs::PointStamped ptMsg_c;
  ptMsg.header.frame_id = CAMERA_NAME;

  lock.lock();
  color = this->color;
  depth = this->depth;
  updateCloud = false;
  lock.unlock();

  const std::string window_name = "color viewer";
  cv::namedWindow(window_name);
  cv::setMouseCallback(window_name, onMouse, nullptr);
  createCloud(depth, color, cloud);

  double x_sum, y_sum, z_sum, x_cent, y_cent, z_cent;
  static tf::TransformBroadcaster broadcaster;
  for(; running && ros::ok();)
  {
    if(updateCloud) 
    {
      lock.lock();
      color = this->color;
      depth = this->depth;
      updateCloud = false;
      lock.unlock();

      createCloud(depth, color, cloud);
      Listener listener;
      static ros::Subscriber sub_2D = nh.subscribe("/yolo_roi", 1, &Listener::Origin_DcallBack, &listener);
      //ros::Subscriber sub_2D = nh.subscribe("/yolo_roi", 1, Origin_DcallBack);

      const pcl::PointXYZRGBA& pt = cloud->points[img_y * depth.cols + img_x];
      ptMsg.point.x = pt.x;
      ptMsg.point.y = pt.y;
      ptMsg.point.z = pt.z;


      ptMsg.header.stamp = ros::Time::now();
      leftBtnPointPub.publish(ptMsg);
      ros::spinOnce();

      ++frameCount;
      now = std::chrono::high_resolution_clock::now();
      double elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
      if(elapsed >= 1.0) 
      {
        fps = frameCount / elapsed;
        oss.str("");
        oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
        start = now;
        frameCount = 0;
      }

      cv::putText(color, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
      ossXYZ.str("");
      ossXYZ << "( " << ptMsg.point.x << ", " << ptMsg.point.y
                                << ", " << ptMsg.point.z << " )";
////////////////////////////
//在这里找到最近的点

        std::vector<tf::Transform> trasform_list;
        
        temp_saved_pose2d_list = saved_pose2d_list;
        
        for(int i = 0; i < temp_saved_pose2d_list.size(); i++)
        {
            tf::Transform transform = find_center_point(temp_saved_pose2d_list.at(i).x, temp_saved_pose2d_list.at(i).y, depth);
            trasform_list.push_back(transform);
        }

        tf::Transform transform = find_the_nearest_transform(trasform_list);

        ossXYZ_c.str("");
        ossXYZ_c << "After Process :( " << transform.getOrigin().x()<< ", " << transform.getOrigin().y()
                                    << ", " << transform.getOrigin().z() << " )";

        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), CAMERA_NAME, TARGET_NAME) );      

        cv::putText(color, ossXYZ.str(), cv::Point(img_x, img_y), font, sizeText, colorText, lineText,CV_AA);
        cv::putText(color, ossXYZ_c.str(), cv::Point(img_x, img_y+20), font, sizeText, colorText, lineText,CV_AA);
        cv::circle(color, cv::Point(img_x, img_y),2, colorText);
        
        cv::imshow(window_name, color);
        cv::waitKey(10);
        }

 }
    cv::destroyAllWindows();
    cv::waitKey(10);
}

tf::Transform Add_transform(tf::Transform &tf_c2t)
{
    tf::Transform tf_b2c;
    tf_b2c.setOrigin(tf::Vector3(-0.0455627443334,-0.373822583469, 0.00825218683868));
    tf_b2c.setRotation(tf::Quaternion(-0.547667638718, 0.546371790541, -0.453722787929, 0.442350150578));
    Matrix4f m_b2c = tf_2_matix(tf_b2c);
    Matrix4f m_c2t = tf_2_matix(tf_c2t);
    Matrix4f result = m_b2c*m_c2t;
    cout << "m_b2c:" << endl << m_b2c << endl;
    cout << "m_c2t:" << endl<< m_c2t << endl;
    cout << "result:" << endl<< result << endl;
    
    
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(result(0,3), result(1,3), result(2,3)));
    tf::Quaternion q;    
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    return transform;
}

Matrix4f tf_2_matix(tf::Transform &input)
{
    Matrix4f m_r;
    float qw = input.getRotation().getW();
    float qx = input.getRotation().getX();
    float qy = input.getRotation().getY();
    float qz = input.getRotation().getZ();

    float n = 1.0f / sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    m_r(0,0) = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
    m_r(0,1) = 2.0f * qx * qy - 2.0f * qz * qw;
    m_r(0,2) = 2.0f * qx * qz + 2.0f * qy * qw;
    m_r(0,3) = input.getOrigin().x();
    m_r(1,0) = 2.0f * qx * qy + 2.0f * qz * qw;
    m_r(1,1) = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
    m_r(1,2) = 2.0f * qy * qz - 2.0f * qx * qw;
    m_r(1,3) = input.getOrigin().y();
    m_r(2,0) = 2.0f * qx * qz - 2.0f * qy * qw;
    m_r(2,1) = 2.0f * qy * qz + 2.0f * qx * qw;
    m_r(2,2) = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;
    m_r(2,3) = input.getOrigin().z();
    m_r(3,0) = 0.0f;
    m_r(3,1) = 0.0f;
    m_r(3,2) = 0.0f;
    m_r(3,3) = 1.0f;
    return m_r;
}

tf::Transform Receiver::find_the_nearest_transform(std::vector<tf::Transform> &trasform_list)
{
    std::vector<int> index_list;
    index_list.resize(0);
    std::vector<tf::Transform> trasform_2B_list;
    for(int i = 0; i < trasform_list.size(); i++)
    {
        tf::Transform transform = Add_transform(trasform_list.at(i));
        cout <<"(" << transform.getOrigin().x()  <<"," <<transform.getOrigin().y() <<","<< transform.getOrigin().z() <<")" << endl;
        double sum = transform.getOrigin().x()*transform.getOrigin().x() + transform.getOrigin().y()*transform.getOrigin().y() + transform.getOrigin().z()*transform.getOrigin().z();
        if(0.64 < sum){ continue;}
        trasform_2B_list.push_back(transform);
        index_list.push_back(i);
    }

    int the_nearest_index = 0;
    double the_current_nearst_distance = 1000000000;
    if(trasform_2B_list.size() == 0)
    {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0, 0, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);

        img_x = 0;
        img_y = 0;
        
        return transform;
    }

    cout << "The size is " << trasform_2B_list.size() << endl;
    for(int i = 0; i < trasform_2B_list.size(); i++)
    {     
        double distance = trasform_2B_list.at(i).getOrigin().y()* trasform_2B_list.at(i).getOrigin().y();
        cout << "The y is " << distance << endl;
        
        if(distance < the_current_nearst_distance)
        {
            the_current_nearst_distance = distance;
            the_nearest_index = i;
        }
    }

    cout << "The nearest one is " << the_nearest_index << endl;
    cout << "The nearest is " << the_current_nearst_distance << endl;
    geometry_msgs::Pose2D saved_pose2d;
    saved_pose2d = temp_saved_pose2d_list.at(index_list.at(the_nearest_index));
    img_x = saved_pose2d.x;
    img_y = saved_pose2d.y;
    return trasform_list.at(the_nearest_index);
}


tf::Transform Receiver::find_center_point(double img_x, double img_y, cv::Mat &depth)
{
    double x_sum, y_sum, z_sum, x_cent, y_cent, z_cent;
    geometry_msgs::PointStamped ptMsg_c;
    
    x_cent = y_cent = z_cent = 0;

    const pcl::PointXYZRGBA& pt_c = cloud->points[img_y * depth.cols + img_x];
    ptMsg_c.point.x = pt_c.x;
    ptMsg_c.point.y = pt_c.y;
    ptMsg_c.point.z = pt_c.z;
    if (ptMsg_c.point.z <= Upper_Bonudary && ptMsg_c.point.z >= Lower_Boundary)
    {
        goto here;
    }


    for(int u = (img_x-choseLocal_size); u < (img_x+choseLocal_size); u++)
    {
        for(int v = (img_y-choseLocal_size); v < (img_y+choseLocal_size); v++)
        {
            if(u <= 0 || v <= 0 || u >= depth.cols || v >= depth.rows) continue;
    
            const pcl::PointXYZRGBA& pt_c = cloud->points[v * depth.cols + u];
            ptMsg_c.point.x = pt_c.x;
            ptMsg_c.point.y = pt_c.y;
            ptMsg_c.point.z = pt_c.z;

            // if( !std::isnan(ptMsg_c.point.x) && !std::isnan(ptMsg_c.point.y) && !std::isnan(ptMsg_c.point.z))
            if( !std::isnan(ptMsg_c.point.z))
            {   
                if (ptMsg_c.point.z <= Upper_Bonudary && ptMsg_c.point.z >= Lower_Boundary)
                    goto here;     
            }
        }
    }
    for(int u = (img_x-choseLocal_size2); u < (img_x+choseLocal_size2); u++)
    {
        for(int v = (img_y-choseLocal_size2); v < (img_y+choseLocal_size2); v++)
        {
            if(u <= 0 || v <= 0 || u >= depth.cols || v >= depth.rows) continue;

            const pcl::PointXYZRGBA& pt_c = cloud->points[v * depth.cols + u];
            ptMsg_c.point.x = pt_c.x;
            ptMsg_c.point.y = pt_c.y;
            ptMsg_c.point.z = pt_c.z;

            // if( !std::isnan(ptMsg_c.point.x) && !std::isnan(ptMsg_c.point.y) && !std::isnan(ptMsg_c.point.z))
            if( !std::isnan(ptMsg_c.point.z))
            {   
                if (ptMsg_c.point.z <= Upper_Bonudary && ptMsg_c.point.z >= Lower_Boundary)
                    goto here;
            }
        }

    }

    for(int k = (img_x-3); k < (img_x+3); k++)
    {
        for(int v = (img_y-3); v < (img_y+3); v++)
        {
            if(k <= 0 || v <= 0 || k >= depth.cols || v >= depth.rows) continue;
    
            const pcl::PointXYZRGBA& pt_c = cloud->points[v * depth.cols + k];
            ptMsg_c.point.x = pt_c.x;
            ptMsg_c.point.y = pt_c.y;
            ptMsg_c.point.z = pt_c.z;

            // if( !std::isnan(ptMsg_c.point.x) && !std::isnan(ptMsg_c.point.y) && !std::isnan(ptMsg_c.point.z))
            if( !std::isnan(ptMsg_c.point.z))
            {   
                if (ptMsg_c.point.z <= Upper_Bonudary && ptMsg_c.point.z >= Lower_Boundary)
                    goto here;
            }
        }
    }

    here:
        x_cent = ptMsg_c.point.x;
        y_cent = ptMsg_c.point.y;
        z_cent = ptMsg_c.point.z;
    
    if(std::isnan(z_cent)) 
    {
        x_cent = 10;
        y_cent = 10;
        z_cent = 10;
    }
  
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x_cent, y_cent, z_cent));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);

    return transform;
}



void Receiver::readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
{
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
}

void Receiver::readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
{
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
        *itC = cameraInfo->K[i];
    }
}

void Receiver::createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
{
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
        pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
        const uint16_t *itD = depth.ptr<uint16_t>(r);
        const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
        const float y = lookupY.at<float>(0, r);
        const float *itX = lookupX.ptr<float>();

        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
        {
        register const float depthValue = *itD / 1000.0f;
        // Check for invalid measurements
        if(*itD == 0)
        {
            // not valid
            itP->x = itP->y = itP->z = badPoint;
            itP->rgba = 0;
            continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 255;
        }
    }
}

void Receiver::createLookup(size_t width, size_t height)
{
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
        *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
        *it = (c - cx) * fx;
    }
}
