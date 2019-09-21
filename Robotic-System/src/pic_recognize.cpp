#include "pic_recognize.h"

std::atomic_int mouseX;
std::atomic_int mouseY;
std::atomic_int mouseBtnType;
ros::Publisher leftBtnPointPub;

geometry_msgs::Pose2D saved_pose2d;

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

void Origin_3DcallBack(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    saved_pose2d = *msg;
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
  int img_x = mouseX;
  int img_y = mouseY;
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

      double count;

      static ros::Subscriber sub_2D = nh.subscribe("/IRIM/pub_2D", 1, Origin_3DcallBack);
      img_x = saved_pose2d.x;
      img_y = saved_pose2d.y;

    //   img_x = mouseX;
    //   img_y = mouseY;

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

      
      x_cent = y_cent = z_cent = 0;

      count = 0.1;

        const pcl::PointXYZRGBA& pt_c = cloud->points[img_y * depth.cols + img_x];
        ptMsg_c.point.x = pt_c.x;
        ptMsg_c.point.y = pt_c.y;
        ptMsg_c.point.z = pt_c.z;
        if (ptMsg_c.point.z <= Upper_Bonudary && ptMsg_c.point.z >= Lower_Boundary)
                goto here;

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

        for(int u = (img_x-3); u < (img_x+3); u++)
        {
            for(int v = (img_y-3); v < (img_y+3); v++)
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
   
        here:
            x_cent = ptMsg_c.point.x;
            y_cent = ptMsg_c.point.y;
            z_cent = ptMsg_c.point.z;
        
        if(std::isnan(z_cent)) 
        {
            x_cent = 0;
            y_cent = 0;
            z_cent = 0;
        }
        ossXYZ_c.str("");
        ossXYZ_c << "After Process :( " << x_cent<< ", " << y_cent
                                    << ", " <<z_cent << " )";

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x_cent, y_cent, z_cent));

        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
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
