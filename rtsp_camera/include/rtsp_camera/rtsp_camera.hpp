/*
 License: BSD
   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
*/

#ifndef RTSP_CAMERA
#define RTSP_CAMERA

#include <string>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>


class RtspCamera {
  public:
    RtspCamera(ros::NodeHandle& n, const std::string& video_stream_url);
    ~RtspCamera();

    void spin();
  
  protected:
    bool hasSubscribers();
    void convertCvToRosImg(const cv::Mat& mat, sensor_msgs::Image& ros_img, sensor_msgs::CameraInfo& ci);

  private:
    cv::VideoCapture                vcap_;
    std::string                     video_stream_address_;

    image_transport::Publisher pub_video_;
    ros::Publisher pub_camera_info_;
    ros::NodeHandle nh_;
};

#endif
