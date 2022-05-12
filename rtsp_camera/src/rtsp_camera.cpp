/*
 License: BSD
   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
*/

#include <rtsp_camera/rtsp_camera.hpp>

RtspCamera::RtspCamera(ros::NodeHandle& n) : nh_(n)
{
  image_transport::ImageTransport it(nh_);    
  pub_video_ = it.advertise("image", 1);
  pub_camera_info_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
  pub_status_ = nh_.advertise<std_msgs::String>("status", 1);
}

RtspCamera::~RtspCamera()
{
  vcap_.release();
}

bool RtspCamera::init(const std::string video_stream_url) {
  video_stream_address_ = video_stream_url;

  if (!vcap_.open(video_stream_address_)) 
    return false; 
  else
    return true;
}

bool RtspCamera::reset(const std::string video_stream_url)
{
  vcap_.release();
  return init(video_stream_url);
}

/*
  Convert cv::Mat to sensor_msgs:Image and CameraInfo
 */
void RtspCamera::convertCvToRosImg(const cv::Mat& mat, sensor_msgs::Image& ros_img, sensor_msgs::CameraInfo& ci)
{
  cv_bridge::CvImage cv_img;

  cv_img.encoding = sensor_msgs::image_encodings::BGR8;
  cv_img.image = mat;
  cv_img.toImageMsg(ros_img);
  ros_img.header.stamp = ros::Time::now();
  ci.header = ros_img.header;
  ci.width = ros_img.width;
  ci.height = ros_img.height;
  
  return;
}


void RtspCamera::spin()
{
  cv::Mat mat;
  sensor_msgs::CameraInfo ci;
  sensor_msgs::Image ros_img;
  std_msgs::String ros_str;

  while(ros::ok())
  {
    if(!vcap_.read(mat)) {
      status_ = "No frame from camera";
      cv::waitKey();
    }
    else {
      status_ = "live";
    }

    ros_str.data = status_;
    
    convertCvToRosImg(mat, ros_img, ci);
    pub_video_.publish(ros_img);
    pub_camera_info_.publish(ci);
    pub_status_.publish(ros_str);
    cv::waitKey(1);
  }
}
