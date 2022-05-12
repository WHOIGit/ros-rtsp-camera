/*
 License: BSD
   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
*/

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <rtsp_camera/rtsp_camera.hpp>

RtspCamera::RtspCamera(ros::NodeHandle& n, const std::string& video_stream_url)
  : nh_(n), video_stream_address_(video_stream_url)
{
  image_transport::ImageTransport it(nh_);
  pub_video_ = it.advertise("image", 1);
  pub_camera_info_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
}

RtspCamera::~RtspCamera()
{
  vcap_.release();
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

bool RtspCamera::hasSubscribers()
{
  return pub_video_.getNumSubscribers() > 0
      || pub_camera_info_.getNumSubscribers() > 0;
}


void RtspCamera::spin()
{
  cv::Mat mat;
  sensor_msgs::CameraInfo ci;
  sensor_msgs::Image ros_img;

  while(ros::ok())
  {
    // Allow ROS to do some work
    ros::spinOnce();

    // Connect or disconnect to the stream according to the subscriber count
    if (hasSubscribers() && !vcap_.isOpened()) {
      ROS_INFO("Connecting to video stream");
      vcap_.open(video_stream_address_);
    } else if (!hasSubscribers() && vcap_.isOpened()) {
      ROS_INFO("Disconnecting from video stream");
      vcap_.release();
    }

    // If the stream is not connected, do not publish anything. Sleep briefly
    // so that we do not consume too much CPU.
    if (!vcap_.isOpened()) {
      ros::Duration(0.1).sleep();
      continue;
    }

    // Attempt to read a frame from the stream
    if (!vcap_.read(mat))
      continue;

    convertCvToRosImg(mat, ros_img, ci);
    pub_video_.publish(ros_img);
    pub_camera_info_.publish(ci);
  }
}
