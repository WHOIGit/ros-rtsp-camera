/*
 License: BSD
   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
*/

#include <ros/ros.h>
#include <rtsp_camera/rtsp_camera.hpp>

int main (int argc, char** argv) 
{
  ros::init(argc, argv, "rtsp_camera");
  ros::NodeHandle pnh("~");
  std::string video_stream_url, user, password;

  pnh.getParam("video_stream_url", video_stream_url);

  RtspCamera rtsp(pnh);
  ROS_INFO("Rtsp Camera : Initialising..");
  if(!rtsp.init(video_stream_url))
  {
    ROS_ERROR("Rtsp Camera : Failed to initialise stream");
    return -1;
  }

  ROS_INFO("Rtsp Camera : Initialised");
  rtsp.spin();
  ROS_INFO("Rtsp Camera : Bye Bye");

  return 0;
}
