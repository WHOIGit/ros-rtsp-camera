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

  std::string video_stream_url;
  pnh.getParam("video_stream_url", video_stream_url);

  RtspCamera rtsp(pnh, video_stream_url);
  rtsp.spin();

  return 0;
}
