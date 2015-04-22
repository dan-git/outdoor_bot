#include <ros/ros.h>
#include "outdoor_bot/NavTargets.h"
#include "std_msgs/String.h"
#include <string>
#include <stdio.h>
#include <stdlib.h>  
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdarg.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gphoto2/gphoto2-camera.h>
#include <iostream>  

int main (int argc, char *argv[])
{
   //Initialize ROS
   ros::init(argc, argv, "camcontrol");
   ros::NodeHandle nh;
   ROS_INFO("control starting");
   ros::ServiceClient client = nh.serviceClient<outdoor_bot::NavTargets>("NavTargetsService");
   outdoor_bot::NavTargets srv;
   srv.request.filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0";
   if (client.call(srv))
   {
      ROS_INFO("Sum: %ld", (long int)srv.response.sum);
   }
   else
   {
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
   }

   while(nh.ok()) ros::spinOnce();  // check for incoming messages
   return EXIT_SUCCESS;
}
