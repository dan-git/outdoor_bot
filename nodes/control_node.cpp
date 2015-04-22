#include <ros/ros.h>
#include "outdoor_bot/NavTargets_service.h"
#include "outdoor_bot/NavTargets_msg.h"
#include "std_msgs/String.h"
/*
#include <string>
#include <stdio.h>
#include <stdlib.h>  
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
*/
#include "outdoor_bot/digcams_custom.h"
#include <iostream>  

using namespace std;

ros::Publisher digcam_pub_, motion_pub_;
int centerX_, centerY_;
int home_image_height_, home_image_width_;
bool homeCenterUpdated_;
float range_;

void homeCenterCallback(const outdoor_bot::NavTargets_msg::ConstPtr &msg)
{
   centerX_ = msg->centerX;
   centerY_ = msg->centerY;
   range_ = msg->range;
   homeCenterUpdated_ = true;
   ROS_INFO("home updated in control: center, range = (%d, %d), %f", centerX_, centerY_, range_);
}

void headForFirstTarget()
{
   // move off the platform and straight forward 10m
   // take photo
   // do we have the target in sight
      // if so, estimate range, if < 20m go to approach target routine
      // otherwise, center on it and then go out 10m and loop to take photo
   // if not, sweep camera to find target, if found loop to "we have the target in sight"
   // if not found, use dead reckoning to get to within about 20m of known location and loop to sweep camera
}

void approachTarget()
{
   // center target, estimate range
   // move in until target is < 1m
}

void captureTarget()
{
   // center target, estimate range
   // lower front loader
   // move forward to target contact with loader
   // decide if drop bar is needed, if so, drop it and move forward to engage it
   // lift loader, move shade to next bin slot
   // verify that target is in loader
   // if drop bar deployed, move in reverse and wind it back up
   // dump target into bin
   // move forward 10m or perhaps farther to find flat ground and to get out of target area range
   // verify that target is in bin
}

void searchForTarget()
{
   // sweep cameras
   // if target found, go to approach target
   // rotate 90 degrees and loop to sweep cameras
   // rotate 180 degrees and loop to sweep cameras
}   
   
void headForHome()
{
   std_msgs::String msgMotion;
   char cmd[32];

   // first find a home target
   std::string filename = "cap_home";
   outdoor_bot::digcams_custom msg;
   msg.command = filename;
   digcam_pub_.publish(msg);
   ROS_INFO("control command to capture was published");
   if (homeCenterUpdated_)
   {
      homeCenterUpdated_ = false;
      // center the target and move forward
      double offsetX = ((double) (home_image_width_ - centerX_)) / ((double) home_image_width_);
      //double offsetY = ((double) (home_image_width_ - centerY_)) / ((double) home_image_height_);
      if (fabs(offsetX) > 0.1)
      {
         if (range_ > 20.)
         {
            //move forward 10m and turn by the angle needed to center the target in X
            ROS_INFO("moving forward 10m and centering the home target");
         }
         else
         {
            // turn by the angle needed to center the target in X
            ROS_INFO("centering the home target");
         }
      }
      else
      {
         if (range_ > 20.)
         {
            //move forward 10m
            cmd[0] = 'n';
            cmd[1] = 0; // end with null character
            msgMotion.data = cmd;
            motion_pub_.publish(msg);
            ROS_INFO("moving forward 10m");
         }
         else if (range_ > 10.)
         {
            //move forward 5 m
            ROS_INFO("moving forward 5m");
         }
         else if (range_ > 5.)
         {
            //move forward 3 m
            ROS_INFO("moving forward 5m");
         }
         else 
         {
            // final approach
            ROS_INFO("moving to home spot");
         }
      }
   }
}
   

int main (int argc, char *argv[])
{
   //Initialize ROS
   ros::init(argc, argv, "control");
   ros::NodeHandle nh;
   
   ros::Subscriber home_center;
   ROS_INFO("control starting");
   centerX_ = -1;
   centerY_ = -1;
   homeCenterUpdated_ = false;
   int ready_to_go;

   home_center = nh.subscribe("Home_target_center", 10, homeCenterCallback); 
   digcam_pub_ = nh.advertise<outdoor_bot::digcams_custom>("digcam_cmd", 50); // advertise camera files
   motion_pub_ = nh.advertise<std_msgs::String>("motion_command", 1);

   
   // set up message to capture an image with camera 0
   std::string filename = "capture";
   outdoor_bot::digcams_custom msg;
   msg.command = filename;
   msg.camera_number = 0;

   ready_to_go = 1;
   while (ready_to_go > 0)
   {
      cout << "input a command, 0 to quit: " << endl;
      cin >> ready_to_go;
      if (ready_to_go > 1)
      {
         digcam_pub_.publish(msg);
         ROS_INFO("control command to capture was published");
      }
      ros::spinOnce();  // check for incoming messages
   }

   
   /*
   ros::ServiceClient client = nh.serviceClient<outdoor_bot::NavTargets_service>("NavTargetsService");
   outdoor_bot::NavTargets_service srv;
   srv.request.image_filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0.jpg";
   cout << "waiting for input" << endl;
   cin >> ready_to_go;
   if (ready_to_go > 1)
   {   
      if (client.call(srv))
      {
         int x = srv.response.centerX;
         int y = srv.response.centerY;
         float range = srv.response.range;
         ROS_INFO("centerX, centerY, range = %d, %d, %f", x, y, range);
      }
      else
      {
         ROS_ERROR("Failed to call service NavTargets");
      }
   }
 
   while(nh.ok()) ros::spinOnce();  // check for incoming messages
   */   

   return EXIT_SUCCESS;
}
