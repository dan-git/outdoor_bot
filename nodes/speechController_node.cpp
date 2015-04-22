// controls a robot from speech commands

#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

ros::Subscriber speechCommandsSub_;
ros::Publisher key_pub_;
std_msgs::String msgSpeech_;

void speechCommandCallback(const std_msgs::String::ConstPtr& msg)
{
   std::string speechCommand = msg->data.c_str();
   msgSpeech_.data = "unrecognized command";
   if (speechCommand.compare("stop") == 0) msgSpeech_.data = "s";
   else if (speechCommand.compare("halt") == 0) msgSpeech_.data = "x";
   else if (speechCommand.compare("start") == 0) msgSpeech_.data = "s";
   else if (speechCommand.compare("right") == 0) msgSpeech_.data = "r";
   else if (speechCommand.compare("left") == 0) msgSpeech_.data = "l";
   else if (speechCommand.compare("forward") == 0) msgSpeech_.data = "f";
   else if (speechCommand.compare("backward") == 0) msgSpeech_.data = "b";
   else if (speechCommand.compare("patrol") == 0) msgSpeech_.data = "p";
   key_pub_.publish(msgSpeech_);
}

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "speech_commands");
  ros::NodeHandle nh;
  // subscribe to speech commands
  speechCommandsSub_ = nh.subscribe("recognizer/output", 1, speechCommandCallback);

  //publish as keyboard commands
  key_pub_ = nh.advertise<std_msgs::String>("keyboard", 1);
  ros::spin();
}
