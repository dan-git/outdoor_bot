#include <string>

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

class Republisher
{
 public:
  Republisher()
  {
    ros::NodeHandle nh;
    publisher_ = nh.advertise<sensor_msgs::LaserScan>("base_scan", 1000);
    subscriber_ = nh.subscribe("/scan", 10, &Republisher::callback, this);
  }

  void callback(const sensor_msgs::LaserScan msg)
  {
    sensor_msgs::LaserScan new_msg = msg;
    new_msg.header.frame_id = "map";
    publisher_.publish(new_msg);
  }

 private:
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "republish_scan_node");
  Republisher republisher;
  ros::spin();
}
