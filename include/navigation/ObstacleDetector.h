#ifndef __OUTDOOR_BOT_NAV_OBSTACLE_DETECTOR_H__
#define __OUTDOOR_BOT_NAV_OBSTACLE_DETECTOR_H__

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace OutdoorBot
{
namespace Navigation
{

class ObstacleDetector
{
 public:
  ObstacleDetector();

  bool obstacleInRectangle(double xL, double yL, double angle) const;

 private:
  void laserCallback(const sensor_msgs::LaserScan msg);

  mutable boost::mutex lock_;
  ros::Subscriber laserSub_;
  sensor_msgs::LaserScan lastMsg_;
  int min_obstacle_points_;
};

}  // namespace Navigation
}  // namespace OutdoorBot

#endif  // __OUTDOOR_BOT_NAV_OBSTACLE_DETECTOR_H__
