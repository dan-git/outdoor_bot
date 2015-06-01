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

  /**
   * Checks if an obstacle is within a defined rectangle.
   *
   * This checks if there is an obstacle within a rectangle at some angle to the robot.  For example, to check if there
   * is an obstacle within two meters in front of the robot with a clearance of 0.5 meters to each side (a rectangle of
   * dimension 2m in x and 1m in y), you would use obstacleInRectangle(2.0, 1.0, 0.0).  To check if there WOULD BE
   * an obstacle in that same rectangle if the robot turned 1.0 radians, use obstacleInRectangle(2.0, 1.0, 1.0).
   *
   * @param xL The x dimension of the rectangle.  This is the dimension in front of the robot.
   * @param yL The y dimension of the rectangle.  This is the dimension perpendicular to the robot.
   * @param theta The orientation of the rectangle.
   */
  bool obstacleInRectangle(double xL, double yL, double theta) const;

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
