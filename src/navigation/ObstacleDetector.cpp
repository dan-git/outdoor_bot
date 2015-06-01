#include <numeric>
#include <vector>

#include "navigation/NavUtils.h"
#include "navigation/ObstacleDetector.h"


namespace OutdoorBot
{
namespace Navigation
{

ObstacleDetector::ObstacleDetector()
{
  ros::NodeHandle nh;
  laserSub_ = nh.subscribe("scan", 10, &ObstacleDetector::laserCallback, this);
  ros::NodeHandle private_nh("~");
  private_nh.param("min_obstacle_points", min_obstacle_points_, 4);
}

bool ObstacleDetector::obstacleInRectangle(double xL, double yL, double angle) const
{
  return false;

  // lock_.lock();
  // double angle_min = lastMsg_.angle_min;
  // double angle_increment = lastMsg_.angle_increment;
  // std::vector<float> ranges = lastMsg_.ranges;
  // lock_.unlock();

  // int points_within_rectangle = 0;
  // for (size_t i = 0; i < ranges.size(); i++)
  // {
  //   if (!std::isfinite(ranges[i]))
  //   {
  //     continue;
  //   }
  //   double angle = angle_min + i * angle_increment;
  //   double x = ranges[i] * cos(angle);
  //   double y = ranges[i] * sin(angle);
  //   if (fabs(x) < distance && fabs(y) < width / 2.0)
  //   {
  //       points_within_rectangle++;
  //     }
  //   }
  // }
  // return points_within_rectangle >= min_obstacle_points_;

}

void ObstacleDetector::laserCallback(const sensor_msgs::LaserScan msg)
{
  lock_.lock();
  lastMsg_ = msg;
  lock_.unlock();
}

}  // namespace Navigation
}  // namespace OutdoorBot
