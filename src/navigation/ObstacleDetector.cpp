#include <numeric>
#include <vector>

#include "navigation/ObstacleDetector.h"


namespace OutdoorBot
{
namespace Navigation
{

double wrapAngle(double angle)
{
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  return angle;
}

ObstacleDetector::ObstacleDetector()
    : min_obstacle_points_(4)
{
  ros::NodeHandle nh;
  laserSub_ = nh.subscribe("scan", 10, &ObstacleDetector::laserCallback, this);
}

bool ObstacleDetector::obstacleInCone(double cone_height, double cone_radius, double angle_in) const
{
  double angle = wrapAngle(angle_in);
  double cone_angle = atan2(cone_radius, cone_height);
  double starting_angle = wrapAngle(angle - cone_angle);
  double ending_angle = wrapAngle(angle + cone_angle);
  if (starting_angle > ending_angle)
  {
    double tmp = ending_angle;
    ending_angle = starting_angle;
    starting_angle = tmp;
  }
  // Make local copies of everything in case a new message comes in while we're doing this.
  lock_.lock();
  double angle_min = lastMsg_.angle_min;
  double angle_increment = lastMsg_.angle_increment;
  std::vector<float> ranges = lastMsg_.ranges;
  lock_.unlock();

  int points_within_cone = 0;
  for (size_t i = 0; i < ranges.size(); i++)
  {
    if (!std::isfinite(ranges[i]))
    {
      continue;
    }
    double current_angle = wrapAngle(angle_min + i * angle_increment);
    if (current_angle >= starting_angle && current_angle <= ending_angle)
    {
      if (ranges[i] <= cone_height)
      {
        points_within_cone++;
      }
    }
  }
  return points_within_cone >= min_obstacle_points_;
}

void ObstacleDetector::laserCallback(const sensor_msgs::LaserScan msg)
{
  lock_.lock();
  lastMsg_ = msg;
  lock_.unlock();
}

}  // namespace Navigation
}  // namespace OutdoorBot
