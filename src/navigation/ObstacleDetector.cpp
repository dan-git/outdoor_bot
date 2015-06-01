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

bool ObstacleDetector::obstacleInRectangle(double xL, double yL, double theta) const
{
  return false;

  lock_.lock();
  double angle_min = lastMsg_.angle_min;
  double angle_increment = lastMsg_.angle_increment;
  std::vector<float> ranges = lastMsg_.ranges;
  lock_.unlock();

  int points_within_rectangle = 0;
  for (size_t i = 0; i < ranges.size(); i++)
  {
    if (!std::isfinite(ranges[i]))
    {
      continue;
    }
    double phi = angle_min + i * angle_increment;
    // x' and y' are the coordinates of the point in the coordinate system rotated at angle theta to the robot.
    double xp = ranges[i] * cos(phi - theta);
    double yp = ranges[i] * sin(phi - theta);
    if (fabs(xp) < xL && fabs(yp) < yL / 2.0)
    {
      points_within_rectangle++;
    }
  }
  return points_within_rectangle >= min_obstacle_points_;
}

void ObstacleDetector::laserCallback(const sensor_msgs::LaserScan msg)
{
  lock_.lock();
  lastMsg_ = msg;
  lock_.unlock();
}

}  // namespace Navigation
}  // namespace OutdoorBot
