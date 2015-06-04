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
  private_nh.param("max_obstacle_detections", max_obstacle_detections_, 3);
}

void ObstacleDetector::activate(double distance, double robot_radius)
{
  state_ = State();
  state_.distance = distance;
  state_.robot_radius = robot_radius;
}

bool ObstacleDetector::update()
{
  if (obstacleInRectangle(state_.distance, 2.0 * state_.robot_radius, 0.0))
  {
    state_.front_obstacle_detections++;
  }
  else
  {
    state_.front_obstacle_detections = 0;
  }
  return state_.front_obstacle_detections > 0; //max_obstacle_detections_; ***************************************************************
}

bool ObstacleDetector::obstacleInRectangle(double xL, double yL, double theta) const
{
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
   // std::cout << "analyzing points" << std::endl;
    if (fabs(xp) < xL && fabs(yp) < yL / 2.0)
    {
      points_within_rectangle++;
    }
  }
  return points_within_rectangle >= 5; // min_obstacle_points_;***********************************************************************
}

void ObstacleDetector::laserCallback(const sensor_msgs::LaserScan msg)
{
  lock_.lock();
  lastMsg_ = msg;
  lock_.unlock();
}

}  // namespace Navigation
}  // namespace OutdoorBot
