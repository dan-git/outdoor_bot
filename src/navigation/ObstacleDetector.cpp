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
}

void ObstacleDetector::activate(const DetectionParamsList& params)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("obstacle_detector/min_obstacle_points", min_obstacle_points_, 4);
  private_nh.param("obstacle_detector/repeat_detections", max_obstacle_detections_, 3);

  ROS_INFO("Obstacle detection: Must see %d points %d times before registering an obstacle.",
           min_obstacle_points_, max_obstacle_detections_);

  state_ = State();
  detection_states_.clear();
  detection_states_.reserve(params.size());
  for (size_t i = 0; i < params.size(); i++)
  {
    detection_states_.push_back(DetectionState(params[i]));
  }
}

bool ObstacleDetector::update()
{
  std::vector<bool> results;
  update(&results);
  if (results.empty())
  {
    return false;
  }
  return results[0];
}

void ObstacleDetector::update(std::vector<bool>* results)
{
  lock_.lock();
  ros::Time curr_time = lastMsg_.header.stamp;
  lock_.unlock();
  results->clear();
  results->resize(detection_states_.size(), false);
  for (size_t i = 0; i < detection_states_.size(); i++)
  {
    if (curr_time == state_.last_detection_time)
    {
      // Nothing has changed.
      (*results)[i] = (detection_states_[i].obstacle_detections_or_misses >= max_obstacle_detections_);
      continue;
    }
    if (!detection_states_[i].params.count_misses)
    {
      // Counting detections - have to have a certain number to return true.
      if (obstacleInRectangle(
              detection_states_[i].params.xL, detection_states_[i].params.yL, detection_states_[i].params.theta))
      {
        detection_states_[i].obstacle_detections_or_misses++;
      }
      else
      {
        detection_states_[i].obstacle_detections_or_misses = 0;
      }
    }
    else
    {
      // Counting misses.  Have to have a certain number to return true.
      if (!obstacleInRectangle(
              detection_states_[i].params.xL, detection_states_[i].params.yL, detection_states_[i].params.theta))
      {
        detection_states_[i].obstacle_detections_or_misses++;
      }
      else
      {
        detection_states_[i].obstacle_detections_or_misses = 0;
      }
    }
    (*results)[i] = (detection_states_[i].obstacle_detections_or_misses >= max_obstacle_detections_);
  }
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
