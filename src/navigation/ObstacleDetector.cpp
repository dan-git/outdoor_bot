#include <numeric>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include "outdoor_bot/ObstacleDetectorFeedback_msg.h"

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
  feedback_pub_ = private_nh.advertise<outdoor_bot::ObstacleDetectorFeedback_msg>("obstacle_detector/feedback", 10);
  points_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("obstacle_detector/points", 100);
}

void ObstacleDetector::activate(const DetectionParamsList& params)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("obstacle_detector/min_obstacle_points", min_obstacle_points_, 50);
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
  state_.last_detection_time = curr_time;
}

bool ObstacleDetector::obstacleInRectangle(double xL, double yL, double theta) const
{
  geometry_msgs::PoseArray points;

  lock_.lock();
  double angle_min = lastMsg_.angle_min;
  double angle_increment = lastMsg_.angle_increment;
  std::vector<float> ranges = lastMsg_.ranges;
  points.header = lastMsg_.header;
  lock_.unlock();

  if (points.header.frame_id.empty())
  {
    points.header.frame_id = "/map";
  }

  points.poses.reserve(ranges.size());

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
      geometry_msgs::Pose pose;
      pose.position.x = ranges[i] * cos(phi);
      pose.position.y = ranges[i] * sin(phi);
      pose.position.z = 0.0;
      pose.orientation.w = 1.0;
      points.poses.push_back(pose);
    }
  }
  outdoor_bot::ObstacleDetectorFeedback_msg feedback;
  feedback.points_within_rectangle = points_within_rectangle;
  feedback_pub_.publish(feedback);
  points_pub_.publish(points);
  return points_within_rectangle >= min_obstacle_points_;
}

void ObstacleDetector::getObstacles(
    const outdoor_bot::ObstacleParams_msg& params, std::vector<outdoor_bot::Obstacle_msg>* obstacles) const
{
  lock_.lock();
  std::vector<float> ranges = lastMsg_.ranges;
  double angle_min = lastMsg_.angle_min;
  double angle_increment = lastMsg_.angle_increment;
  lock_.unlock();

  double avg_distance = 0.0;
  int n_finite_ranges = 0;
  for (size_t i = 0; i < ranges.size(); i++)
  {
    if (std::isfinite(ranges[i])  && (fabs(params.max_distance < 0.001) || ranges[i] < params.max_distance))
    {
      double angle = angle_min + i * angle_increment;
      if ((fabs(params.min_angle) < 0.001 || fabs(params.max_angle) < 0.001) ||
          (angle >= params.min_angle && angle <= params.max_angle))
      {
        avg_distance += ranges[i];
        n_finite_ranges++;
      }
    }
  }

  if (n_finite_ranges > 0)
  {
    outdoor_bot::Obstacle_msg obstacle;
    obstacle.distance = avg_distance / static_cast<double>(n_finite_ranges);
    obstacles->push_back(obstacle);
  }
}

void ObstacleDetector::laserCallback(const sensor_msgs::LaserScan msg)
{
  lock_.lock();
  lastMsg_ = msg;
  lock_.unlock();
}

}  // namespace Navigation
}  // namespace OutdoorBot
