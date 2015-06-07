#ifndef __OUTDOOR_BOT_NAV_OBSTACLE_DETECTOR_H__
#define __OUTDOOR_BOT_NAV_OBSTACLE_DETECTOR_H__

#include <vector>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "outdoor_bot/Obstacle_msg.h"
#include "outdoor_bot/ObstacleParams_msg.h"

namespace OutdoorBot
{
namespace Navigation
{

class ObstacleDetector
{
 public:
  struct DetectionParams
  {
    double xL;
    double yL;
    double theta;
    bool count_misses;

    DetectionParams() : xL(0.0), yL(0.0), theta(0.0), count_misses(false) {}
    DetectionParams(double xL_, double yL_, double theta_, bool count_misses_)
        : xL(xL_),
          yL(yL_),
          theta(theta_),
          count_misses(count_misses_)
    {}
  };
  typedef std::vector<DetectionParams> DetectionParamsList;
 public:
  ObstacleDetector();

  /**
   * Starts the in front obstacle detection.
   *
   * @param distance The distance in front of the robot to check for obstacles.
   * @param robot_radius The radius of the robot (half of the rectangle width).
   */
  void activate(double distance, double robot_radius)
  {activate(DetectionParamsList(1, DetectionParams(distance, 2.0 * robot_radius, 0.0, false)));}
  /**
   * Starts the any angle obstacle detection.
   *
   * @param distance The distance in front of the robot to check for obstacles.
   * @param robot_radius The radius of the robot (half of the rectangle width).
   */
  void activate(const DetectionParamsList& params);
  /**
   * Uses time filtering to decide if there is an obstacle in front of the robot.
   *
   * This calls obstacleInRectangle(distance_, 2 * robot_radius_, 0.0) on each call.  If max_obstacle_detection_ calls
   * in a row return true, this returns that there is an obstacle in front of the robot.
   *
   * @return True if there is an obstacle in front of the robot.
   */
  bool update();
  void update(std::vector<bool>* results);
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

  void getObstacles(const outdoor_bot::ObstacleParams_msg& params, std::vector<outdoor_bot::Obstacle_msg>* obstacles)
      const;

 private:
  void laserCallback(const sensor_msgs::LaserScan msg);

  mutable boost::mutex lock_;
  ros::Subscriber laserSub_;
  sensor_msgs::LaserScan lastMsg_;
  int min_obstacle_points_;
  int max_obstacle_detections_;

  struct State
  {
    ros::Time last_detection_time;
  } state_;

  struct DetectionState
  {
    DetectionParams params;
    int obstacle_detections_or_misses;
    explicit DetectionState(const DetectionParams& params_) : params(params_), obstacle_detections_or_misses(0) {}
  };

  std::vector<DetectionState> detection_states_;
};

}  // namespace Navigation
}  // namespace OutdoorBot

#endif  // __OUTDOOR_BOT_NAV_OBSTACLE_DETECTOR_H__
