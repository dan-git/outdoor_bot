#ifndef __OUTDOOR_BOT_NAV_WALL_FOLLOWER_H__
#define __OUTDOOR_BOT_NAV_WALL_FOLLOWER_H__

#include "FBFSM/FBFSM.h"
#include "navigation/NavTypes.h"
#include "navigation/ObstacleDetector.h"

namespace OutdoorBot
{
namespace Navigation
{

class WallFollower
{
 public:
  enum Side
  {
    LEFT  = 0,
    RIGHT = 1
  };
 public:
  WallFollower();

  void activate(const Side& side, double linear_velocity, double angular_velocity);
  Velocity update(const Pose2D& robot_pose);

 private:
  void setupFSM();

  void on_enter_move_forward();
  int on_update_move_forward();

  void on_enter_wait_for_obstacle_clear();
  int on_update_wait_for_obstacle_clear();

  void on_enter_turn_away();
  int on_update_turn_away();

  void on_enter_turn_into();
  int on_update_turn_into();

  FBFSM fsm_;
  int move_forward_state_;
  int wait_for_obstacle_clear_state_;
  int turn_away_state_;
  int turn_into_state_;

  struct MoveForwardData
  {
    int no_wall_detections;
    int forward_obstacle_detections;
    MoveForwardData() : no_wall_detections(0), forward_obstacle_detections(0) {}
  } move_forward_data_;

  struct WaitForObstacleClearData
  {
    ros::WallTime start_time;
    int no_obstacle_detections;

    WaitForObstacleClearData() : no_obstacle_detections(0) {}
  } wait_for_obstacle_clear_data_;

  struct TurnData
  {
    double starting_angle;
  };

  TurnData turn_into_data_;
  TurnData turn_away_data_;

  struct Params
  {
    // If an obstacle is this close to the front of the robot, stop and wait for it to clear.  If it doesn't, turn
    // and wall follow.
    double obstacle_distance;
    // While wall following the wall should be within obstacle_distance_ + wall_distance_ from the robot's side or
    // the robot will turn.
    double wall_distance;
    // We check for obstacles in a cone that has a base of 2 * robot_radius_.
    double robot_radius;
    // In order to decide there is truly an obstacle(not a wall) we must get this many obstacle(clear) detections in a
    // row.
    int max_obstacle_detections;
    // The time to wait and see if obstacles go away.
    double wait_for_obstacle_clear_duration;
    Params()
        : obstacle_distance(1.5),
          wall_distance(1.0),
          robot_radius(0.5),
          max_obstacle_detections(3),
          wait_for_obstacle_clear_duration(30.0)
    {}
  } params_;

  ObstacleDetector obstacle_detector_;
  Side side_;
  Velocity velocity_;
  double linear_velocity_;
  double angular_velocity_;
  Pose2D robot_pose_;
};

}  // namespace Navigation
}  // namespace OutdoorBotx

#endif  // __OUTDOOR_BOT_NAV_WALL_FOLLOWER_H__
