#ifndef __OUTDOOR_BOT_NAV_WALL_FOLLOWER_H__
#define __OUTDOOR_BOT_NAV_WALL_FOLLOWER_H__

#include "FBFSM/FBFSM.h"
#include "navigation/ObstacleDetector.h"

namespace OutdoorBot
{
namespace Navigation
{

class WallFollower
{
 public:
  class Output
  {
   public:
    /**
     * The modes are
     *  TIMEOUT: A turn, move, or wait for stop timed out.  The obstacle avoidance failed.
     *  SUCCESSFUL_COMPLETION: The turns have all canceled out and the robot is back on its original heading.  The
     *   obstacle avoidance succeeded.
     *  WAIT_FOR_READY: The wall follower has issued a move or stop command and is waiting for the robot to report
     *   successful completion of this command.
     *  OBSTACLE_AHEAD: An obstacle is detected in front of the robot.  Slam on the brakes now no matter what you were
     *   doing before.
     *  MOVE_FORWARD: Move forward a certain distance.
     *  TURN: Turn a certain distance.
     *  STOP: Stay stopped.  This is only issued after EMERGENCY_STOP while the robot is waiting to see if an obstacle
     *   in front of it will leave.
     */
    enum Mode
    {
      TIMEOUT               = -1,
      SUCCESSFUL_COMPLETION = 0,
      WAIT_FOR_READY        = 1,
      OBSTACLE_AHEAD        = 2,
      MOVE_FORWARD          = 3,
      TURN                  = 4,
      STOP                  = 5
    };
 public:
    Output() : mode_(WAIT_FOR_READY), distance_(0.0) {}

    const Mode& mode() const {return mode_;}
    double distance() const {return distance_;}
    void set_mode(const Mode& mode) {mode_ = mode;}
    void set_distance(double distance) {distance_ = distance;}
   private:
    Mode mode_;
    double distance_;
  };

  class Goal
  {
   public:
    enum Side
    {
      LEFT  = 0,
      RIGHT = 1
    };
   public:
    /**
     * This will start by turning 90 degrees to SIDE.
     */
    explicit Goal(const Side& side) : side_(side) {}

    const Side& side() const {return side_;}

   private:
    Side side_;
  };

  class Input
  {
   public:
    /**
     * When the input mode is EXECUTING_COMMAND, this will only return the  WAIT_FOR_READY or EMERGENCY_STOP modes.
     * When the input mode is READY_FOR_NEW_COMMAND, this may return any of the output modes.
     */
    enum Mode
    {
      EXECUTING_COMMAND     = 0,
      READY_FOR_NEW_COMMAND = 1
    };
   public:
    explicit Input(const Mode& mode) : mode_(mode) {}

    const Mode& mode() const {return mode_;}

   private:
    Mode mode_;
  };

 public:
  WallFollower();

  void activate(const Goal& goal);
  Output update(const Input& input);

 private:
  void setupFSM();

  void on_enter_command_move_forward();
  int on_update_command_move_forward();

  void on_enter_move_forward();
  int on_update_move_forward();

  void on_enter_wait_for_obstacle_clear();
  int on_update_wait_for_obstacle_clear();

  void on_enter_command_turn_away();
  int on_update_command_turn_away();

  void on_enter_command_turn_into();
  int on_update_command_turn_into();

  void on_enter_wait_for_turn();
  int on_update_wait_for_turn();

  void on_enter_successful_completion();
  int on_update_successful_completion();

  void on_enter_prepare_for_timeout();
  int on_update_prepare_for_timeout();

  void on_enter_timeout();
  int on_update_timeout();

  FBFSM fsm_;
  int command_move_forward_state_;
  int move_forward_state_;
  int wait_for_obstacle_clear_state_;
  int command_turn_away_state_;
  int command_turn_into_state_;
  int wait_for_turn_state_;
  int successful_completion_state_;
  int prepare_for_timeout_state_;
  int timeout_state_;

  struct MoveForwardData
  {
    int no_wall_detections;
    int forward_obstacle_detections;
    ros::WallTime start_time;
    MoveForwardData() : no_wall_detections(0), forward_obstacle_detections(0) {}
  } move_forward_data_;

  struct WaitForObstacleClearData
  {
    ros::WallTime start_time;
    int no_obstacle_detections;

    WaitForObstacleClearData() : no_obstacle_detections(0) {}
  } wait_for_obstacle_clear_data_;

  struct WaitForTurnData
  {
    ros::WallTime start_time;
  } wait_for_turn_data_;

  struct Params
  {
    // The dimensions of the rectangle to check for obstacles.  The x parameter is the distance "in front" of the robot
    // and the y parameter is the distance perpendicular.
    double obstacle_rect_x, obstacle_rect_y;
    // In order to decide there is truly an obstacle(not a wall) we must get this many obstacle(clear) detections in a
    // row.
    int max_obstacle_detections;
    // The time to wait and see if obstacles go away.
    double wait_for_obstacle_clear_duration;
    // The angle (in rad) at which to check for obstacles on the side.
    double side_angle;
    // How far the robot should move before checking to see if it can turn back.
    double incremental_distance;
    // Timeout for forward moves (-1 for no timeout).
    double move_timeout;
    // Timeout for turns (-1 for no timeout).
    double turn_timeout;
    Params()
        : obstacle_rect_x(2.0),
          obstacle_rect_y(1.3),
          max_obstacle_detections(3),
          wait_for_obstacle_clear_duration(30.0),
          side_angle(1.0),
          incremental_distance(2.0),
          move_timeout(-1.0),
          turn_timeout(-1.0)
    {}
  } params_;

  struct State
  {
    double current_angle;

    State() : current_angle(0.0) {}
  } state_;

  ObstacleDetector obstacle_detector_;
  Goal goal_;
  Input input_;
  Output output_;
};

}  // namespace Navigation
}  // namespace OutdoorBotx

#endif  // __OUTDOOR_BOT_NAV_WALL_FOLLOWER_H__
