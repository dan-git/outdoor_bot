#ifndef __OUTDOOR_BOT_DIR_ANT_FOLLOWER_H__
#define __OUTDOOR_BOT_DIR_ANT_FOLLOWER_H__

#include <ros/ros.h>

#include "FBFSM/FBFSM.h"

namespace OutdoorBot
{
namespace Navigation
{

class DirAntFollower
{
 public:
  class Output
  {
   public:
    /**
     * The modes are
     *  TIMEOUT: The directional antenna timed out.
     *  WAIT_FOR_READY: The dir ant follower has issued a move or stop command and is waiting for the robot to report
     *   successful completion of this command.
     *  MOVE_FORWARD: Move forward a certain distance (the dir ant follower will give you incremental distance or you
     *   can just move any distance you think is appropriate.
     *  TURN: Turn a distance in DEGREES.
     *  STOP: Stay stopped.  We're waiting for the dir ant to sweep.
     */
    enum Mode
    {
      TIMEOUT               = -1,
      WAIT_FOR_READY        = 0,
      MOVE_FORWARD          = 1,
      TURN                  = 2,
      STOP                  = 3
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
  DirAntFollower();

  void activate();
  Output update(const Input& input);

 private:
  void setupFSM();

  void on_enter_wait_for_first_turn();
  int on_update_wait_for_first_turn();

  void on_enter_wait_for_angle();
  int on_update_wait_for_angle();

  void on_enter_turn();
  int on_update_turn();

  void on_enter_wait_before_move();
  int on_update_wait_before_move();

  void on_enter_move();
  int on_update_move();

  void on_enter_timeout();
  int on_update_timeout();

  FBFSM fsm_;
  int wait_for_first_turn_state_;
  int wait_for_angle_state_;
  int turn_state_;
  int wait_before_move_state_;
  int move_state_;
  int timeout_state_;

  struct WaitForAngleData
  {
    int sweep_number;
    ros::Time start_time;
    WaitForAngleData() : sweep_number(0) {}
  } wait_for_angle_data_;

  struct TurnData
  {
    double angle;
    TurnData() : angle(0.0) {}
  } turn_data_;

  struct WaitBeforeMoveData
  {
    ros::Time start_time;
  } wait_before_move_data_;

  struct Params
  {
    // A timeout on waiting for the directional antenna.
    double dir_ant_timeout;
    // How far to move forward each time.
    double incremental_distance;
    // If we're supposed to turn less than this, we skip the turn and go directly to the move state.
    double min_angle;
    // The time to wait before moving (usually for radar data).
    double wait_before_move_duration;
  } params_;

  ros::Publisher dir_ant_pub_;
  ros::ServiceClient dir_ant_client_;
  Input input_;
  Output output_;
};

}  // namespace Navigation
}  // namespace OutdoorBotx

#endif  // __OUTDOOR_BOT_DIR_ANT_FOLLOWER_H__
