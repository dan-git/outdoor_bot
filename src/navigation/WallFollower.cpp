#include <math.h>
#include <vector>

#include <boost/bind.hpp>

#include "navigation/NavUtils.h"
#include "navigation/WallFollower.h"

namespace OutdoorBot
{
namespace Navigation
{

WallFollower::WallFollower()
    : goal_(Goal::LEFT),
      input_(Input::EXECUTING_COMMAND)
{
  setupFSM();
}

void WallFollower::setupFSM()
{
  command_move_forward_state_    = fsm_.add_state();
  move_forward_state_            = fsm_.add_state();
  wait_for_obstacle_clear_state_ = fsm_.add_state();
  command_turn_away_state_       = fsm_.add_state();
  command_turn_into_state_       = fsm_.add_state();
  wait_for_turn_state_           = fsm_.add_state();
  successful_completion_state_   = fsm_.add_state();
  prepare_for_timeout_state_     = fsm_.add_state();
  timeout_state_                 = fsm_.add_state();

  fsm_.set_entry_function(command_move_forward_state_, boost::bind(&WallFollower::on_enter_command_move_forward, this));
  fsm_.set_update_function(
      command_move_forward_state_, boost::bind(&WallFollower::on_update_command_move_forward, this));

  fsm_.set_entry_function(move_forward_state_, boost::bind(&WallFollower::on_enter_move_forward, this));
  fsm_.set_update_function(move_forward_state_, boost::bind(&WallFollower::on_update_move_forward, this));

  fsm_.set_entry_function(
      wait_for_obstacle_clear_state_, boost::bind(&WallFollower::on_enter_wait_for_obstacle_clear, this));
  fsm_.set_update_function(
      wait_for_obstacle_clear_state_, boost::bind(&WallFollower::on_update_wait_for_obstacle_clear, this));

  fsm_.set_entry_function(command_turn_away_state_, boost::bind(&WallFollower::on_enter_command_turn_away, this));
  fsm_.set_update_function(command_turn_away_state_, boost::bind(&WallFollower::on_update_command_turn_away, this));

  fsm_.set_entry_function(command_turn_into_state_, boost::bind(&WallFollower::on_enter_command_turn_into, this));
  fsm_.set_update_function(command_turn_into_state_, boost::bind(&WallFollower::on_update_command_turn_into, this));

  fsm_.set_entry_function(wait_for_turn_state_, boost::bind(&WallFollower::on_enter_wait_for_turn, this));
  fsm_.set_update_function(wait_for_turn_state_, boost::bind(&WallFollower::on_update_wait_for_turn, this));

  fsm_.set_entry_function(
      successful_completion_state_, boost::bind(&WallFollower::on_enter_successful_completion, this));
  fsm_.set_update_function(
      successful_completion_state_, boost::bind(&WallFollower::on_update_successful_completion, this));

  fsm_.set_entry_function(prepare_for_timeout_state_, boost::bind(&WallFollower::on_enter_prepare_for_timeout, this));
  fsm_.set_update_function(prepare_for_timeout_state_, boost::bind(&WallFollower::on_update_prepare_for_timeout, this));

  fsm_.set_entry_function(timeout_state_, boost::bind(&WallFollower::on_enter_timeout, this));
  fsm_.set_update_function(timeout_state_, boost::bind(&WallFollower::on_update_timeout, this));
}

void WallFollower::activate(const Goal& goal)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("obstacle_avoider/stop_if_obstacle_within_distance", params_.stop_if_obstacle_within_distance, 1.5);
  private_nh.param("obstacle_avoider/robot_radius", params_.robot_radius, 0.63);
  private_nh.param("obstacle_avoider/wait_for_obstacle_clear_duration", params_.wait_for_obstacle_clear_duration, 30.0);
  private_nh.param("obstacle_avoider/side_angle", params_.side_angle, 1.0);
  private_nh.param("obstacle_avoider/incremental_distance", params_.incremental_distance, 2.0);
  private_nh.param("obstacle_avoider/move_timeout", params_.move_timeout, -1.0);
  private_nh.param("obstacle_avoider/turn_timeout", params_.turn_timeout, -1.0);
  private_nh.param("obstacle_avoider/always_turn_back", params_.always_turn_back, true);

  ROS_INFO("Activating wall follower.  Rectangle x dimension: %f, robot radius: %f, "
           "wait for obstacle clear duration: %f, side angle: %f, "
           "incremental forward distance: %f, move timeout: %f, turn timeout: %f, always turn back: %d",
           params_.stop_if_obstacle_within_distance,
           params_.robot_radius,
           params_.wait_for_obstacle_clear_duration,
           params_.side_angle,
           params_.incremental_distance,
           params_.move_timeout,
           params_.turn_timeout,
           params_.always_turn_back);

  state_ = State();
  goal_ = goal;
  fsm_.set_state(wait_for_obstacle_clear_state_);
}

WallFollower::Output WallFollower::update(const Input& input)
{
  input_ = input;
  output_ = Output();
  fsm_.update();
  return output_;
}

void WallFollower::on_enter_command_move_forward()
{
  ROS_INFO("WallFollower: Commanding forward move.");
}

int WallFollower::on_update_command_move_forward()
{
  output_.set_mode(Output::MOVE_FORWARD);
  output_.set_distance(params_.incremental_distance);
  return move_forward_state_;
}

void WallFollower::on_enter_move_forward()
{
  ROS_INFO("WallFollower: Waiting for forward move to complete.");
  // Reset the counts in the move forward data.
  move_forward_data_ = MoveForwardData();
  move_forward_data_.start_time = ros::WallTime::now();
  ObstacleDetector::DetectionParamsList detection_params(2);
  // Forwards detection.
  detection_params[0] = ObstacleDetector::DetectionParams(
      params_.stop_if_obstacle_within_distance, 2.0 * params_.robot_radius, 0.0, false);
  // Wall detection.  We want to count misses.
  double angle = params_.side_angle;
  if (goal_.side() == Goal::LEFT)
  {
    angle = params_.side_angle;
  }
  detection_params[1] = ObstacleDetector::DetectionParams(
      params_.stop_if_obstacle_within_distance, 2.0 * params_.robot_radius, angle, true);
  obstacle_detector_.activate(detection_params);
}

int WallFollower::on_update_move_forward()
{
  // Update the obstacle detector.
  std::vector<bool> detections;
  obstacle_detector_.update(&detections);

  if (detections[0])
  {
    // STOOOOOOOOP!  STOOOOOP NOW.
    return wait_for_obstacle_clear_state_;
  }

  // Check for timeout.
  if (params_.move_timeout > 0 &&
      ros::WallTime::now() - move_forward_data_.start_time > ros::WallDuration(params_.move_timeout))
  {
    return prepare_for_timeout_state_;
  }

  if (input_.mode() != Input::READY_FOR_NEW_COMMAND)
  {
    // We aren't ready to do anything new.
    return move_forward_state_;
  }

  if (params_.always_turn_back || detections[1])
  {
    // Wall on the side is gone!  Turn to follow wall.
    return command_turn_into_state_;
  }

  // There is still something next to us.  Move forward an incremental distance.
  return command_move_forward_state_;
}

void WallFollower::on_enter_wait_for_obstacle_clear()
{
  ROS_INFO("WallFollower: Waiting %f seconds to see if obstacle clears.", params_.wait_for_obstacle_clear_duration);
  wait_for_obstacle_clear_data_ = WaitForObstacleClearData();
  wait_for_obstacle_clear_data_.start_time = ros::WallTime::now();
  // Count obstacle misses.
  obstacle_detector_.activate(ObstacleDetector::DetectionParamsList(
      1, ObstacleDetector::DetectionParams(
          params_.stop_if_obstacle_within_distance, 2.0 * params_.robot_radius, 0.0, true)));
  output_.set_mode(Output::OBSTACLE_AHEAD);
}

int WallFollower::on_update_wait_for_obstacle_clear()
{
  bool no_obstacle = obstacle_detector_.update();

  // We're still trying to stop.
  if (input_.mode() != Input::READY_FOR_NEW_COMMAND)
  {
    return wait_for_obstacle_clear_state_;
  }

  if (no_obstacle)
  {
    // The obstacle left!  Yay.
    if (fabs(state_.current_angle) < 0.1)
    {
      // Since we start in this state it's possible we just waited for an obstacle to go by and now we are done.
      return successful_completion_state_;
    }

    // Move forward again.
    return command_move_forward_state_;
  }

  if (ros::WallTime::now() - wait_for_obstacle_clear_data_.start_time >
      ros::WallDuration(params_.wait_for_obstacle_clear_duration))
  {
    // The obstacle is still there :(  Wall follow around it.
    return command_turn_away_state_;
  }

  // Keep waiting.
  if (output_.mode() == Output::WAIT_FOR_READY)
  {
    output_.set_mode(Output::STOP);
  }
  return wait_for_obstacle_clear_state_;
}

void WallFollower::on_enter_command_turn_away()
{
  ROS_INFO("WallFollower: Commanding turn away from wall.");
}

int WallFollower::on_update_command_turn_away()
{
  output_.set_mode(Output::TURN);
  double angle = M_PI / 2.0;
  if (goal_.side() == Goal::RIGHT)
  {
    angle = -M_PI / 2.0;
  }
  output_.set_distance(angle);
  state_.current_angle = wrapAngle(state_.current_angle + angle);
  return wait_for_turn_state_;
}

void WallFollower::on_enter_command_turn_into()
{
  ROS_INFO("WallFollower: No wall detected.  Commanding a turn back.");
}

int WallFollower::on_update_command_turn_into()
{
  output_.set_mode(Output::TURN);
  // We're going to turn the opposite way from how the goal commanded.
  double angle = M_PI / 2.0;
  if (goal_.side() == Goal::LEFT)
  {
    angle = -M_PI / 2.0;
  }
  output_.set_distance(angle);
  state_.current_angle = wrapAngle(state_.current_angle + angle);
  return wait_for_turn_state_;
}

void WallFollower::on_enter_wait_for_turn()
{
  ROS_INFO("WallFollower: Waiting for turn to be completed.");
  wait_for_turn_data_.start_time = ros::WallTime::now();
}

int WallFollower::on_update_wait_for_turn()
{
  if (params_.turn_timeout > 0 && ros::WallTime::now() - wait_for_turn_data_.start_time >
      ros::WallDuration(params_.turn_timeout))
  {
    return prepare_for_timeout_state_;
  }
  if (input_.mode() != Input::READY_FOR_NEW_COMMAND)
  {
    return wait_for_turn_state_;
  }

  if (fabs(state_.current_angle) > 0.1)
  {
    return command_move_forward_state_;
  }

  // We've regained our original heading.  We're done!
  return successful_completion_state_;
}

void WallFollower::on_enter_successful_completion()
{
  ROS_INFO("WallFollower: Original heading regained.  Obstacle successfully avoided!");
}

int WallFollower::on_update_successful_completion()
{
  output_.set_mode(Output::SUCCESSFUL_COMPLETION);
  return successful_completion_state_;
}

void WallFollower::on_enter_prepare_for_timeout()
{
  ROS_ERROR("WallFollower: A move or turn timed out.  Issuing a STOP command.");
  output_.set_mode(Output::OBSTACLE_AHEAD);
}

int WallFollower::on_update_prepare_for_timeout()
{
  if (input_.mode() == Input::READY_FOR_NEW_COMMAND)
  {
    return timeout_state_;
  }
  return prepare_for_timeout_state_;
}

void WallFollower::on_enter_timeout()
{
  ROS_ERROR("WallFollower: A move or turn timed out.");
}

int WallFollower::on_update_timeout()
{
  output_.set_mode(Output::TIMEOUT);
  return timeout_state_;
}

}  // namespace Navigation
}  // namespace OutdoorBot
