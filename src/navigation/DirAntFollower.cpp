#include <boost/bind.hpp>
#include <boost/math/special_functions/sign.hpp>

#include <ros/ros.h>

#include "outdoor_bot/dirAnt_msg.h"
#include "outdoor_bot/dirAnt_service.h"
#include "navigation/DirAntFollower.h"

namespace OutdoorBot
{
namespace Navigation
{

DirAntFollower::DirAntFollower()
    : input_(Input::EXECUTING_COMMAND)
{
  ros::NodeHandle nh;
  dir_ant_pub_ = nh.advertise<outdoor_bot::dirAnt_msg>("dirAnt_cmd", 5);
  dir_ant_client_ = nh.serviceClient<outdoor_bot::dirAnt_service>("dirAnt_service");
  setupFSM();
}

void DirAntFollower::setupFSM()
{
  wait_for_first_turn_state_ = fsm_.add_state();
  run_multiple_sweeps_state_ = fsm_.add_state();
  wait_for_angle_state_ = fsm_.add_state();
  retry_sweep_state_ = fsm_.add_state();
  turn_state_ = fsm_.add_state();
  wait_before_move_state_ = fsm_.add_state();
  move_state_ = fsm_.add_state();
  timeout_state_ = fsm_.add_state();

  fsm_.set_entry_function(wait_for_first_turn_state_, boost::bind(&DirAntFollower::on_enter_wait_for_first_turn, this));
  fsm_.set_update_function(
      wait_for_first_turn_state_, boost::bind(&DirAntFollower::on_update_wait_for_first_turn, this));

  fsm_.set_entry_function(run_multiple_sweeps_state_, boost::bind(&DirAntFollower::on_enter_run_multiple_sweeps, this));
  fsm_.set_update_function(run_multiple_sweeps_state_, boost::bind(&DirAntFollower::on_update_run_multiple_sweeps, this));

  fsm_.set_entry_function(wait_for_angle_state_, boost::bind(&DirAntFollower::on_enter_wait_for_angle, this));
  fsm_.set_update_function(
      wait_for_angle_state_, boost::bind(&DirAntFollower::on_update_wait_for_angle, this));

  fsm_.set_entry_function(retry_sweep_state_, boost::bind(&DirAntFollower::on_enter_retry_sweep, this));
  fsm_.set_update_function(retry_sweep_state_, boost::bind(&DirAntFollower::on_update_retry_sweep, this));

  fsm_.set_entry_function(turn_state_, boost::bind(&DirAntFollower::on_enter_turn, this));
  fsm_.set_update_function(turn_state_, boost::bind(&DirAntFollower::on_update_turn, this));

  fsm_.set_entry_function(wait_before_move_state_, boost::bind(&DirAntFollower::on_enter_wait_before_move, this));
  fsm_.set_update_function(wait_before_move_state_, boost::bind(&DirAntFollower::on_update_wait_before_move, this));

  fsm_.set_entry_function(move_state_, boost::bind(&DirAntFollower::on_enter_move, this));
  fsm_.set_update_function(move_state_, boost::bind(&DirAntFollower::on_update_move, this));

  fsm_.set_entry_function(timeout_state_, boost::bind(&DirAntFollower::on_enter_timeout, this));
  fsm_.set_update_function(timeout_state_, boost::bind(&DirAntFollower::on_update_timeout, this));
}

void DirAntFollower::activate()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("dir_ant_follower/dir_ant_timeout", params_.dir_ant_timeout, -1.0);
  private_nh.param("dir_ant_follower/incremental_distance", params_.incremental_distance, 10.0);
  private_nh.param("dir_ant_follower/min_angle", params_.min_angle, 0.5);
  private_nh.param("dir_ant_follower/wait_before_move_duration", params_.wait_before_move_duration, 2.0);
  private_nh.param("dir_ant_follower/n_sweeps", params_.n_sweeps, 3);
  private_nh.param("dir_ant_follower/max_dir_ant_angle", params_.max_dir_ant_angle, 30.0);


  if (params_.min_angle < 0.5)
  {
    ROS_ERROR("DirAntFollower: Cannot turn less than 0.5 degrees.  Setting min angle to 0.5.");
    params_.min_angle = 0.5;
  }

  ROS_INFO("Activating directional antenna follower.  The timeout on the antenna is %f, the incremental forward"
           " distance is %f, the minimum turn is %f degrees, and we will wait %f seconds before starting a move.",
           params_.dir_ant_timeout,
           params_.incremental_distance,
           params_.min_angle,
           params_.wait_before_move_duration);
  fsm_.set_state(wait_for_first_turn_state_);
}

DirAntFollower::Output DirAntFollower::update(const Input& input)
{
  input_ = input;
  output_ = Output();
  fsm_.update();
  return output_;
}

void DirAntFollower::on_enter_wait_for_first_turn()
{
  ROS_INFO("DirAntFollower: Waiting for first turn (from accelerometers presumably) to be completed.");
}

int DirAntFollower::on_update_wait_for_first_turn()
{
  if (input_.mode() == Input::READY_FOR_NEW_COMMAND)
  {
    return run_multiple_sweeps_state_;
  }
  return wait_for_first_turn_state_;
}

void DirAntFollower::on_enter_run_multiple_sweeps()
{
	ROS_INFO("Starting angle sweep.  Will do %d sweeps if we see angles larger than %f degrees.",
		params_.n_sweeps, params_.max_dir_ant_angle);
	wait_for_angle_data_.n_sweeps = 0;
}

int DirAntFollower::on_update_run_multiple_sweeps()
{
	return wait_for_angle_state_;
}

void DirAntFollower::on_enter_wait_for_angle()
{
  ROS_INFO("DirAntFollower: Sweeping directional antenna...");
  // Check what the sweep number is now so we know when we get a new one.
  outdoor_bot::dirAnt_service::Request req;
  outdoor_bot::dirAnt_service::Response resp;
  dir_ant_client_.call(req, resp);
  wait_for_angle_data_.sweep_number = resp.dirAntSweepNumber;

  // Publish a message to tell the antenna to start its sweep.
  outdoor_bot::dirAnt_msg dir_ant_msg;
  dir_ant_msg.antennaCommand = 1;
  dir_ant_msg.antennaPan = 0;
  dir_ant_pub_.publish(dir_ant_msg);

  wait_for_angle_data_.start_time = ros::Time::now();
}

int DirAntFollower::on_update_wait_for_angle()
{
  output_.set_mode(Output::STOP);
  outdoor_bot::dirAnt_service::Request req;
  outdoor_bot::dirAnt_service::Response resp;
  dir_ant_client_.call(req, resp);
  if (params_.dir_ant_timeout > 0 &&
      ros::Time::now() - wait_for_angle_data_.start_time > ros::Duration(params_.dir_ant_timeout))
  {
    return timeout_state_;
  }
  if (resp.dirAntSweepNumber == wait_for_angle_data_.sweep_number)
  {
    // Haven't finished the sweep yet.
    return wait_for_angle_state_;
  }
  // Finished sweep.  Turn.
  turn_data_.angle = resp.dirAntMaxAngle;
  wait_for_angle_data_.n_sweeps++;
  if (fabs(turn_data_.angle) > params_.max_dir_ant_angle)
  {
  	 ROS_WARN("DirAntFollower: Saw an angle of %f > %f from directional antenna.", turn_data_.angle, params_.max_dir_ant_angle);
  	 if (wait_for_angle_data_.n_sweeps < params_.n_sweeps)
  	 {
  	 	// Try again
  	 	return retry_sweep_state_;
  	 }
	 // Don't turn more than the max allowed angle.
  	 turn_data_.angle = boost::math::sign(turn_data_.angle) * params_.max_dir_ant_angle;
  } 
  if (fabs(turn_data_.angle) < params_.min_angle)
  {
    return move_state_;
  }
  return turn_state_;
}

void DirAntFollower::on_enter_retry_sweep()
{
	ROS_INFO("DirAntFollower: Retrying sweep.");
}

int DirAntFollower::on_update_retry_sweep()
{
	return wait_for_angle_state_;
}
void DirAntFollower::on_enter_turn()
{
  ROS_INFO("DirAntFollower: Turning %f degrees.", turn_data_.angle);
  output_.set_mode(Output::TURN);
  output_.set_distance(turn_data_.angle);
}

int DirAntFollower::on_update_turn()
{
  if (output_.mode() == Output::TURN)
  {
    // First time through.
    return turn_state_;
  }
  if (input_.mode() == Input::EXECUTING_COMMAND)
  {
    return turn_state_;
  }
  if (params_.wait_before_move_duration > 0)
  {
    return wait_before_move_state_;
  }
  return move_state_;
}

void DirAntFollower::on_enter_wait_before_move()
{
  ROS_INFO("DirAntFollower: Waiting before we move (probably for laser).");
  wait_before_move_data_.start_time = ros::Time::now();
}

int DirAntFollower::on_update_wait_before_move()
{
  output_.set_mode(Output::STOP);
  if (ros::Time::now() - wait_before_move_data_.start_time > ros::Duration(params_.wait_before_move_duration))
  {
    return move_state_;
  }
  return wait_before_move_state_;
}

void DirAntFollower::on_enter_move()
{
  ROS_INFO("DirAntFollower: Moving forward.");
  output_.set_mode(Output::MOVE_FORWARD);
  output_.set_distance(params_.incremental_distance);
}

int DirAntFollower::on_update_move()
{
  if (output_.mode() == Output::MOVE_FORWARD)
  {
    return move_state_;
  }
  if (input_.mode() == Input::EXECUTING_COMMAND)
  {
    return move_state_;
  }
  return run_multiple_sweeps_state_;
}

void DirAntFollower::on_enter_timeout()
{
  ROS_ERROR("DirAntFollower: Antenna timed out.");
}

int DirAntFollower::on_update_timeout()
{
  output_.set_mode(Output::TIMEOUT);
  return timeout_state_;
}

}  // namespace Navigation
}  // namespace OutdoorBot
