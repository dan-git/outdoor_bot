#include <boost/bind.hpp>

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
  wait_for_angle_state_ = fsm_.add_state();
  turn_state_ = fsm_.add_state();
  move_state_ = fsm_.add_state();
  timeout_state_ = fsm_.add_state();

  fsm_.set_entry_function(wait_for_angle_state_, boost::bind(&DirAntFollower::on_enter_wait_for_angle, this));
  fsm_.set_update_function(
      wait_for_angle_state_, boost::bind(&DirAntFollower::on_update_wait_for_angle, this));

  fsm_.set_entry_function(turn_state_, boost::bind(&DirAntFollower::on_enter_turn, this));
  fsm_.set_update_function(turn_state_, boost::bind(&DirAntFollower::on_update_turn, this));

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

  ROS_INFO("Activating directional antenna follower.  The timeout on the antenna is %f and the incremental forward"
           " distance is %f.",
           params_.dir_ant_timeout,
           params_.incremental_distance);
  fsm_.set_state(wait_for_angle_state_);
}

DirAntFollower::Output DirAntFollower::update(const Input& input)
{
  input_ = input;
  output_ = Output();
  fsm_.update();
  return output_;
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
  return turn_state_;
}

void DirAntFollower::on_enter_turn()
{
  ROS_INFO("DirAntFollower: Turning %f degrees.", turn_data_.angle);
  output_.set_mode(Output::TURN);
  output_.set_distance(turn_data_.angle);
}

int DirAntFollower::on_update_turn()
{
  if (input_.mode() == Input::EXECUTING_COMMAND)
  {
    return turn_state_;
  }
  return move_state_;
}

void DirAntFollower::on_enter_move()
{
  ROS_INFO("DirAntFollower: Moving forward.");
  output_.set_mode(Output::MOVE_FORWARD);
  output_.set_distance(params_.incremental_distance);
}

int DirAntFollower::on_update_move()
{
  if (input_.mode() == Input::EXECUTING_COMMAND)
  {
    return move_state_;
  }
  return wait_for_angle_state_;
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
