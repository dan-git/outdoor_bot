#include <math.h>
#include <boost/bind.hpp>

#include "navigation/NavUtils.h"
#include "navigation/WallFollower.h"

namespace OutdoorBot
{
namespace Navigation
{

WallFollower::WallFollower()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("obstacle_distance", params_.obstacle_distance, 1.5);
  private_nh.param("wall_distance", params_.wall_distance, 1.0);
  private_nh.param("robot_radius", params_.robot_radius, 0.65);
  private_nh.param("wait_for_obstacle_clear_duration", params_.wait_for_obstacle_clear_duration, 3.0);

  setupFSM();
}

void WallFollower::setupFSM()
{
  move_forward_state_            = fsm_.add_state();
  wait_for_obstacle_clear_state_ = fsm_.add_state();
  turn_away_state_               = fsm_.add_state();
  turn_into_state_               = fsm_.add_state();

  fsm_.set_entry_function(move_forward_state_, boost::bind(&WallFollower::on_enter_move_forward, this));
  fsm_.set_update_function(move_forward_state_, boost::bind(&WallFollower::on_update_move_forward, this));

  fsm_.set_entry_function(
      wait_for_obstacle_clear_state_, boost::bind(&WallFollower::on_enter_wait_for_obstacle_clear, this));
  fsm_.set_update_function(
      wait_for_obstacle_clear_state_, boost::bind(&WallFollower::on_update_wait_for_obstacle_clear, this));

  fsm_.set_entry_function(turn_away_state_, boost::bind(&WallFollower::on_enter_turn_away, this));
  fsm_.set_update_function(turn_away_state_, boost::bind(&WallFollower::on_update_turn_away, this));

  fsm_.set_entry_function(turn_into_state_, boost::bind(&WallFollower::on_enter_turn_into, this));
  fsm_.set_update_function(turn_into_state_, boost::bind(&WallFollower::on_update_turn_into, this));
}

void WallFollower::activate(const Side& side, double linear_velocity, double angular_velocity)
{
  side_ = side;
  linear_velocity_ = linear_velocity;
  angular_velocity_ = angular_velocity;
  fsm_.set_state(move_forward_state_);
}

Velocity WallFollower::update(const Pose2D& robot_pose)
{
  robot_pose_ = robot_pose;
  fsm_.update();
  return velocity_;
}

void WallFollower::on_enter_move_forward()
{
  ROS_INFO("WallFollower: Entering move forward state.");
  // Reset the counts in the move forward data.
  move_forward_data_ = MoveForwardData();
}

int WallFollower::on_update_move_forward()
{
  velocity_.set_velocity(linear_velocity_);
  velocity_.set_type(Velocity::LINEAR);

  // Is there still a wall next to us?
  double angle = -M_PI / 2.0;
  if (side_ == LEFT)
  {
    angle = M_PI / 2.0;
  }
  if (obstacle_detector_.obstacleInCone(params_.obstacle_distance + params_.wall_distance, params_.robot_radius, angle))
  {
    move_forward_data_.no_wall_detections = 0;
  }
  else
  {
    move_forward_data_.no_wall_detections++;
  }
  if (move_forward_data_.no_wall_detections >= params_.max_obstacle_detections)
  {
    // Wall on the side is gone!  Turn to follow wall.
    return turn_into_state_;
  }

  // Are we about to hit another wall?
  if (obstacle_detector_.obstacleInCone(params_.obstacle_distance, params_.robot_radius, 0.0))
  {
    move_forward_data_.forward_obstacle_detections++;
  }
  else
  {
    move_forward_data_.forward_obstacle_detections = 0;
  }
  // Oops we're going to hit a wall.
  if (move_forward_data_.forward_obstacle_detections >= params_.max_obstacle_detections)
  {
    return wait_for_obstacle_clear_state_;
  }
  // Just keep swimming.
  return move_forward_state_;
}

void WallFollower::on_enter_wait_for_obstacle_clear()
{
  ROS_INFO("WallFollower: Waiting %f seconds to see if obstacle clears.", params_.wait_for_obstacle_clear_duration);
  wait_for_obstacle_clear_data_ = WaitForObstacleClearData();
  wait_for_obstacle_clear_data_.start_time = ros::WallTime::now();
}

int WallFollower::on_update_wait_for_obstacle_clear()
{
  velocity_.set_velocity(0.0);
  velocity_.set_type(Velocity::STOP);

  // Is the obstacle gone?
  if (!obstacle_detector_.obstacleInCone(params_.obstacle_distance + params_.wall_distance, params_.robot_radius, 0.0))
  {
    wait_for_obstacle_clear_data_.no_obstacle_detections++;
  }
  else
  {
    wait_for_obstacle_clear_data_.no_obstacle_detections = 0;
  }

  if (wait_for_obstacle_clear_data_.no_obstacle_detections >= params_.max_obstacle_detections)
  {
    // The obstacle left!  Yay.
    return move_forward_state_;
  }

  if (ros::WallTime::now() - wait_for_obstacle_clear_data_.start_time >
      ros::WallDuration(params_.wait_for_obstacle_clear_duration))
  {
    // The obstacle is still there :(  Wall follow around it.
    return turn_away_state_;
  }

  // Keep waiting.
  return wait_for_obstacle_clear_state_;
}

void WallFollower::on_enter_turn_away()
{
  ROS_INFO("WallFollower: Turning away from obstacle in front of me.");
  turn_away_data_.starting_angle = robot_pose_.theta();
}

int WallFollower::on_update_turn_away()
{
  velocity_.set_type(Velocity::ANGULAR);
  velocity_.set_velocity(angular_velocity_);
  if (side_ == LEFT)
  {
    velocity_.set_velocity(-angular_velocity_);
  }
  if (angularDistance(robot_pose_.theta(), turn_away_data_.starting_angle >= M_PI / 2.0))
  {
    return move_forward_state_;
  }
  return turn_away_state_;
}

void WallFollower::on_enter_turn_into()
{
  ROS_INFO("WallFollower: Lost wall.  Turning to follow wall on my side.");
  turn_into_data_.starting_angle = robot_pose_.theta();
}

int WallFollower::on_update_turn_into()
{
  velocity_.set_type(Velocity::ANGULAR);
  velocity_.set_velocity(angular_velocity_);
  if (side_ == RIGHT)
  {
    velocity_.set_velocity(-angular_velocity_);
  }
  if (angularDistance(robot_pose_.theta(), turn_into_data_.starting_angle >= M_PI / 2.0))
  {
    return move_forward_state_;
  }
  return turn_into_state_;
}

}  // namespace Navigation
}  // namespace OutdoorBot
