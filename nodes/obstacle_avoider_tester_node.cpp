#include <string>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "FBFSM/FBFSM.h"
#include "navigation/ObstacleDetector.h"
#include "navigation/WallFollower.h"

namespace OutdoorBot
{
namespace Navigation
{

class ObstacleAvoiderTester
{
 public:
  ObstacleAvoiderTester()
  {
    ros::NodeHandle nh;
    subscriber_ = nh.subscribe("/ready_for_new_command", 1, &ObstacleAvoiderTester::new_command_cb, this);
    setupFSM();
  }

  void update() {fsm_.update();}

 private:
  void new_command_cb(const std_msgs::Empty& msg)
  {
    ready_for_new_command_ = true;
  }

  void setupFSM()
  {
    move_state_ = fsm_.add_state();
    avoid_state_ = fsm_.add_state();

    fsm_.set_entry_function(move_state_, boost::bind(&ObstacleAvoiderTester::on_enter_move, this));
    fsm_.set_update_function(move_state_, boost::bind(&ObstacleAvoiderTester::on_update_move, this));

    fsm_.set_entry_function(avoid_state_, boost::bind(&ObstacleAvoiderTester::on_enter_avoid, this));
    fsm_.set_update_function(avoid_state_, boost::bind(&ObstacleAvoiderTester::on_update_avoid, this));

    fsm_.set_state(move_state_);
  }

  void on_enter_move()
  {
    detector_.activate(1.5, 0.63);
  }

  int on_update_move()
  {
    if (detector_.update())
    {
      ROS_INFO("SEE AN OBSTACLE");
      return avoid_state_;
    }
    return move_state_;
  }

  void on_enter_avoid()
  {
    avoider_.activate(WallFollower::Goal(WallFollower::Goal::LEFT));
    ready_for_new_command_ = true;
  }

  int on_update_avoid()
  {
    WallFollower::Input input(WallFollower::Input::EXECUTING_COMMAND);
    if (ready_for_new_command_)
    {
      input = WallFollower::Input(WallFollower::Input::READY_FOR_NEW_COMMAND);
    }
    WallFollower::Output output = avoider_.update(input);

    switch (output.mode())
    {
      case WallFollower::Output::TIMEOUT:
        ROS_ERROR("There was a timeout!!!");
        return move_state_;
      case WallFollower::Output::SUCCESSFUL_COMPLETION:
        ROS_INFO("Successfully avoided the obstacle!");
        return move_state_;
      case WallFollower::Output::WAIT_FOR_READY:
        return avoid_state_;
      case WallFollower::Output::OBSTACLE_AHEAD:
        ROS_INFO("OBSTACLE AHEAD!!!");
        break;
      case WallFollower::Output::MOVE_FORWARD:
        ROS_INFO("Please move %f meters.", output.distance());
        break;
      case WallFollower::Output::TURN:
        ROS_INFO("Please turn %f radians.  Press enter to ack command, publish to report done.", output.distance());
        break;
      case WallFollower::Output::STOP:
        return avoid_state_;
    }
    ROS_INFO("Press enter now to ack new command.  Publish to /ready_for_new_command when done.");
    std::string instr;
    getline(std::cin, instr);
    ready_for_new_command_ = false;
    return avoid_state_;
  }

  FBFSM fsm_;
  int move_state_;
  int avoid_state_;

  bool ready_for_new_command_;
  WallFollower avoider_;
  ObstacleDetector detector_;
  ros::Subscriber subscriber_;
};

}  // namespace Navigation
}  // namespace OutdoorBot

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoider_tester_node");
  ros::NodeHandle nh;
  OutdoorBot::Navigation::ObstacleAvoiderTester tester;
  ros::Rate r(100);
  while (ros::ok())
  {
    ros::spinOnce();
    tester.update();
    r.sleep();
  }
}
