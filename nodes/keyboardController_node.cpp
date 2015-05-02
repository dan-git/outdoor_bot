// publishes keyboard commands

#include <ros/ros.h>
#include <std_msgs/String.h>


class KeyboardCommands
{
private:
  ros::NodeHandle nh_;
  ros::Publisher key_pub_;

public:
  // ROS node initialization
  KeyboardCommands(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for keyboard commands
    key_pub_ = nh_.advertise<std_msgs::String>("keyboard", 1);
  }

  // Loop forever while sending drive commands based on keyboard input
  void driveKeyboard()
  {
    std_msgs::String msgKeyboard;
    std::cout << "Type a command and then press enter.  "
      "Use 'f' to move forward, 'b' to go backwards, 'l' to bend left, "
      "'r' to bend right, 'p' to toggle patrolling, 'h' to stop patrolling and go home,"
      " m to move 1 m forward, n to move 10 m forward, 's' to stop and/or re-enable, 'x' to disable,'.' to exit.\n";

    char cmd[5];
    while(nh_.ok()){
      std::cin.getline(cmd, 2); // note that because this blocks, the node does not exit
                                 // nicely with Ctrl-c  if you want a nice exit, get out of
                                 // the loop with the cmd . before hitting Ctrl-c
      if(cmd[0]!='f' && cmd[0]!='l' && cmd[0]!='r' && cmd[0] != 'c' && cmd[0] != 'p' && cmd[0] != 'h'
	&& cmd[0] != 'b' && cmd[0] != 's' && cmd[0] != 'x' && cmd[0] != 'm' && cmd[0] != 'n' && cmd[0] != 't' && cmd[0]!='.')
      {
	     std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      if(cmd[0]=='.'){
        ROS_INFO("Shutting down keyboard controller");
        return;
      }
      msgKeyboard.data = cmd;
      key_pub_.publish(msgKeyboard);
    }
}
};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "keyboard_commands");
  ros::NodeHandle nh;
  KeyboardCommands driver(nh);
  driver.driveKeyboard();
}


