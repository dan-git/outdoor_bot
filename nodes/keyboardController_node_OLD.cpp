// controls a robot from the keyboard
/*
w	x	y	z	Description
1	0	0	0	Identity quaternion, no rotation
0	1	0	0	180° turn around X axis
0	0	1	0	180° turn around Y axis
0	0	0	1	180° turn around Z axis
sqrt(0.5)	sqrt(0.5)	0	0	90° rotation around X axis
sqrt(0.5)	0	sqrt(0.5)	0	90° rotation around Y axis
sqrt(0.5)	0	0	sqrt(0.5)	90° rotation around Z axis
sqrt(0.5)	-sqrt(0.5)	0	0	-90° rotation around X axis
sqrt(0.5)	0	-sqrt(0.5)	0	-90° rotation around Y axis
sqrt(0.5)	0	0	-sqrt(0.5)	-90° rotation around Z axis

*/

#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  // create the action client to send goals to move_base
  // true causes the client to spin its own thread


class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_, speech_pub_; //action_goal_pub_;

  //MoveBaseClient *ac;
  move_base_msgs::MoveBaseGoal goal;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // and the publisher for speech output
    speech_pub_ = nh_.advertise<std_msgs::String>("play_wavfile", 1);


    //action_goal_pub_ = nh_.advertise<action_lib_msgs::GoalID>("move_base/cancel", 1);
    //ac = new MoveBaseClient("move_base", true);
  }

  //! Loop forever while sending drive commands based on keyboard input
  void driveKeyboard()
  {
    std_msgs::String msgSpeech;
    std::cout << "Type a command and then press enter.  "
      "Use 'f' to move forward, 'b' to go backwards, 'l' to bend left, "
      "'r' to bend right, 's' to stop and/or re-enable, 'x' to disable,'.' to exit.\n";

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    base_cmd.angular.x = 0.0;
    base_cmd.angular.y = 0.0;
    base_cmd.linear.y = 0.0;
 
    MoveBaseClient ac("move_base",true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Keyboard controller is waiting for the move_base action server to come up");
    }
    ROS_INFO("Keyboard controller sees that the move_base action server is up");
    char cmd[50];
    while(nh_.ok()){
      std::cin.getline(cmd, 50); // note that because this blocks, the node does not exit
                                 // nicely with Ctrl-c  if you want a nice exit, get out of
                                 // the loop with the cmd . before hitting Ctrl-c
      if(cmd[0]!='f' && cmd[0]!='l' && cmd[0]!='r' && cmd[0] != 'c'
	&& cmd[0] != 'b' && cmd[0] != 's' && cmd[0] != 'x' && cmd[0] != 'm' && cmd[0] != 't' && cmd[0]!='.')
      {
        base_cmd.angular.z = 0.0;
        base_cmd.linear.x = 0.0;
	std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      base_cmd.linear.x = 0.0;
      base_cmd.linear.z = 0.0;
      base_cmd.angular.z = 0.0;   
      //move forward
      if(cmd[0]=='f'){
        base_cmd.linear.x = 0.15;
        base_cmd.angular.z = 0.0;
        //publish the assembled command
        cmd_vel_pub_.publish(base_cmd);
        msgSpeech.data = pickAcknowledgeWav();
      } 
      //turn left (yaw) and drive forward at the same time
      else if(cmd[0]=='l'){
        base_cmd.angular.z = 1.0;
        base_cmd.linear.x = 0.0; //0.25;
        //publish the assembled command
        cmd_vel_pub_.publish(base_cmd);
        msgSpeech.data = pickAcknowledgeWav();
      } 
      //turn right (yaw) and drive forward at the same time
      else if(cmd[0]=='r'){
        base_cmd.angular.z = -1.0;
        base_cmd.linear.x = 0.0; //.25;
        //publish the assembled command
        cmd_vel_pub_.publish(base_cmd);
        msgSpeech.data = pickAcknowledgeWav();
      }
      //move backwards
      else if(cmd[0]=='b'){
        base_cmd.linear.x = -0.15;
        base_cmd.angular.z = 0.0;
        //publish the assembled command
        cmd_vel_pub_.publish(base_cmd);
        msgSpeech.data = pickAcknowledgeWav();
      } 
      //stop
      else if(cmd[0]=='s'){
        base_cmd.angular.z = 0.0;
        base_cmd.linear.x = 0.0;
        base_cmd.linear.z = 1.0; // we cannot move vertically, this is our re-enable command
        //publish the assembled command
        cmd_vel_pub_.publish(base_cmd);
        msgSpeech.data = pickCompletedWav();
      }
      //disable
     else if(cmd[0]=='x'){

        base_cmd.angular.z = 0.0;
        base_cmd.linear.x = 0.0;
        base_cmd.linear.z = -1.0; // we cannot move vertically, this is our disable command
        //publish the assembled command
        cmd_vel_pub_.publish(base_cmd);
        msgSpeech.data = pickAnnoyedWav();
      }
      //cancel goal
      else if(cmd[0]=='c'){

        ac.cancelAllGoals();
        ROS_INFO("canceling all goals");
        msgSpeech.data = pickAnnoyedWav();
        //we'll send a goal to the robot to stop where it is
        /*	goal.target_pose.header.frame_id = "base_link";
        	goal.target_pose.header.stamp = ros::Time::now();

        	goal.target_pose.pose.position.x = 0.0;
        	goal.target_pose.pose.orientation.w = 1.0;
              goal.target_pose.pose.orientation.z = 0.0;
              goal.target_pose.pose.orientation.y = 0.0;
              goal.target_pose.pose.orientation.x = 0.0;

        	ROS_INFO("Sending stop goal");
        	ac.sendGoal(goal);
         */
      }	
      // move 1 meter forward
      else if(cmd[0]=='m'){
        	//we'll send a goal to the robot to move 1 meter forward
        	goal.target_pose.header.frame_id = "base_link";
        	goal.target_pose.header.stamp = ros::Time::now();

        	goal.target_pose.pose.position.x = 1.0;
              goal.target_pose.pose.orientation.w = 1.0;
              goal.target_pose.pose.orientation.z = 0.0;
              goal.target_pose.pose.orientation.y = 0.0;
              goal.target_pose.pose.orientation.x = 0.0;

              ROS_INFO("Sending goal");
        	ac.sendGoal(goal);
         /*
	 bool finishedInTime = ac.waitForResult(ros::Duration(15));
         if (finishedInTime)
         {
             if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
             {
               ROS_INFO("Hooray, the base moved 1 meter forward");
               msgSpeech.data = pickCompletedWav();
             }
             else
             {
               ROS_INFO("The base failed to move forward 1 meter for some reason"); 
               msgSpeech.data = pickAnnoyedWav();
             }
         }
         else
         {
            ROS_INFO("Timed out moving to goal");
            msgSpeech.data = pickAnnoyedWav();
         }
	 */

      }
      // turn 90 degrees to the right
      else if(cmd[0]=='t'){
	      goal.target_pose.header.frame_id = "base_link";
        	goal.target_pose.header.stamp = ros::Time::now();

        	goal.target_pose.pose.position.x = 0.0;
        	goal.target_pose.pose.orientation.w = sqrt(0.5);
              goal.target_pose.pose.orientation.z = -sqrt(0.5);

        	ROS_INFO("Sending goal");
        	ac.sendGoal(goal);
         bool finishedInTime = ac.waitForResult(ros::Duration(15));
         if (finishedInTime)
         {
             if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
             {
               ROS_INFO("Hooray, the base turned 90 degrees right");
               msgSpeech.data = pickCompletedWav();
             }
             else
             {
               ROS_INFO("The base failed to turn 90 degrees right for some reason"); 
               msgSpeech.data = pickAnnoyedWav();
             }
         }
         else
         {
            ROS_INFO("Timed out moving to goal");
            msgSpeech.data = pickAnnoyedWav();
         }
      }
      //quit
      else if(cmd[0]=='.'){
        base_cmd.angular.z = 0.0;
        base_cmd.linear.x = 0.0;
        //publish the assembled command
        cmd_vel_pub_.publish(base_cmd);
        ROS_INFO("Shutting down keyboard controller");
        break;
      }
      speech_pub_.publish(msgSpeech);
    }


   }

  std::string pickAcknowledgeWav()
  {
      int randInt = rand() % 10; // generate random integer number between 0 and 9
      switch(randInt)
      {
         case 0: return "/warcraft_alliance/ok_peasant.wav";
         case 1: return "/warcraft_alliance/morework_peasant.wav";
         case 2: return "/warcraft_alliance/makeupyourmind_basic.wav";
         case 3: return "/warcraft_alliance/yesmylord_peasant.wav";
         case 4: return "/warcraft_alliance/righto_peasant.wav";
         case 5: return "/warcraft_alliance/alright_peasant.wav";
         case 6: return "/ok_troll.wav";
         case 7: return "/thisway_no_thatway.wav";
         case 8: return "/alright_troll.wav";
         case 9: return "/ok.wav";
      }
      return "";
  }

  std::string pickCompletedWav()
  {
      int randInt = rand() % 7; // generate random integer number between 0 and 6
      switch(randInt)
      {
         case 0: return "/warcraft_alliance/jobsdone_peasant.wav";
         case 1: return "/readytowork_peon.wav";
         case 2: return "/ready_imnotready.wav";
         case 3: return "/alright_nowImhungry.wav";
         case 4: return "/burp_scuseme_basic.wav";
         case 5: return "/warcraft_alliance/righto_peasant.wav";
         case 6: return "/warcraft_alliance/alright_peasant.wav";

      }
      return "";
  }

  std::string pickAnnoyedWav()
  {
      int randInt = rand() % 8; // generate random integer number between 0 and 7
      switch(randInt)
      {
         case 0: return "/warcraft_alliance/idontwanttodothis_peasant.wav";
         case 1: return "/warcraft_alliance/leavemealone_peasant.wav";
         case 2: return "/warcraft_alliance/ohwhat_peasant.wav";
         case 3: return "/warcraft_alliance/whatisit_peasant.wav";
         case 4: return "/arrh_dk.wav";
         case 5: return "/dontpushit_dk.wav";
         case 6: return "/grumbling_troll.wav";
         case 7: return "/whenmyworkisfinishedimcomingbackforyou_dk.wav";
      }
      return "";
  }
};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  RobotDriver driver(nh);
  driver.driveKeyboard();
}


