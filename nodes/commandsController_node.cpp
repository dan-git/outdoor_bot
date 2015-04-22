// controls a robot from speech or keyboard commands
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
#include <tf/LinearMath/QuadWord.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "outdoor_bot/movement_msg.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher cmd_vel_pub_, speech_pub_;
// we subscribe to the speech recognizer and keyboard commands publishers
ros::Subscriber keyboardCommandsSub_, motionCommandsSub_, movementCommandsSub_;

// create the action client to send goals to move_base
// true causes the client to spin its own thread


MoveBaseClient *ac_;

std::string pickAcknowledgeWav()
{
   int randInt = rand() % 9; // generate random integer number between 0 and 8
   switch(randInt)
   {
      case 0: return "/warcraft_alliance/ok_peasant.wav";
      case 1: return "/warcraft_alliance/morework_peasant.wav";
      case 2: return "/warcraft_alliance/yesmylord_peasant.wav";
      case 3: return "/warcraft_alliance/righto_peasant.wav";
      case 4: return "/warcraft_alliance/alright_peasant.wav";
      case 5: return "/ok_troll.wav";
      case 6: return "/thisway_no_thatway.wav";
      case 7: return "/alright_troll.wav";
      case 8: return "/ok.wav";
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

void sendMotionCommand(std::string motionCmd)
{  
   std_msgs::String msgSpeech;
   move_base_msgs::MoveBaseGoal goal;
   // some goal parameters never change (e.g., we cannot translate in z or rotate in x  or y)
   goal.target_pose.pose.position.z = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   //we will be sending commands of type "twist"
   geometry_msgs::Twist base_cmd;
   // our bot does not use these motions, set them to 0
   base_cmd.angular.x = 0.0;
   base_cmd.angular.y = 0.0;
   base_cmd.linear.y = 0.0;
   // we do use these motions, but start them at 0 too
   base_cmd.linear.x = 0.0;
   base_cmd.linear.z = 0.0;   // this is our enable command.  start enabled.
   base_cmd.angular.z = 0.0;   

   bool finishedInTime;

   //move forward
   if(motionCmd[0]=='f'){
     base_cmd.linear.x = 0.15;
     base_cmd.angular.z = 0.0;
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
     msgSpeech.data = pickAcknowledgeWav();
   } 
   //turn left (yaw) and drive forward at the same time
   else if(motionCmd[0]=='l'){
     base_cmd.angular.z = 0.75;
     base_cmd.linear.x = 0.0; //0.25;
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
     msgSpeech.data = pickAcknowledgeWav();
   } 
   //turn right (yaw) and drive forward at the same time
   else if(motionCmd[0]=='r'){
     base_cmd.angular.z = -0.75;
     base_cmd.linear.x = 0.0; //.25;
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
     msgSpeech.data = pickAcknowledgeWav();
   }
   //move backwards
   else if(motionCmd[0]=='b'){
     base_cmd.linear.x = -0.15;
     base_cmd.angular.z = 0.0;
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
     msgSpeech.data = pickAcknowledgeWav();
   } 
   //stop
   else if(motionCmd[0]=='s'){
     base_cmd.angular.z = 0.0;
     base_cmd.linear.x = 0.0;
     base_cmd.linear.z = 1.0; // we cannot move vertically, this is our re-enable command
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
     msgSpeech.data = pickCompletedWav();
   }
   //disable
  else if(motionCmd[0]=='x'){

     base_cmd.angular.z = 0.0;
     base_cmd.linear.x = 0.0;
     base_cmd.linear.z = -1.0; // we cannot move vertically, this is our disable command
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
     msgSpeech.data = pickAnnoyedWav();
   }
   //cancel goal
   else if(motionCmd[0]=='c'){

     ac_->cancelAllGoals();
     ROS_INFO("canceling all goals");
     msgSpeech.data = pickAnnoyedWav();
   }	
   // move 1 meter forward
   else if(motionCmd[0]=='m'){
     	//we'll send a goal to the robot to move 1 meter forward
     	goal.target_pose.header.frame_id = "base_link";
     	goal.target_pose.header.stamp = ros::Time::now();

     	goal.target_pose.pose.position.x = 1.0;
      goal.target_pose.pose.position.y = 0.0;
      goal.target_pose.pose.orientation.w = 1.0;
      goal.target_pose.pose.orientation.z = 0.0;

      ROS_INFO("Sending goal");
     	ac_->sendGoal(goal);
      finishedInTime = ac_->waitForResult(ros::Duration(15));
      if (finishedInTime)
      {
          if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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
   }
   // move 10 meters forward 
   else if(motionCmd[0]=='n'){
     	//we'll send a goal to the robot to move 10 meters forward
     	goal.target_pose.header.frame_id = "base_link";
     	goal.target_pose.header.stamp = ros::Time::now();

     	goal.target_pose.pose.position.x = 10.0;
      goal.target_pose.pose.position.y = 0.0;
      goal.target_pose.pose.orientation.w = 1.0;
      goal.target_pose.pose.orientation.z = 0.0;

      ROS_INFO("Sending goal");
     	ac_->sendGoal(goal);
     
      finishedInTime = ac_->waitForResult(ros::Duration(15));
      if (finishedInTime)
      {
          if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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

   }
   // turn 90 degrees to the right
   else if(motionCmd[0]=='t'){
      goal.target_pose.header.frame_id = "base_link";
     	goal.target_pose.header.stamp = ros::Time::now();

     	goal.target_pose.pose.position.x = 0.0;
      goal.target_pose.pose.position.y = 0.0;
     	goal.target_pose.pose.orientation.w = sqrt(0.5);
      goal.target_pose.pose.orientation.z = -sqrt(0.5);

     	ROS_INFO("Sending goal");
     	ac_->sendGoal(goal);
      finishedInTime = ac_->waitForResult(ros::Duration(15));
      if (finishedInTime)
      {
          if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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
   // go to a specified pose on the map
   else if(motionCmd[0]=='p' && motionCmd.length() > 10)
   {
      ROS_INFO("publishing pose command received");
      goal.target_pose.header.frame_id = "map";
     	goal.target_pose.header.stamp = ros::Time::now();

      // take off the leading "p"
      std::string poseCmd = motionCmd.substr(1,std::string::npos);
      // locate the first comma, which arrives after the "x" value
      std::string poseDataBuffer[32];
      std::size_t found = poseCmd.find_first_of(",");
      int numPoseValues = 0;
      while (found!=std::string::npos)
      {
         poseDataBuffer[numPoseValues] = poseCmd.substr(0,found);
         numPoseValues++;
         poseCmd = poseCmd.substr(found + 1, std::string::npos);
         found = poseCmd.find_first_of(",");
      }

      if (numPoseValues == 4)
      {
         goal.target_pose.pose.position.x = atof(poseDataBuffer[0].c_str());
         goal.target_pose.pose.position.y = atof(poseDataBuffer[1].c_str());
         goal.target_pose.pose.orientation.z = atof(poseDataBuffer[2].c_str());
         goal.target_pose.pose.orientation.w = atof(poseDataBuffer[3].c_str());

     	   ROS_INFO("Sending goal pose");
     	   ac_->sendGoal(goal);
      }
      else ROS_WARN("Goal pose request has incorrect number of parameters");
   }

   speech_pub_.publish(msgSpeech);

}

bool movementLinear(double distance)
{
   move_base_msgs::MoveBaseGoal goal;
   bool finishedInTime;
  	goal.target_pose.header.frame_id = "base_link";
  	goal.target_pose.header.stamp = ros::Time::now();

  	goal.target_pose.pose.position.x = distance;
   goal.target_pose.pose.position.y = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;
   goal.target_pose.pose.orientation.z = 0.0;

   ROS_INFO("Sending goal");
  	ac_->sendGoal(goal);
  
   finishedInTime = ac_->waitForResult(ros::Duration(60));
   if (finishedInTime)
   {
       if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) return true;
   }
   return false;
}

bool movementAngular(double angle)  // for now we just turn 90 degrees to the right
{
   move_base_msgs::MoveBaseGoal goal;
   bool finishedInTime;
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);
   goal.target_pose.header.frame_id = "base_link";
  	goal.target_pose.header.stamp = ros::Time::now();

  	goal.target_pose.pose.position.x = 0.0;
   goal.target_pose.pose.position.y = 0.0;
  	goal.target_pose.pose.orientation = odom_quat;

  	ROS_INFO("Sending goal");
  	ac_->sendGoal(goal);
   finishedInTime = ac_->waitForResult(ros::Duration(15));
   if (finishedInTime)
   {
       if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) return true;
   }
   return false;

}

void keyboardCommandCallback(const std_msgs::String::ConstPtr& msg)
{
   std::string keyboardCommand = msg->data.c_str();
   sendMotionCommand(keyboardCommand);

}

void motionCommandCallback(const std_msgs::String::ConstPtr& msg)
{
   std::string motionCommand = msg->data.c_str();
   sendMotionCommand(motionCommand);
}

void movementCommandCallback(const outdoor_bot::movement_msg msg)
{
   double distance = msg.distance;
   double angle = msg.angle;
   if (abs(distance) > 0.01)
   {
      if (movementLinear(distance)) ROS_INFO("the base moved %f meters", distance);
      else ROS_INFO("the base failed to move");
   }

   if (abs(angle) > 0.01)
   {
      if (movementAngular(angle)) ROS_INFO("the base turned %f radians", angle);
      else ROS_INFO("the base failed to turn");
   }         
}

int main(int argc, char** argv)
{
   //init the ROS node
   ros::init(argc, argv, "commands_controller");
   ros::NodeHandle nh;
   cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   // and the publisher for speech output
   speech_pub_ = nh.advertise<std_msgs::String>("play_wavfile", 1);

   // subscribe to keyboard and speech commands
   keyboardCommandsSub_ = nh.subscribe("keyboard", 1, keyboardCommandCallback);
   motionCommandsSub_ = nh.subscribe("motion_command", 1, motionCommandCallback);
   movementCommandsSub_ = nh.subscribe("movement_command", 2, movementCommandCallback);
   ac_ = new MoveBaseClient("move_base",true);
   //wait for the action server to come up
   while(!ac_->waitForServer(ros::Duration(5.0))){
   ROS_INFO("commands controller is waiting for the move_base action server to come up");
   }
   ROS_INFO("commands controller sees that the move_base action server is up");
   ros::spin();
   delete ac_;
}


