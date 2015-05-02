// movement control for outdoor bot
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
#include "outdoor_bot/pmotor_msg.h"
#include "outdoor_bot/radar_service.h"
#include "outdoor_bot_defines.h"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class movementControl
{
private:

   ros::NodeHandle nh_;
   ros::ServiceClient radar_client_;
   ros::Publisher cmd_vel_pub_, complete_pub_, pmotor_pub_;
   // we subscribe motion and keyboard commands publishers
   ros::Subscriber keyboardCommandsSub_, motionCommandsSub_, movementCommandsSub_;

   // create the action client to send goals to move_base
   // true causes the client to spin its own thread
   MoveBaseClient *ac_;

   int distance_, angle_, map_or_bot_;
   int quatX_, quatY_, quatZ_, quatW_;
   std::string command_;
   
   double distanceToHome_[NUM_RADARS], deltaDistanceToHome_[NUM_RADARS], velocityFromHome_[NUM_RADARS];
   float range_;

public:
  movementControl(ros::NodeHandle &nh)
   :  nh_(nh)
{
   range_ = 0.;
   distance_ = 0;
   angle_ = 0;
   map_or_bot_ = 0;
   quatX_ = 0;
   quatY_ = 0;
   quatZ_ = 0;
   quatW_ = 0;
   command_ = "";
   cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

   // subscribe to keyboard and motion
   keyboardCommandsSub_ = nh.subscribe("keyboard", 1, &movementControl::keyboardCommandCallback, this);
   movementCommandsSub_ = nh.subscribe("movement_command", 2, &movementControl::movementCommandCallback, this);
   radar_client_ = nh.serviceClient<outdoor_bot::radar_service>("radar_service");
   complete_pub_ = nh.advertise<std_msgs::String>("movement_complete", 4);
   pmotor_pub_ = nh.advertise<outdoor_bot::pmotor_msg>("pmotor_cmd", 5);
   ac_ = new MoveBaseClient("move_base",true);
   //wait for the action server to come up
   while(!ac_->waitForServer(ros::Duration(5.0))){
   ROS_INFO("commands controller is waiting for the move_base action server to come up");
   }
   ROS_INFO("commands controller sees that the move_base action server is up");
}
   ~movementControl()
   {
      delete ac_;
   }
   
bool callRadarService(int radarNumber = LEFT_RADAR_NUMBER)
{
   outdoor_bot::radar_service::Request req;
   outdoor_bot::radar_service::Response resp;
   req.radarNumber = radarNumber;
   cout << "Calling radar service" << endl;
   bool success = radar_client_.call(req,resp);
   if (success)
   { 
      if (resp.distanceToHome > 0.)
      {
         int radarIndex;
         if (radarNumber == LEFT_RADAR_NUMBER) radarIndex = 0;
         if (radarNumber == RIGHT_RADAR_NUMBER) radarIndex = 1;
         distanceToHome_[radarIndex] = resp.distanceToHome;
         deltaDistanceToHome_[radarIndex] = resp.deltaDistanceToHome;
         velocityFromHome_[radarIndex] = resp.velocityFromHome;
      }
      return true;
   }
   return false;
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
   } 
   //turn left (yaw) 
   else if(motionCmd[0]=='l'){
     base_cmd.angular.z = 0.75;
     base_cmd.linear.x = 0.0;
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
   } 
   //turn right (yaw)
   else if(motionCmd[0]=='r'){
     base_cmd.angular.z = -0.75;
     base_cmd.linear.x = 0.0;
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
   }
   //move backwards
   else if(motionCmd[0]=='b'){
     base_cmd.linear.x = -0.15;
     base_cmd.angular.z = 0.0;
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
   } 
   //stop
   else if(motionCmd[0]=='s'){
     base_cmd.angular.z = 0.0;
     base_cmd.linear.x = 0.0;
     base_cmd.linear.z = 1.0; // we cannot move vertically, this is our re-enable command
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
   }
   //disable
  else if(motionCmd[0]=='x'){

     base_cmd.angular.z = 0.0;
     base_cmd.linear.x = 0.0;
     base_cmd.linear.z = -1.0; // we cannot move vertically, this is our disable command
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
   }
   //cancel goal
   else if(motionCmd[0]=='c'){

     ac_->cancelAllGoals();
     ROS_INFO("canceling all goals");
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
          }
          else
          {
            ROS_INFO("The base failed to move forward 1 meter for some reason"); 
          }
      }
      else
      {
         ROS_INFO("Timed out moving to goal");
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
            ROS_INFO("Hooray, the base moved 10 meters forward");
          }
          else
          {
            ROS_INFO("The base failed to move forward 10 meters for some reason"); 
          }
      }
      else
      {
         ROS_INFO("Timed out moving to goal");
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
          }
          else
          {
            ROS_INFO("The base failed to turn 90 degrees right for some reason"); 
          }
      }
      else
      {
         ROS_INFO("Timed out moving to goal");
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

}

bool movementLinear(double distance)
{
   move_base_msgs::MoveBaseGoal goal;
   bool finishedInTime;
  	goal.target_pose.header.frame_id = "base_footprint";
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

bool movementPose(double x, double y, double thetaDegrees)
{
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
   goal.target_pose.pose.position.x = 0.; //x;
   goal.target_pose.pose.position.y = 0.; // y;
   goal.target_pose.pose.orientation.w = 1.; //cos ((thetaDegrees/2.)*(3.14/180));
   goal.target_pose.pose.orientation.z = 0.; //sin ((thetaDegrees/2.)*(3.14/180));


   ROS_INFO("Sending goal");
  	ac_->sendGoal(goal);
  
   bool finishedInTime = ac_->waitForResult(ros::Duration(60));
   if (finishedInTime)
   {
       if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) return true;
   }
   return false;
}

void keyboardCommandCallback(const std_msgs::String::ConstPtr& msg)
{
   std::string keyboardCommand = msg->data.c_str();
   sendMotionCommand(keyboardCommand);

}

void movementCommandCallback(const outdoor_bot::movement_msg msg)
{
   
   string command = msg.command;
   std_msgs::String msgResult;
		cout << "movement callback in movement node " << endl;    
   if (!command.compare("move"))	// command bot to enter pause mode
	{
		cout << "moving in movement node " << endl;
		double distance = msg.distance;
      if (movementLinear(distance)) 
      {
         ROS_INFO("the base moved %f meters", distance);
         msgResult.data = "move_done";
      }
      else
      {
         ROS_INFO("in movement node, the base failed to move");
         msgResult.data = "failed";
      }
      complete_pub_.publish(msgResult);
      return;
   }
   
   if (!command.compare("turn"))	// command bot to enter pause mode
   {
      double angle = msg.angle;		
		if (movementAngular(angle))
      {
         ROS_INFO("the base turned %f radians", angle);
         msgResult.data = "turn_done";
      }
      else
      {
         ROS_INFO("in movement node, the base failed to turn");
         msgResult.data = "failed";
      }
      complete_pub_.publish(msgResult);
      return;
   }
   
   if (!command.compare("pose"))	// command bot to enter pause mode 
   {
   	double poseX = msg.poseX;
   	double poseY = msg.poseY;
   	double poseTheta = msg.poseThetaDegrees;
   	if (movementPose(poseX, poseY, poseTheta))
   	{   
         cout << "the base moved to new pose" << endl;
         msgResult.data = "pose_done";
      }
      else
      {
         ROS_INFO("in movement node, the base failed to move to new pose");
         msgResult.data = "failed";
      }
      complete_pub_.publish(msgResult);
      return;  
    }
    
    if (!command.compare("platform"))	// time for final approach
    {
      cout << "final approach phase in movement node" << endl;
      range_ = -1.;
      if (callRadarService(LEFT_RADAR_NUMBER) && distanceToHome_[0] > 0.01) range_ = distanceToHome_[0]; 
      cout << "Heading for Home, range = " << range_ << endl;
      if (range_ < 0.01)
      {
      	ROS_INFO("failed to get range data in movement_node, so the base failed to move");
         msgResult.data = "failed";
         complete_pub_.publish(msgResult);
         return;
      }
      
      float startingRange = range_;
      geometry_msgs::Twist movement_cmd;
		// our bot does not use these motions, set them to 0
		movement_cmd.angular.x = 0.0;
		movement_cmd.angular.y = 0.0;
		movement_cmd.linear.y = 0.0;
		// we do use these motions, but start them at 0 too
		movement_cmd.linear.x = 0.0;
		movement_cmd.linear.z = 0.0;   // this is our enable command.  start enabled.
		movement_cmd.angular.z = 0.0; 
      movement_cmd.linear.x = -0.15;
      movement_cmd.angular.z = 0.0;
      //publish the assembled command and start moving forward
      cmd_vel_pub_.publish(movement_cmd);
      int stopCount = 0;
      int notProgressingCounter = 0;
      while ((range_ > FINAL_RANGE) && stopCount < 10 && notProgressingCounter < 100)
      {
      	if (callRadarService(LEFT_RADAR_NUMBER)) 
      	{
      		if (distanceToHome_[0] > 0.01) range_ = distanceToHome_[0]; 
      		cout << "current range in movement node = " << range_ << endl;
      		notProgressingCounter++;
      		struct timespec ts;
				ts.tv_sec = 0;
				ts.tv_nsec = 100000000;
				nanosleep(&ts, NULL); // update every 100 ms, we are limited by the rate that robotPose updates this value
      	}
      	else
      	{
      		cout << "failed to get range data in movement node" << endl;
      		stopCount++;
      	}
      }
      movement_cmd.angular.z = 0.0;
      movement_cmd.linear.x = 0.0;
      movement_cmd.linear.z = 1.0; // we cannot move vertically, this is our re-enable command
      //publish the assembled command
      cmd_vel_pub_.publish(movement_cmd); // stopping on the platform
      
      
      if (callRadarService(LEFT_RADAR_NUMBER) && distanceToHome_[0] > 0.01) range_ = distanceToHome_[0];
      cout << "final range in movement node = " << range_ << endl;
      cout << "base moved " << startingRange - range_ << " meters" << endl;
      msgResult.data = "platform_done";
      complete_pub_.publish(msgResult);
      return;
    }

   if (!command.compare("all_done"))	// command bot to enter pause mode
   {
   	cout << "all done phase in movement node" << endl;
   	outdoor_bot::pmotor_msg pmotorMsg;
   	pmotorMsg.pmotorNumber = MOTOR_BRAKE_SOLENOID;
   	pmotorMsg.pmotorSpeed = PD_SPEED_MAX_VALUE + 10; // this is the code for arduino to put the robot in pause mode
   	pmotor_pub_.publish(pmotorMsg);
   	msgResult.data = "all_done";
      complete_pub_.publish(msgResult);
   	return;
   } 
       
}
};

int main(int argc, char** argv)
{
   //init the ROS node
   ros::init(argc, argv, "commands_controller");
   ros::NodeHandle nh;
   movementControl mC(nh);

   ros::spin();

}


