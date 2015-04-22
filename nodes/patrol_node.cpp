// patroller node

// setup waypoints for patrolling robot
// for quaternion specification of rotation:
//[w, x, y, z] = [cos(a/2), sin(a/2) * nx, sin(a/2)* ny, sin(a/2) * nz]
// Where a is the angle of rotation and {nx,ny,nz} is the axis of rotation.
// we always want to rotate around the z axis, so x = y = 0
// for 0 rotation, just use w = 1, x=y=z=0

#include <ctime>
//#include <mutex>
//#include <thread>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define NUM_GOALS 6
#define HOME 10       // waypoint designated as home base
#define HOME_X 0.0
#define HOME_Y 0.0
#define BATTERY_RETURN_HOME 11.0 // main battery voltage that causes us to go home
#define BATT_DIGITAL_MAX 883
#define BATT_FULL_VOLTS 12.7
#define LAPTOP_BATT_RETURN_HOME 20.  // laptop battery percentage that causes us to go home
#define DISCHARGING 0
#define CHARGING 1

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

volatile bool patrolling_ = false, goHome_ = false, finished_ = false;
//double x_ = 0., y_ = 0., z_ = 0., w_ = 1.; // identity quaternion
double varX_ = 100., varY_ = 100., varYaw_ = 100.;
double batteryVolts_ = 10.5;
double laptopBattPercentage_ = 20.;
int chargingState_ = DISCHARGING;
bool goalReached_ = false;
move_base_msgs::MoveBaseGoal goal_;
MoveBaseClient *ac_;

ros::Publisher poseGoal_pub_, speech_pub_;
ros::Subscriber poseWithCovariance_, sensors_, keyboardCommandsSub_;

double Xwaypoint_[100], Ywaypoint_[100], Twaypoint_[100];
std::string Swaypoint_[100];
std::string sensorReadings_;
boost::mutex myLock;

void waypointsInit()
{
	// mid living room
	Xwaypoint_[0] = HOME_X + 2.0;
	Ywaypoint_[0] = HOME_Y + 0.5;
	Twaypoint_[0] = 90.0;	// positive is counterclockwise
   Swaypoint_[0] = "mid living room, 90 yaw";

	// front living room
	Xwaypoint_[1] = HOME_X + 1.6;
	Ywaypoint_[1] = HOME_Y + 3.3;
	Twaypoint_[1] = 135.0;
   Swaypoint_[1] = "front living room, 135 yaw";

	// front door
	Xwaypoint_[2] = HOME_X - 2.5;
	Ywaypoint_[2] = HOME_Y + 4.5;
	Twaypoint_[2] = -90.0;
   Swaypoint_[2] = "front door, -90 yaw";

   // hall
	Xwaypoint_[3] = HOME_X - 2.5;
	Ywaypoint_[3] = HOME_Y + 1.0;
	Twaypoint_[3] = -90.;
   Swaypoint_[3] = "hall, -90 yaw";

   // kitchen
	Xwaypoint_[4] = HOME_X - 2.0;
	Ywaypoint_[4] = HOME_Y - 1.4;
	Twaypoint_[4] = 0.;
   Swaypoint_[4] = "kitchen 0 yaw";

	// dining room
   Xwaypoint_[5] = HOME_X + 1.5;
	Ywaypoint_[5] = HOME_Y - 1.2;
	Twaypoint_[5] = 90.;
   Swaypoint_[5] = "dining room, 90 yaw";

	// kitchen table
   Xwaypoint_[6] = HOME_X - 6.0;
	Ywaypoint_[6] = HOME_Y - 1.2;
	Twaypoint_[6] = 180.;
   Swaypoint_[6] = "kitchen table, 180 yaw";

	// mudroom
   Xwaypoint_[7] = HOME_X - 10.0;
	Ywaypoint_[7] = HOME_Y + 0.5;
	Twaypoint_[7] = 180;
   Swaypoint_[7] = "mudroom, 180 yaw";

	// familyroom
   Xwaypoint_[8] = HOME_X - 14.0;
	Ywaypoint_[8] = HOME_Y + 0.5;
	Twaypoint_[8] = 0.0;
   Swaypoint_[8] = "familyroom, 0 yaw";

	// sue's office
   Xwaypoint_[9] = HOME_X - 6.0;
	Ywaypoint_[9] = HOME_Y + 3.5;
	Twaypoint_[9] = 90.;
   Swaypoint_[9] = "Sue's office, 90 yaw";

	// near home facing out
	Xwaypoint_[10] = 1.3;
	Ywaypoint_[10] = 0.1;
	Twaypoint_[10] = 0.;
   Swaypoint_[10] = "near home facing out, 0 yaw";

	// home facing in
	Xwaypoint_[11] = 0.0;
	Ywaypoint_[11] = 0.0;
	Twaypoint_[11] = 180.;
   Swaypoint_[11] = "home facing in, 180 yaw";

}

/*
// clock_t is a like typedef unsigned int clock_t. Use clock_t instead of integer in this context
void mySleep(clock_t millisec) 
{
   clock_t start_time = clock();
   clock_t end_time = millisec * 1000 + start_time;
   while(clock() < end_time);
} 
*/

void setVariances(double vX, double vY, double vYaw)
{
   varX_ = vX;
   varY_ = vY;
   varYaw_ = vYaw;
}

void getVariances(double *vX, double *vY, double *vYaw)
{
   *vX = varX_;
   *vY = varY_;
   *vYaw = varYaw_;
}


bool reduceVariance()
{
   int counter = 1;
   char buffer[2];
   // rotate back and forth in position up to four times to see walls and reduce variance
   double vX, vY, vYaw;
   ros::spinOnce();  // allow the callbacks a chance to see a new pose and update variance
   getVariances(&vX, &vY, &vYaw);
   ROS_INFO("Checking variances: vx = %f, vy = %f, vYaw = %f", vX, vY, vYaw); 
   while ((vX > 0.05 || vY > 0.05 || vYaw > .01) && counter < 5)
   {
      if (counter % 2) buffer[0] = 'r';
      else buffer[0] = 'l';
      std_msgs::String rotate;
      rotate.data = buffer;
      poseGoal_pub_.publish(rotate);
      usleep(10000000);
      buffer[0] = 's';
      rotate.data = buffer;
      poseGoal_pub_.publish(rotate);
      usleep(1000000);
      counter++;
      ros::spinOnce();  // allow the callbacks a chance to see a new pose and update variance
      getVariances(&vX, &vY, &vYaw);
      ROS_INFO("After rotating to reduce variance: vx = %f, vy = %f, vYaw = %f", vX, vY, vYaw);
   }
   if (vX > 0.05 || vY > 0.05 || vYaw > .01)
   {
      ROS_INFO("Variance too high to continue: vx = %f, vy = %f, vYaw = %f", vX, vY, vYaw);
      return false;
   }
   ROS_INFO("Variance is within limits");
   return true;
}

bool sendWaypoint(int waypointNumber)
{
   static int firstPatrol = true;
   if (firstPatrol)  // after initial bootup, move out from home base, then reduce variance
   {
      char buffer[5] = "s";
      ROS_INFO("Enabling keyboard commands");
      std_msgs::String forward;
      forward.data = buffer;
      poseGoal_pub_.publish(forward);
      usleep(500000);
      ROS_INFO("Moving forward to start patrol");
      buffer[0] = 'f';
      forward.data = buffer;
      poseGoal_pub_.publish(forward); 
      usleep(6000000);
      ROS_INFO("Stopped, checking location, starting patrol");
      buffer[0] = 's';
      forward.data = buffer;
      poseGoal_pub_.publish(forward);
      usleep(1000000);
      firstPatrol = false;
   }

   if (!reduceVariance()) return false; // if variance is too high, we don't know where we are well enough to find a goal
  
   goal_.target_pose.header.stamp = ros::Time::now();
   goal_.target_pose.pose.position.x = Xwaypoint_[waypointNumber];
   goal_.target_pose.pose.position.y = Ywaypoint_[waypointNumber];
   goal_.target_pose.pose.orientation.w = cos ((Twaypoint_[waypointNumber]/2)*(3.14/180));
   goal_.target_pose.pose.orientation.z = sin ((Twaypoint_[waypointNumber]/2)*(3.14/180));

   ROS_INFO("Sending goal of %s", Swaypoint_[waypointNumber].c_str());
   ac_->sendGoal(goal_);
   ac_->waitForResult();

   if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
   {
      ROS_INFO("goal reached ");
      goalReached_ = true;
   }
   else
   {
       ROS_INFO("failed to reach goal");
       goalReached_ = false;
       // provide some time and opportunity for messages
       for (int i=0; i < 5; i++)
       {
         usleep(500000);  // sleep for 0.5 sec
         ros::spinOnce();
       }
       double vX, vY, vYaw;
       getVariances(&vX, &vY, &vYaw);
       ROS_INFO("After failing goal and waiting a bit, variance: vx = %f, vy = %f, vYaw = %f", vX, vY, vYaw);
   }
   return true;  // return true even if we fail the goal, since we can reduce variance and try again for the next one
}



//void poseCommandCallback(const boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped>& msg) // this works
void poseCommandCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) // but this is cleaner
{
   geometry_msgs::PoseWithCovariance poseWithCovariance = msg->pose;
   geometry_msgs::Pose pose = poseWithCovariance.pose;
   geometry_msgs::Point position = pose.position;
   geometry_msgs::Quaternion orientation = pose.orientation;
   double x = position.x;
   double y = position.y;
   double z = orientation.z;
   double w = orientation.w;
   //setPositions(x,y,z,w);

   double varianceX = poseWithCovariance.covariance[0];      // x variance
   double varianceY = poseWithCovariance.covariance[7];      // y variance
   double varianceYaw = poseWithCovariance.covariance[35];   // Yaw variance
   setVariances(varianceX, varianceY, varianceYaw);

   //ROS_INFO("pose received by patrol_node: ");
   //ROS_INFO("pose position x,y = %f, %f", x, y);
   //ROS_INFO("pose orientation w,z = %f, %f", w, z);
   //ROS_INFO("variance x,y,Yaw = %f, %f, %f", varianceX, varianceY, varianceYaw);
}

void keyboardCommandCallback(const std_msgs::String::ConstPtr& msg)
{
   std::string keyboardCommand = msg->data.c_str();
   ROS_INFO("robotPose_node received keyboard command, %s", keyboardCommand.c_str());
   if (keyboardCommand[0] == 'p' && keyboardCommand.length() < 10) // toggle patrolling
   {
      if (patrolling_ || finished_)
      {
         patrolling_ = false;
         finished_ = true;
         char buffer[5] = "c";
         std_msgs::String cancelPatrol;
         cancelPatrol.data = buffer;
         poseGoal_pub_.publish(cancelPatrol);
         ROS_INFO("patrolling finished");         
      }
      else
      {
         patrolling_ = true;
         goHome_ = false;
         ROS_INFO("starting to patrol");
      }
   }
   if (keyboardCommand[0] == 'h') // time to wrap it up and go home
   {
         patrolling_ = false;
         finished_ = true;
         goHome_ = true;
         char buffer[5] = "c";
         std_msgs::String cancelPatrol;
         cancelPatrol.data = buffer;
         poseGoal_pub_.publish(cancelPatrol);
         ROS_INFO("patrolling finished, heading home");
   }
}

void sensorReadingsCallback(const std_msgs::String::ConstPtr& msg)
{
   myLock.lock(); // for some reason, without the lock, it does not pick up the laptop message
   sensorReadings_ = msg->data.c_str();
   std::size_t found;

   found = sensorReadings_.find("charging");
   if (found != std::string::npos)
   {
      found = sensorReadings_.find("discharging");
      if (found != std::string::npos) chargingState_ = DISCHARGING;
      else chargingState_ = CHARGING;
   } 

   found = sensorReadings_.find("%");
   if (found != std::string::npos && found > 7) // % character says this is a laptop battery reading
   {
      std::string percentageString = sensorReadings_.substr(found - 7, 7);    
      laptopBattPercentage_ = atof(percentageString.c_str());
      //ROS_INFO("Laptop battery percentage = %f", laptopBattPercentage_);
   }

   else
   {
      found = sensorReadings_.find("=");  // = character says this is a robot main battery reading
      if (found != std::string::npos)
      {
         std::string batteryString = sensorReadings_.substr(found + 1, std::string::npos);    
         double value = atof(batteryString.c_str());
         if (value > BATT_DIGITAL_MAX) value = BATT_DIGITAL_MAX;
         batteryVolts_ = BATT_FULL_VOLTS - (0.12*(BATT_DIGITAL_MAX - value));
         if (batteryVolts_ < 0.) batteryVolts_ = 0.;
         //ROS_INFO("Main robot battery voltage = %f", batteryVolts_);
      }
   }
   myLock.unlock();
}      

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "patrol_node");
  ros::NodeHandle nh;
  poseGoal_pub_ = nh.advertise<std_msgs::String>("keyboard", 5);  // send out keyboard commands, these generate poses from amcl
  speech_pub_ = nh.advertise<std_msgs::String>("play_wavfile", 1);    // and the publisher for speech output
  poseWithCovariance_ = nh.subscribe("amcl_pose", 10, poseCommandCallback); // subscribe to amcl pose
  keyboardCommandsSub_ = nh.subscribe("keyboard", 1, keyboardCommandCallback);  // subscribe to keyboard commands
  sensors_ = nh.subscribe("sensor_readings",5, sensorReadingsCallback);
  waypointsInit(); // setup our patrol waypoints

  //tell the action client that we want to spin a thread by default
  ac_ = new MoveBaseClient("move_base",true);

  //wait for the action server to come up
  while(!ac_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  goal_.target_pose.header.frame_id = "map";  // we want coordinates on the map, not with respect to the robot
  int counter = 0, loopNumber = 1;
  double lastTime, totalTime;

  ROS_INFO("Patrol bot is ready for commands");

  while (nh.ok())
  {
      if (patrolling_)
      {
         if (counter == 0 && loopNumber == 1)
         {
            lastTime = ros::Time::now().toSec();
            totalTime = 0.;
         }
         if (sendWaypoint(counter))
         {
            counter++;
            if (counter > NUM_GOALS-1)
            {
               ros::spinOnce();  // make sure we have the latest messages, especially for battery
               double loopTime = ros::Time::now().toSec() - lastTime;
               lastTime = ros::Time::now().toSec();
               totalTime += loopTime;
               ROS_INFO("completed patrol loop %d, loop time = %f, total time = %f secs, batterystring is %s, battery volts = %f, laptop battery percentage = %f", loopNumber, loopTime, totalTime, sensorReadings_.c_str(), batteryVolts_, laptopBattPercentage_);

               if (batteryVolts_ <= BATTERY_RETURN_HOME  || laptopBattPercentage_ <= LAPTOP_BATT_RETURN_HOME)
               {
                  //patrolling_ = false;
                  goHome_ = true;
                  finished_ = true;
                  ROS_INFO("battery or laptop needs charging, battery volts = %f, laptop battery percentage = %f, heading home", batteryVolts_, laptopBattPercentage_);
               std_msgs::String msgSpeech;
               msgSpeech.data = "/MyBatterysLowPleasePlugMeIn.wav";
               speech_pub_.publish(msgSpeech);
               }                  
               loopNumber++;
               counter = 0;
             }
         }
         else patrolling_ = false;
         ros::spinOnce();  // update messages, especially want to know if new keyboard command sent
      }
      ros::spinOnce(); // update messages, especially want to know if new keyboard command sent
      //usleep(1000000);  //wait 1 sec at the waypoint before going to the next one
      if (finished_)
      {
         if (goHome_) // finish up by going home if we got the 'h' command or low battery
         {
            if (sendWaypoint(HOME))
            {
               if (goalReached_) // don't want to back up if we didn't make it to home pre-position.
               {
                  char buffer[5] = "b";
                  std_msgs::String backward;
                  backward.data = buffer;
                  ROS_INFO("Backing into home location");
                  poseGoal_pub_.publish(backward);  
                  usleep(6000000);
                  buffer[0] = 's';
                  backward.data = buffer;
                  poseGoal_pub_.publish(backward); 
               }
            }
         }

         // all done, just waiting for user to close the node, now hitting 'h' or 'p' will just cancel the home goal
         bool pluggedIn;
         if (chargingState_)
         {
            pluggedIn = true;
            ROS_INFO("Already plugged in");
         }
         else 
         {
            pluggedIn = false;
            ROS_INFO("Not plugged in yet");
         }
         long bugCounter = 10;   // ask to be plugged in every 10th loop
         long waitMicrosec = 5000000; // loop every 5 seconds to see if we are charging
         if (chargingState_) waitMicrosec = 100000000;  // loop every 100 seconds if we are charging
         while (nh.ok())
         {
            ROS_INFO("Monitoring charge state");
            if (chargingState_ && (!pluggedIn)) // somebody plugged us in!
            {
               ROS_INFO("Got plugged in");
               pluggedIn = true;
               std_msgs::String msgSpeech;
               msgSpeech.data = "/ThanksForPluggingMeIn.wav";
               speech_pub_.publish(msgSpeech);
               ROS_INFO("Thanking for plugging in");
               waitMicrosec = 100000000; // now loop only once every 100 seconds 
            } 
 
            if ( (!chargingState_) && (!pluggedIn) && (!bugCounter % 10))
            {
               std_msgs::String msgSpeech;
               msgSpeech.data = "/MyBatterysLowPleasePlugMeIn.wav";
               speech_pub_.publish(msgSpeech);
               ROS_INFO("Asking to be plugged in");
            }  
            bugCounter++;

            if (chargingState_ && laptopBattPercentage_ > 95 && batteryVolts_ > 12.6)
            {
               ROS_INFO("Fully charged");
               std_msgs::String msgSpeech;
               msgSpeech.data = "/AllChargedUpReadyToWork.wav";
               speech_pub_.publish(msgSpeech); 
            }                        
            ros::spinOnce();
            usleep(waitMicrosec);
         }
      }
  }
  
  delete ac_;
  ROS_INFO("patrol_node exiting");
  ros::spinOnce(); // let that last ROS_INFO get out there.
}
