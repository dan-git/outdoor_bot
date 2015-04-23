#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/QuadWord.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "outdoor_bot/encoders_service.h"
#include "outdoor_bot/dirAnt_service.h"
#include "outdoor_bot/radar_service.h"
#include "outdoor_bot/setPose_service.h"
#include "rcmRadar/radar.h"
#include "outdoor_bot_defines.h"
#include <sstream>


#define OUTDOOR_TICKS_PER_METER 50
/*
#define SMOOTH_POINTS 0 // roger has low resolution encoders and sometimes needs points smoothed
                        // for now we are smoothing on the arduino side to help the pid calcs
                        // so no need to smooth here
*/

boost::array<double, 36>  ODOM_POSE_COVARIANCE = {
                       {1e-3, 0, 0, 0, 0, 0,  // covariance on x
                        0, 1e-3, 0, 0, 0, 0,  // covariance on y
                        0, 0, 1e6, 0, 0, 0,   // large covariance on z, since we do not move on that axis
                        0, 0, 0, 1e6, 0, 0,   // large covariance on rot x, since we do not rotate around that axis
                        0, 0, 0, 0, 1e6, 0,   // large covariance on rot y, since we do not rotate around that axis
                        0, 0, 0, 0, 0, 1e-3}};  // covariance on rot z

// remember that odom pose uses the parent frame (odom)
// while odom twist uses the child frame (base-footprint)
// so the covariances for x and y are both 1e-9, but the covariances for vx and vy are 1e-3 and 1e6 
boost::array<double, 36> ODOM_POSE_STOPPED_COVARIANCE = {
                        {1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-9, 0, 0, 0, 0,   // turtlebot has a nonzero yz entry, dont know why
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9}};

boost::array<double, 36> ODOM_TWIST_COVARIANCE = {
                        {1e-3, 0, 0, 0, 0, 0, // covariance on vx
                         0, 1e6, 0, 0, 0, 0,  // large covariance on vy (in robot frame vy = 0)
                         0, 0, 1e6, 0, 0, 0,  // large covariance on vz
                         0, 0, 0, 1e6, 0, 0,  // large covariance on rot vx
                         0, 0, 0, 0, 1e6, 0,  // large covariance on rot vy
                         0, 0, 0, 0, 0, 1e-3}}; // covariance on rot vz

boost::array<double, 36>  ODOM_TWIST_STOPPED_COVARIANCE = {
                         {1e-9, 0, 0, 0, 0, 0, 
                          0, 1e6, 0, 0, 0, 0, // turtlebot has a nonzero yz entry, dont know why
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9}};

boost::array<double, 9> LINEAR_ACCELERATION_COVARIANCE = {
    {1e-3, 0, 0,   //covarience on accelX
	   0, 1e-3, 0,  //covarience on accelY
     	0, 0, 1e-3}}; //covarience on accelZ

boost::array<double, 9> ANGULAR_VELOCITY_COVARIANCE = {
    {1e6, 0, 0,    // large covarience on rot x
	   0, 1e6, 0,   // large covariance on rot y
      0, 0, 1e-3}}; // covarience on rot z

boost::array<double, 9> ORIENTATION_COVARIANCE = {
    {-1.0, 0, 0, //covarience = -1 since we do not have an orientation
		0, 1e6, 0,  
     	0, 0, 1e6}}; 

using namespace std;

ros::Subscriber ucResponseMsg, assignedPose, poseWithCovariance_, moveCmd;
ros::Publisher odom_pub, imu_pub, sensors_pub, pause_pub;
ros::ServiceServer encoders_serv, radar_serv, dirAnt_serv, setPose_serv;
tf::TransformBroadcaster *odom_broadcaster;	// have to use a pointer because declaring this before running
						// ros:init causes a run-time error

double ticksPerMeter_;
double accelX, accelY, accelZ;
double velocityLeft, velocityRight, dtROS_;
long EncoderTicksRight = 0, EncoderTicksLeft = 0, previousEncoderTicksRight = 0, previousEncoderTicksLeft = 0;
long EncoderPickerUpper = 0, EncoderBinShade = 0, EncoderDropBar = 0, EncoderExtra = 0;
int battery, pauseState = 0, dirAntMaxDir = 0, dirAntMaxTilt = 0,  dirAntLevel = 0;
unsigned long arduinoCycleTime, arduinoDataCounter;
bool firstTime = true;
ros::Time current_time, last_time;
radarRanger rRanger_;   // remember to set the serial port for this to work!
double homeX_ = 0., homeY_ = 0., homeYaw_ = 0.;
double distanceToHome_[NUM_RADARS], previousDistanceToHome_[NUM_RADARS], deltaDistanceToHome_[NUM_RADARS];
int destNode_[NUM_RADARS], radarCounter_ = 0;
double velocityFromHome_[NUM_RADARS], currentVelocity_ = 0.; 


//double x = 24.0, y = 19.0, yaw = 2.3;	// match with initial pose specificed in _amcl_hokuyo.launch
double x = 0., y = 0., yaw = 0.;
double vYaw = 0.0;


/*
void smoothOdometry(double *distanceMoved)
{
   if (SMOOTH_POINTS <= 1) return;
   double static distanceArray[SMOOTH_POINTS];
   int static numPoints = 0, maxPoints = 0; 
   if (maxPoints == 0) for (int i=0; i < SMOOTH_POINTS; i++) distanceArray[i] = 0.0;  // zero the array first time through
   distanceArray[numPoints] = *distanceMoved;
   if (numPoints > SMOOTH_POINTS - 1) numPoints = 0;
   else numPoints++;
   if (numPoints > maxPoints) maxPoints = numPoints;
   double totalValue = 0;
   for (int i=0; i < maxPoints; i++) totalValue += distanceArray[i];
   *distanceMoved = totalValue / ( (double)(maxPoints));
 } 
*/ 

bool getRadarData(int destNodeNumber)
{
   double currentDistanceToHome  = ((double) rRanger_.getRange(destNodeNumber));	// returns value in mm  
   
   if (currentDistanceToHome > 0.01)
   {
      if (destNodeNumber == LEFT_RADAR_NUMBER) 
      {
		   distanceToHome_[LEFT_RADAR_INDEX] = currentDistanceToHome / 1000.;	// convert to meters
		   deltaDistanceToHome_[LEFT_RADAR_INDEX] = distanceToHome_[LEFT_RADAR_INDEX] - previousDistanceToHome_[LEFT_RADAR_INDEX];
		   previousDistanceToHome_[LEFT_RADAR_INDEX] = distanceToHome_[LEFT_RADAR_INDEX];
		   if (dtROS_ > 0) velocityFromHome_[LEFT_RADAR_INDEX] =  deltaDistanceToHome_[LEFT_RADAR_INDEX] / dtROS_;
		}
      
      else if (destNodeNumber == RIGHT_RADAR_NUMBER) 
      {
		   distanceToHome_[RIGHT_RADAR_INDEX] = currentDistanceToHome / 1000.;	// convert to meters
		   deltaDistanceToHome_[RIGHT_RADAR_INDEX] = distanceToHome_[RIGHT_RADAR_INDEX] - previousDistanceToHome_[RIGHT_RADAR_INDEX];
		   previousDistanceToHome_[RIGHT_RADAR_INDEX] = distanceToHome_[RIGHT_RADAR_INDEX];
		   if (dtROS_ > 0) velocityFromHome_[RIGHT_RADAR_INDEX] =  deltaDistanceToHome_[RIGHT_RADAR_INDEX] / dtROS_;
		}
		
      else if (destNodeNumber == CENTER_RADAR_NUMBER) 
      {
		   distanceToHome_[CENTER_RADAR_INDEX] = currentDistanceToHome / 1000.;	// convert to meters
		   deltaDistanceToHome_[CENTER_RADAR_INDEX] = distanceToHome_[CENTER_RADAR_INDEX] - previousDistanceToHome_[CENTER_RADAR_INDEX];
		   previousDistanceToHome_[CENTER_RADAR_INDEX] = distanceToHome_[CENTER_RADAR_INDEX];
		   if (dtROS_ > 0) velocityFromHome_[CENTER_RADAR_INDEX] =  deltaDistanceToHome_[CENTER_RADAR_INDEX] / dtROS_;
		}
				
      return true;
   }
   //if (currentDistanceToHome == -2) radarStatus_ = false;
   cout << "radar not reporting" << endl;
   return false;
}
   
  void publishPose(double poseX, double poseY, double poseYaw, double pose_vYaw, double pose_currentVelocity)
  {
     
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(poseYaw);

    // Turtlebot way to get quaternion from yaw uses simplified version
    // of tf.transformations.quaternion_about_axis
    // geometry_msgs::Quaternion  odom_quat = (0., 0., sin(yaw/2.), cos(yaw/2.));   

  
    // Since we are using robot_pose_ekf, we do not want to publish a tf
    // from base_footprint to odom
    // robot_pose_ekf will publish a tf from base_footprint to odom
    // and then base_footprint would have two parent nodes, which won't work
    
      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_footprint";

      odom_trans.transform.translation.x = poseX;
      odom_trans.transform.translation.y = poseY;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster->sendTransform(odom_trans);
     
      
      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";

      // set the position  NOTE:  this is in the header frame (odom)
      odom.pose.pose.position.x = poseX;
      odom.pose.pose.position.y = poseY;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;
        
      // set the velocity  NOTE: this is in the child frame (base_footprint)
      odom.twist.twist.linear.x = pose_currentVelocity;
      odom.twist.twist.linear.y = 0;
      odom.twist.twist.angular.z = pose_vYaw;

      if (abs(pose_currentVelocity) < 0.01 )
      {
        odom.pose.covariance = ODOM_POSE_STOPPED_COVARIANCE;
        odom.twist.covariance = ODOM_TWIST_STOPPED_COVARIANCE;
      }
      else
      {
        odom.pose.covariance = ODOM_POSE_COVARIANCE;
        odom.twist.covariance = ODOM_TWIST_COVARIANCE;
      }

      //publish the message
      odom_pub.publish(odom);

    								  
    //next, we'll publish the Imu message over ROS.  This is for use with robot_pose_EKF ************ uncomment this if you decide to go back to using EKF
    /*sensor_msgs::Imu imu;
    imu.header.stamp = current_time;
    // be sure to put this link into urdf or there won't be a transform for it
    imu.header.frame_id = "Imu_link";

    imu.orientation_covariance = ORIENTATION_COVARIANCE;
    imu.angular_velocity_covariance = ANGULAR_VELOCITY_COVARIANCE;
    imu.linear_acceleration_covariance = LINEAR_ACCELERATION_COVARIANCE;

    geometry_msgs::Quaternion orientation;
    orientation.x = 0;	// identity quaternion, since we are not reporting an orientation from the imu
    orientation.y = 0;
    orientation.z = 0;
    orientation.w = 1;
    imu.orientation = orientation;

    geometry_msgs::Vector3 angV;
    angV.x = 0.0;
    angV.y = 0.0;
    angV.z = vYaw;
    imu.angular_velocity = angV;	//rads/sec

    geometry_msgs::Vector3 linearAccel;
    linearAccel.x = accelX;
    linearAccel.y = accelY;
    linearAccel.z = accelZ;
    imu.linear_acceleration = linearAccel;	// meters/(sec*sec)

    // publish the message
    imu_pub.publish(imu);
	*/
  /*
  ROS_INFO("dtROS_, dtArduino = %f, %f", dtROS_, dtArduino);
  ROS_INFO("Left Encoder ticks = %ld", EncoderTicksLeft);
  ROS_INFO("Right Encoder ticks = %ld", EncoderTicksRight);
  ROS_INFO("delta_DistanceMoved = %f", delta_DistanceMoved);
  ROS_INFO("delta_Yaw = %f", delta_Yaw);
  ROS_INFO("x = %f", x);
  ROS_INFO("y = %f", y);
  ROS_INFO("yaw = %f", yaw);
  ROS_INFO("yaw in degrees = %f", yaw * 57.3);
  //ROS_INFO("compassDegrees = %d", compassDegrees);
  ROS_INFO("currentVelocity = %f", currentVelocity_);
  ROS_INFO("vYaw = %f", vYaw);
  //ROS_INFO("velocity reported left wheel = %f", velocityLeft);
  //ROS_INFO("velocity reported right wheel = %f", velocityRight);
  */
}
                 

void sendOutNavData()
  {
    double delta_DistanceMoved = 0.0, delta_x = 0.0, delta_y = 0.0, delta_Yaw = 0.0, dtArduino;
    currentVelocity_ = 0.0;
    static int leftTotalTicks, rightTotalTicks;
    int leftDeltaTicks = 0, rightDeltaTicks = 0;
    static unsigned long previousArduinoDataCounter = 0;
    // if this is the first time odometry data has come in, then we have no previous time reference
    // and no initial number of odometer ticks, so we need to set those

   if (pauseState)
   {
      std_msgs::Int32 pauseAlert;
      pauseAlert.data = pauseState;
      pause_pub.publish(pauseAlert);
   }

   if (firstTime)
   {
      firstTime = false;
      last_time = ros::Time::now();
      previousEncoderTicksRight = EncoderTicksRight;
      previousEncoderTicksLeft = EncoderTicksLeft;
      leftTotalTicks = 0;
      rightTotalTicks = 0;
      getRadarData(destNode_[LEFT_RADAR_INDEX]);
      ROS_INFO("first nav data: EncoderTicksLeft = %ld, EncoderTicksRight = %ld", EncoderTicksLeft, EncoderTicksRight);
      return;
    } 

    // check for missed data:
    if (arduinoDataCounter != previousArduinoDataCounter + 1)
    {
      ROS_WARN("arduinoDataCounter not sequential, may have missed data, %ld, %ld",
          arduinoDataCounter, previousArduinoDataCounter);
    }
    previousArduinoDataCounter = arduinoDataCounter;

    //compute odometry
    // first calculate times
    dtArduino = ((double) arduinoCycleTime) / 1000.0;  // use the arduino delta time for velocity
                  // since that is the actual time between the odometry measurements.
                  //  The time to this line in robot pose varies by about +- 5 msec
    current_time = ros::Time::now();
    dtROS_ = current_time.toSec() - last_time.toSec();    
    if (dtROS_ > 0.05 || dtROS_ < 0.002)
    {
      //ROS_WARN("dtROS is very short or very long, dtROS in msec, arduino time, dataCounter = %f, %f, %ld",
      //     dtROS_, dtArduino, arduinoDataCounter);
      //double ct = current_time.toSec(), lt = last_time.toSec();
      //ROS_WARN("current time, last time = %f, %f", ct, lt);
    }
    last_time = current_time;

    // now calculate delta ticks since last cycle    
    if ( (EncoderTicksRight == 0 && (!(EncoderTicksLeft == 0)))
      || (EncoderTicksLeft == 0 && (!(EncoderTicksRight == 0))))
        ROS_WARN("One encoder is reading zero when the other is not, possible encoder failure, L,R ticks = %ld %ld",
                  EncoderTicksLeft, EncoderTicksRight);

    leftDeltaTicks = EncoderTicksLeft - previousEncoderTicksLeft;
    rightDeltaTicks = EncoderTicksRight - previousEncoderTicksRight;
    previousEncoderTicksRight = EncoderTicksRight;
    previousEncoderTicksLeft = EncoderTicksLeft;
    //ROS_INFO("left, right encoder ticks = %ld, %ld", EncoderTicksLeft, EncoderTicksRight);
    //ROS_INFO("left, right previous encoder ticks = %ld, %ld", previousEncoderTicksLeft, previousEncoderTicksRight);
    //ROS_INFO("left, right delta ticks = %d, %d", leftDeltaTicks, rightDeltaTicks);
    leftTotalTicks += leftDeltaTicks;
    rightTotalTicks += rightDeltaTicks;

    /*
    if (roger_) // this will leave delta_DistanceMoved unchanged if we have a burst of ticks, which happens sometimes in roger_
    {           // in that case, we will just use a distance equal to the previous distance
      if (abs(leftDeltaTicks) + abs(rightDeltaTicks) <= ROGER_MAX_TICKS_PER_CYCLE)
      {
        delta_DistanceMoved = ((double) (rightDeltaTicks + leftDeltaTicks)) / (2. * ticksPerMeter_); 
        smoothOdometry(&delta_DistanceMoved); // roger's course encoders are helped by some smoothing
      }
    }
    // the other robots do not need these checks and smoothing
    else
    */

    delta_DistanceMoved = ((double) (rightDeltaTicks + leftDeltaTicks)) / (2. * ticksPerMeter_); 
     

    currentVelocity_ = delta_DistanceMoved / dtArduino;   // in base_footprint frame, which is what the twist
                        // messages are supposed to be in.  Use dtArduino, so that the deltaV is based
                        // on the actual odometry measurement interval, not the time to this point in the program


    // now calculate delta yaw since last cycle
    if (abs(vYaw) < 0.02) vYaw = 0.0; // tiny gyro readings are almost always
               // just noise and sometimes the noise
               // is biased a little which leads to drift if you use it
    // if too much time passes between updates, we will get yaw movements
    // that are way off due to small residual gyro drifts so we toss out those
    if (dtArduino < 5) delta_Yaw = vYaw * dtArduino; 
    else delta_Yaw = 0;


    // my way ( a little different than turtlebot way, but gets the same result )
    /*
    yaw += delta_Yaw; // putting this here assumes we turn first and then move
    delta_x = delta_DistanceMoved * cos(yaw); 
    delta_y = delta_DistanceMoved * sin(yaw);
    x += delta_x;
    y += delta_y;

    */
    // turtlebot way is this:
    delta_x = delta_DistanceMoved * cos(delta_Yaw);
    delta_y = delta_DistanceMoved * (-sin(delta_Yaw));
    x += cos(yaw) * delta_x - sin(yaw) * delta_y; // using the yaw from the previous cycle
    y += sin(yaw) * delta_x + cos(yaw) * delta_y; // assumes 
    
  
    // since radar gives us an absolute distance from home, we can scale x and y to match this
    // radar data takes around a hundred milliseconds to return, so we have to only occasionally get the data
    
	  //cout << "radar counter,  mod 50 = " << radarCounter_ << ", " << radarCounter_ % 50 << endl;
	  if (radarCounter_ % 200 == 0)
     {
		 int radarIndex = LEFT_RADAR_INDEX;
		 radarCounter_ = 0;	// ************* remove this line when uncommentingto include right radar	 
		 /*if (radarCounter_ % 400 == 0)
		 {
		 	radarIndex = RIGHT_RADAR_INDEX;
		 	radarCounter_ = 0;
		 }
		 */
		 if (getRadarData(destNode_[radarIndex]))
		 {
		 	cout << "in robotPose, radar distance to home = " <<  distanceToHome_[radarIndex] << endl;
		 	/*if (distanceToHome_[radarIndex] > 0.001)
			{
				double odomDistanceToHome = sqrt((double) (((x - homeX_) * (x - homeX_)) + ((y - homeY_) * (y - homeY_))));
				cout << "in robotPose, odom distance to home = " <<  odomDistanceToHome << endl;
				double ratioHomeDistances;
				if (odomDistanceToHome > 0.01) 
				{
					ratioHomeDistances = ((double) distanceToHome_[radarIndex]) / odomDistanceToHome;
					x = ((x - homeX_) * ratioHomeDistances) + homeX_;
					y = ((y - homeY_) * ratioHomeDistances) + homeY_;
					if (ratioHomeDistances > 1.5 || ratioHomeDistances < 0.5) // let the user know we corrected by more than a bit
					{
						cout << "ratio of home distances = " <<  ratioHomeDistances << endl;
						cout << "distanceToHome = " <<  distanceToHome_[radarIndex] << endl;
						cout << "odomDistanceToHome = " <<  odomDistanceToHome << endl;
					 }
				  }
			   }
			else cout << "radar data not available for odom updating" << endl;
			*/
		 }
		 else cout << "radar call failed in robotPose, radar distance to Home = " << distanceToHome_[radarIndex] << endl;
	  }
	  radarCounter_++;

	  
    yaw += delta_Yaw;
    
    publishPose(x, y, yaw, vYaw, currentVelocity_);

  }
 

void parseNavData(std::string data)
{
  std::string navDataBuffer[1024];

  //ROS_INFO("parsing nav data");
  //ROS_INFO(data.c_str());
  std::size_t found = data.find_first_of(",");
  int numDataValues = 0;
  ros::Time static local_last_time;
  while (found!=std::string::npos)
  {
    navDataBuffer[numDataValues] = data.substr(0,found);
    numDataValues++;
    data = data.substr(found + 1, std::string::npos);
    found = data.find_first_of(",");
  }
  /*
  ROS_INFO("parsed nav data strings: ");
  for (int i = 0; i < numDataValues; i++)
  {
    ROS_INFO(navDataBuffer[i].c_str());
  }
  
  ROS_INFO("converted nav data to numbers: ");
  double navDataValues[256];
  for (int i = 0; i < numDataValues; i++)
  {
    navDataValues[i] = atof(navDataBuffer[i].c_str());
    ROS_INFO("%f", navDataValues[i]);
  }
  */
  vYaw = atof(navDataBuffer[0].c_str()) / 57.3; // convert from degrees/sec to rads/sec
  EncoderTicksRight = atof(navDataBuffer[1].c_str());
  EncoderTicksLeft = atof(navDataBuffer[2].c_str());
  EncoderPickerUpper = atof(navDataBuffer[3].c_str());
  EncoderBinShade = atof(navDataBuffer[4].c_str());
  EncoderDropBar = atof(navDataBuffer[5].c_str());
  //EncoderExtra = atof(navDataBuffer[6].c_str());
  accelX = atof(navDataBuffer[6].c_str());
  accelY = atof(navDataBuffer[7].c_str());
  accelZ = atof(navDataBuffer[8].c_str());
  battery = atof(navDataBuffer[9].c_str());
  pauseState = atof(navDataBuffer[10].c_str());
  dirAntMaxDir = atof(navDataBuffer[11].c_str());
  dirAntMaxTilt = atof(navDataBuffer[12].c_str());
  dirAntLevel = atof(navDataBuffer[13].c_str());
  arduinoCycleTime = atof(navDataBuffer[14].c_str());
  arduinoDataCounter = atof(navDataBuffer[15].c_str());
  // use this to check the timing of arduino data
  /*  ros::Time current_time = ros::Time::now();
    double dtROS_ = current_time.toSec() - local_last_time.toSec();
    double rosTransferTime = current_time.toSec() - atof(navDataBuffer[11].c_str());
    ros::Duration durROS = current_time - local_last_time;
    local_last_time = current_time;
    std_msgs::String dataTiming;
    std::stringstream sss;
    sss.str(" ");
    sss << navDataBuffer[10] << ", " << navDataBuffer[11] << ", " << navDataBuffer[12] << ", current time = " << current_time << ", " << dtROS_ << ", " << arduinoDataCounter << ", " << arduinoCycleTime << " transfer time = " << rosTransferTime;
    dataTiming.data =  sss.str();
    if (dtROS_ > 0.025 || dtROS_ < 0.005 ) sensors_pub.publish(dataTiming); 
   */ 

   sendOutNavData();
} 

//Process ROS message, detect if it is nav data, if so, publish transform and odom
void ucResponseCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string response = msg->data.c_str();
  //ROS_INFO("incoming message is");
  //ROS_INFO(response.c_str());
  //int responseLength = response.length();
  //int compareValue = response.compare(0,7,"navdata");
  std::string first7 = response.substr(0,7); 
  if (response.compare(0,7,"navdata") == 0)
  {
     //ROS_INFO("navdata received, length = %d", responseLength);
     parseNavData(response.substr(7,std::string::npos));	// send substring, removing "navdata" from the front    
  }
}


void assignedPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{ 
  tf::Pose assignedPose;
  tf::poseMsgToTF(msg->pose.pose, assignedPose);
  ROS_INFO("Pose prior to being set by user was the following: x = %f, y = %f, yaw = %f",x,y,yaw);

  geometry_msgs::Quaternion assignedAngle = msg->pose.pose.orientation;
  
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  yaw = tf::getYaw(assignedAngle);
  current_time = ros::Time::now();

  //publishPose(x, y, yaw, vYaw, currentVelocity_);
  ROS_INFO("Pose set by user as the following: x = %f, y = %f, yaw = %f",x, y, yaw);
  ROS_INFO("When pose set by user, EncoderTicksLeft = %ld, EncoderTicksRight = %ld, Battery = %d", EncoderTicksLeft, EncoderTicksRight, battery);

  // don't want to be constantly publishing battery value
  // seems reasonable to publish it when the user enters a new pose.
  char str[256];
  sprintf(str, "battery=%d", battery);
  std_msgs::String batteryStr;
  batteryStr.data = str;
  sensors_pub.publish(batteryStr);  
}

void poseCommandCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) 
{
   /*	these poses are from amcl, so should we update pose to agree?*******************************************************************************
   geometry_msgs::PoseWithCovariance poseWithCovariance = msg->pose;
   geometry_msgs::Pose pose = poseWithCovariance.pose;
   geometry_msgs::Point position = pose.position;
   geometry_msgs::Quaternion orientation = pose.orientation;
   x = position.x;
   y = position.y;
   z = orientation.z;
   w = orientation.w;
   
   //double varianceX = poseWithCovariance.covariance[0];      // x variance
   //double varianceY = poseWithCovariance.covariance[7];      // y variance
   //double varianceYaw = poseWithCovariance.covariance[35];   // Yaw variance
   //setVariances(varianceX, varianceY, varianceYaw);

   //ROS_INFO("pose received by patrol_node: ");
   //ROS_INFO("pose position x,y = %f, %f", x, y);
   //ROS_INFO("pose orientation w,z = %f, %f", w, z);
   //ROS_INFO("variance x,y,Yaw = %f, %f, %f", varianceX, varianceY, varianceYaw);
   */

  // don't want to be constantly publishing battery value
  // seems reasonable to publish it when amcl sends out a new pose.
  char str[256];
  sprintf(str, "battery=%d", battery);
  std_msgs::String batteryStr;
  batteryStr.data = str;
  sensors_pub.publish(batteryStr); 
}

bool setPose_service_send(outdoor_bot::setPose_service::Request  &req, outdoor_bot::setPose_service::Response &res)
{
   bool setHome = req.setHome;
   if (setHome)
   {
      homeX_ = req.x;
      homeY_ = req.y;
      homeYaw_ = req.yaw;
   }
   else
   {
      x = req.x;
      y = req.y;
      yaw = req.yaw;
   } 
   return true;
}

bool encoders_service_send(outdoor_bot::encoders_service::Request  &req, outdoor_bot::encoders_service::Response &res)
{
   res.encoderPickerUpper = EncoderPickerUpper;
   res.encoderDropBar = EncoderDropBar;
   res.encoderBinShade = EncoderBinShade;   
   return true;
}

bool dirAnt_service_send(outdoor_bot::dirAnt_service::Request  &req, outdoor_bot::dirAnt_service::Response &res)
{
   res.dirAntMaxDir = dirAntMaxDir;
   res.dirAntMaxTilt = dirAntMaxTilt;
   res.dirAntLevel = dirAntLevel;   
   return true;
}

bool radar_service_send(outdoor_bot::radar_service::Request  &req, outdoor_bot::radar_service::Response &res)
{
   int radarNumber = req.radarNumber;
   cout << " robotPose received radar service request for radar number = " << radarNumber << endl;
   
   if (radarNumber == LEFT_RADAR_NUMBER)
   {
   	res.distanceToHome = distanceToHome_[LEFT_RADAR_INDEX];
   	res.deltaDistanceToHome = deltaDistanceToHome_[LEFT_RADAR_INDEX];
		res.velocityFromHome = velocityFromHome_[LEFT_RADAR_INDEX]; 
		cout << "robotPose responded to radar service request with distanceToHome_ = " << distanceToHome_[LEFT_RADAR_INDEX] << endl;
	} 
	
	if (radarNumber == RIGHT_RADAR_NUMBER)
   {
   	res.distanceToHome = distanceToHome_[RIGHT_RADAR_INDEX];
   	res.deltaDistanceToHome = deltaDistanceToHome_[RIGHT_RADAR_INDEX];
		res.velocityFromHome = velocityFromHome_[RIGHT_RADAR_INDEX]; 
	} 
	
	if (radarNumber == CENTER_RADAR_NUMBER)
   {
   	res.distanceToHome = distanceToHome_[CENTER_RADAR_INDEX];
   	res.deltaDistanceToHome = deltaDistanceToHome_[CENTER_RADAR_INDEX];
		res.velocityFromHome = velocityFromHome_[CENTER_RADAR_INDEX]; 
	} 

   return true;
}

void moveCommandCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  // generate fake pose, for testing move commands
  if (firstTime)
  {
      firstTime = false;
      last_time = ros::Time::now();
      return;
  }
  double vx = cmd_vel->linear.x;	// in meters/sec
  double vy = cmd_vel->linear.y;  
  if (abs(vy) > 0.001) cout << "vy is not zero" << endl << endl;
  vYaw = cmd_vel->angular.z;	// in radians/sec
  current_time = ros::Time::now();
  double deltaSeconds = current_time.toSec() - last_time.toSec();
  last_time = ros::Time::now();
  if (deltaSeconds > 0.5)
  {
  	cout << "deltaSeconds exceeded max, = " << deltaSeconds << endl;
  	deltaSeconds = 0.5; // not a valid update time
  }
  double deltaX = vx * deltaSeconds;
  double deltaY = vy * deltaSeconds;
  yaw += vYaw * deltaSeconds;
  x += cos(yaw) * deltaX - sin(yaw) * deltaY; 
  y += sin(yaw) * deltaX + cos(yaw) * deltaY;  
  currentVelocity_ = sqrt((vx * vx) + (vy * vy));
  cout << "vx, vy, vYaw, dt = " << ", " << vx << ", " << vy << ", " << vYaw << ", " << deltaSeconds << endl;
  cout << "x, y, yaw = " << x << ", " << y << ", " << yaw << endl << endl;
}


int main(int argc, char** argv){
  //Initialize ROS
  ros::init(argc, argv, "robotPose");
  ros::NodeHandle n;
  ROS_INFO("robotPose starting");

// need to parameterize this********************************************************
  ticksPerMeter_ = OUTDOOR_TICKS_PER_METER;
  ROS_WARN("Assuming that this robot is outdoor_bot!");
  odom_broadcaster = new tf::TransformBroadcaster;  // send out transform

  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);  //advertise odometry
  imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 50);  //advertise imu
  sensors_pub = n.advertise<std_msgs::String>("sensor_readings", 50);
  pause_pub = n.advertise<std_msgs::Int32>("pause_state", 50);
  encoders_serv = n.advertiseService("encoders_service", encoders_service_send);
  radar_serv = n.advertiseService("radar_service", radar_service_send);
  dirAnt_serv = n.advertiseService("dirAnt_service", dirAnt_service_send);
  setPose_serv = n.advertiseService("setPose_service", setPose_service_send);
  
  // subscribe to move commands for testing move_base.  comment out
  // when running the real bot
  moveCmd = n.subscribe("cmd_vel", 50, moveCommandCallback);  

  //Subscribe to nav_data messages with arduino sensor data
  ucResponseMsg = n.subscribe("uc1Response", 100, ucResponseCallback);

  // Subscribe to pose settings from rviz (pose estimates entered by the user)
  assignedPose = n.subscribe("initialpose",10, assignedPoseCallback);

  // Subscribe to pose settings from amcl.  We'll use these as times to send out battery values.
  poseWithCovariance_ = n.subscribe("amcl_pose", 10, poseCommandCallback); // subscribe to amcl pose
  
  for (int i=0; i < NUM_RADARS; i++)
  {
  	 distanceToHome_[i] = 0.;
  	 previousDistanceToHome_[i] = 0.;
  	 deltaDistanceToHome_[i] = 0.;
	 velocityFromHome_[i] = 0.;
	}
	
	destNode_[LEFT_RADAR_INDEX] = LEFT_RADAR_NUMBER;
	destNode_[RIGHT_RADAR_INDEX] = RIGHT_RADAR_NUMBER;
	destNode_[CENTER_RADAR_INDEX] = CENTER_RADAR_NUMBER;
	

// this section is just for testing move_base
while(n.ok())
{
	ros::spinOnce();  // check for incoming messages
	struct timespec ts;
   ts.tv_sec = 0;
   ts.tv_nsec = 10000000;
   nanosleep(&ts, NULL); // update every 10 ms
   current_time = ros::Time::now();
   //getRadarData(destNode_[LEFT_RADAR_INDEX]);
   publishPose(x, y, yaw, vYaw, currentVelocity_);
}
// comment out the above section when running the real bot


ros::spin();
delete odom_broadcaster;
}
