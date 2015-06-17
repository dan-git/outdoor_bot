#include <ros/ros.h>
#include "outdoor_bot/radar_msg.h"
#include "rcmRadar/radar.h"
#include "outdoor_bot_defines.h"

//#define NUM_RADARS 4
#define LEFT_RADAR_NUMBER 101
#define RIGHT_RADAR_NUMBER 102
//#define CENTER_RADAR_NUMBER 100
#define LEFT_RADAR_INDEX 0
#define RIGHT_RADAR_INDEX 1
//#define CENTER_RADAR_INDEX 2
//#define LEFT_RADAR_HOME_DISTANCE 1.47	// when at home, distances in meters for radar rangers
//#define RIGHT_RADAR_HOME_DISTANCE 2.17

#define RADAR_WAIT_TIME 0.5
#define HOME_RADAR_SEPARATION 1900. // distance between radars on home platform, in mm
#define BOT_RADAR_SEPARATION 1092.

using namespace std;

class radar
{

private:

	radarRanger leftRanger_, rightRanger_; 
	double distanceFromLeftToLeft_, distanceFromLeftToRight_, distanceFromRightToLeft_, distanceFromRightToRight_;
	double distanceToHome_, angleToHome_, botOrientation_;
	double distanceToStagingPoint_, angleToStagingPoint_;

public:
	radar()
		: leftRanger_("/dev/ttyRadar_100"),
		  rightRanger_("/dev/ttyRadar_102"),
		  distanceFromLeftToLeft_(0), distanceFromLeftToRight_(0.), distanceFromRightToLeft_(0.), distanceFromRightToRight_(0),
		  distanceToHome_(0.),
		  angleToHome_(0.),
		  botOrientation_(0.)
	{
	}
	
bool getThreeRadarLocation()
{
	if (distanceFromLeftToLeft_ < 0.1 && distanceFromRightToLeft_ > 0.1)
	{
		distanceToHome_ = distanceFromRightToLeft_ / 1000.;
		//cout << "distance to home is from right side = " << distanceToHome_ << endl;
		return false;
	}
	else if ((distanceFromRightToLeft_ < 0.1 && distanceFromLeftToLeft_ > 0.1) 
	|| (distanceFromRightToLeft_ > distanceFromLeftToLeft_ + 5000))
	{
		distanceToHome_ = distanceFromLeftToLeft_ /1000.;
		//cout << "distance to home is from left side = " << distanceToHome_ << endl;
		return false;
	}
	else if (distanceFromRightToLeft_ > 0.1 && distanceFromLeftToLeft_ > 0.1) 
	
	{
		distanceToHome_ = (distanceFromLeftToLeft_ + distanceFromRightToLeft_) / 2000.;
		angleToHome_ =  -(asin( (distanceFromLeftToLeft_ - distanceFromRightToLeft_) / BOT_RADAR_SEPARATION)) * 57.3;
		return true;
		//cout << "distance to home is averaged value = " << distanceToHome_ << endl;
		//cout << "angle to home is from asin calculation = " << angleToHome_ << endl;
	}
	else
	{
		cout << " no radar data received in getThreeRadarLocation" << endl;
		return false;
	}
}
		

bool getLocation()
{
	if ( distanceFromLeftToLeft_ < 0.1 ||
		distanceFromLeftToRight_ < 0.1 ||
		distanceFromRightToLeft_ < 0.1 ||
		distanceFromRightToRight_ < 0.1) 
	{
			distanceToHome_ = 0.;
			angleToHome_ = 0.;
			cout << "unable to get some radar ranges, cannot calculate location" << endl;
			return false;
	}
		
	double cosLeftHomeToLeftBot =
	   ( (distanceFromLeftToLeft_ * distanceFromLeftToLeft_)
		+ (HOME_RADAR_SEPARATION * HOME_RADAR_SEPARATION)
		- (distanceFromLeftToRight_ * distanceFromLeftToRight_) )
		/  ( 2. * distanceFromLeftToLeft_ * HOME_RADAR_SEPARATION);
		
	double cosAngleB = 
		( (distanceFromLeftToRight_ * distanceFromLeftToRight_) 
		+ (BOT_RADAR_SEPARATION * BOT_RADAR_SEPARATION)
		- (distanceFromRightToRight_ * distanceFromRightToRight_) )
		/  ( 2. * distanceFromLeftToRight_ * BOT_RADAR_SEPARATION);
		
	double cosAngleD =
		( (distanceFromLeftToLeft_ * distanceFromLeftToLeft_) 
		+ (distanceFromLeftToRight_ * distanceFromLeftToRight_)
		- (HOME_RADAR_SEPARATION * HOME_RADAR_SEPARATION) )
		/  ( 2. * distanceFromLeftToLeft_ * distanceFromLeftToRight_);	
		
	botOrientation_ = -3.14 + acos(cosAngleB) + acos(cosAngleD) + acos(cosLeftHomeToLeftBot); 
	
	double distanceFromLeftToCenterSquared = 
		( (distanceFromLeftToLeft_ * distanceFromLeftToLeft_) 
		+ ((HOME_RADAR_SEPARATION * HOME_RADAR_SEPARATION)/4.) )
		- (distanceFromLeftToLeft_ * HOME_RADAR_SEPARATION * cosLeftHomeToLeftBot);
		
	double distanceFromLeftToCenter;
	
	if (distanceFromLeftToCenterSquared >= 0) distanceFromLeftToCenter = sqrt(distanceFromLeftToCenterSquared);
	else 
	{
		cout << "distanceFromLeftToCenterSquared was < 0 " << endl;
		return false;
	}
	
	double cosAngleE = 
		( (distanceFromLeftToLeft_ * distanceFromLeftToLeft_) 
		+ (distanceFromLeftToCenter * distanceFromLeftToCenter)
		- ((HOME_RADAR_SEPARATION * HOME_RADAR_SEPARATION)/4.) )
		/ ( 2. * distanceFromLeftToLeft_ * distanceFromLeftToCenter);
	
	double anglef = acos(cosLeftHomeToLeftBot) - botOrientation_;
	double angleh = 3.14 - (acos(cosAngleE) + anglef);
	
	double distanceFromCenterToCenterSquared = 
		((BOT_RADAR_SEPARATION * BOT_RADAR_SEPARATION)/4.)
		+ (distanceFromLeftToCenter * distanceFromLeftToCenter)
		- (BOT_RADAR_SEPARATION * distanceFromLeftToCenter * cos(angleh));

	double distanceFromCenterToCenter;
	if (distanceFromCenterToCenterSquared >= 0) distanceFromCenterToCenter = sqrt(distanceFromCenterToCenterSquared);
	else 
	{
		cout << "distanceFromRightToCenterSquared was < 0 " << endl;
		return false;
	}		
	
	double cosAngleI = 
		( (distanceFromCenterToCenter * distanceFromCenterToCenter)
		+ ((BOT_RADAR_SEPARATION * BOT_RADAR_SEPARATION)/4.) 
		- distanceFromLeftToCenterSquared ) 
		/ (distanceFromCenterToCenter * BOT_RADAR_SEPARATION);
		
	angleToHome_ = (1.57 - acos(cosAngleI)) * 57.3;	// convert to degrees
	
	// convert to meters
	if  (distanceFromCenterToCenterSquared >= 0) distanceToHome_ = sqrt(distanceFromCenterToCenterSquared);
	else 
	{
		cout << "distanceFromCenterToCenterSquared was negative, = " << distanceFromCenterToCenterSquared << endl;
		return false;
	}
	
	double angleN = 3.14 - (acos(cosAngleI) + botOrientation_);
	double angleO = angleN - 1.57;
	double distanceToStagingPointSquared = 
		(RADAR_STAGING_POINT_DISTANCE * RADAR_STAGING_POINT_DISTANCE * 1000000.)
		+ distanceFromCenterToCenterSquared
		- (2.* cos(angleO) * RADAR_STAGING_POINT_DISTANCE * 1000. * distanceToHome_);
		
	double cosAngleToStagingPoint;
	if (distanceToStagingPointSquared > 0.1)
	{
		distanceToStagingPoint_ = sqrt(distanceToStagingPointSquared);
		
		cosAngleToStagingPoint = 
		( (distanceToStagingPointSquared + distanceFromCenterToCenterSquared )
		- (RADAR_STAGING_POINT_DISTANCE * RADAR_STAGING_POINT_DISTANCE * 1000000.) )
		/ (2. * distanceToStagingPoint_ * distanceToHome_);
		
		angleToStagingPoint_ = (acos(cosAngleToStagingPoint) * 57.3) + angleToHome_;	// convert to degrees
	}
	else cout << "distanceToStagingPointSquared is negative, = " << distanceToStagingPointSquared << endl;
	
	distanceToHome_ /= 1000.; // convert to meters
	distanceToStagingPoint_ /= 1000.;
	
	/*
	cout << "cosLeftHomeToLeftBot (angle a) = " << cosLeftHomeToLeftBot << ", an angle of " << acos(cosLeftHomeToLeftBot) * 57.3 << " degrees" << endl;
	cout << "cosAngleB = " << cosAngleB << ", an angle of " << acos(cosAngleB) * 57.3 << " degrees" << endl;
	cout << "cosAngleD = " << cosAngleD << ", an angle of " << acos(cosAngleD) * 57.3 << " degrees" << endl;
	cout << "botOrientation = " << botOrientation_ * 57.3 << " degrees " << endl;
	cout << "distanceFromLeftToCenterSquared (distance b squared) = " << distanceFromLeftToCenterSquared << endl;
	cout << "distanceFromLeftToCenter (distance b) = " << distanceFromLeftToCenter << endl;
	cout << "cosAngleE = " << cosAngleE << ", an angle of " << acos(cosAngleE) * 57.3 << " degrees" << endl;
	cout << "anglef = " << anglef * 57.3 << endl;
	cout << "angleh = " << angleh * 57.3 << endl;
	cout << "distanceFromCenterToCenterSquared (distance s squared) = " << distanceFromCenterToCenterSquared << endl;	
	if (distanceFromCenterToCenterSquared >= 0)
		cout << "distanceFromCenterToCenter (distance s) = " << sqrt(distanceFromCenterToCenterSquared) << endl;
	else cout << "distanceFromCenterToCenterSquared is a negative number " << endl;
	cout << "cosAngleI = " << cosAngleI << ", an angle of " << acos(cosAngleI) * 57.3 << " degrees " << endl;
	cout << "angleToHome_ (angle j) = " << angleToHome_ << endl;
	cout << "angleN = " << angleN * 57.3 << endl;
	cout << "angleO = " << angleO * 57.3 << endl << endl;
	cout << "cosAngleM = " << cosAngleToStagingPoint << ", an angle of " << acos(cosAngleToStagingPoint) * 57.3 << endl;
	cout << "angle to staging point (angles m + j)  = " << angleToStagingPoint_ << " degrees" << endl;
	cout << "distanceToStagingPoint (distance t) = " << distanceToStagingPoint_ << " meters" << endl;
	*/	
	cout << "Home is " << distanceToHome_ << " meters away at an angle = " << angleToHome_ << " degrees" << endl;
	cout << "Bot's orientation (angle c) with respect to the platform = " << botOrientation_ * 57.3 << " degrees " << endl;
	cout << "Staging point is " << distanceToStagingPoint_ << " meters away at an angle = " <<
		 angleToStagingPoint_  << " degrees" << endl << endl << endl;
	return true;
}
	

double getDistanceFromLeftToLeft() { return distanceFromLeftToLeft_; }
double getDistanceFromLeftToRight() { return distanceFromLeftToRight_; }
double getDistanceFromRightToLeft() { return distanceFromRightToLeft_; }
double getDistanceFromRightToRight() { return distanceFromRightToRight_; }
double getDistanceToHome() { return distanceToHome_; }
double getAngleToHome() { return angleToHome_; }
double getOrientation() { return botOrientation_ * 57.3; }
double getDistanceToStagingPoint() { return distanceToStagingPoint_; }
double getAngleToStagingPoint() { return angleToStagingPoint_; }


void getRadarRanges()
{
	ros::Duration duration(RADAR_WAIT_TIME);
   
	distanceFromLeftToRight_ = leftRanger_.getRange(RIGHT_RADAR_NUMBER);
	duration.sleep();
	
	distanceFromRightToLeft_ = rightRanger_.getRange(LEFT_RADAR_NUMBER);
	//ROS_INFO("got range from right to left = %f", distanceFromRightToLeft_);
	duration.sleep();
	
	distanceFromRightToRight_ = rightRanger_.getRange(RIGHT_RADAR_NUMBER);
	duration.sleep();
	
   distanceFromLeftToLeft_ = leftRanger_.getRange(LEFT_RADAR_NUMBER);
   //ROS_INFO("got range from left to left = %f", distanceFromLeftToLeft_);
   duration.sleep();
   
}

/*

bool radar_service_send(outdoor_bot::radar_service::Request  &req, outdoor_bot::radar_service::Response &res)
{
	radarDataEnabled_ = req.enableRadarData;
	cout << " robotPose received radar service request " << endl;
	res.distanceToHomeLeft = distanceToHome_[LEFT_RADAR_INDEX];
	//res.deltaDistanceToHome = deltaDistanceToHome_[LEFT_RADAR_INDEX];
	//res.velocityFromHome = velocityFromHome_[LEFT_RADAR_INDEX]; 
	cout << "robotPose responded to radar service request with left distanceToHome_ = " << distanceToHome_[LEFT_RADAR_INDEX] << endl;
	
	res.distanceToHomeRight = distanceToHome_[RIGHT_RADAR_INDEX];
	//res.deltaDistanceToHome = deltaDistanceToHome_[RIGHT_RADAR_INDEX];
	//res.velocityFromHome = velocityFromHome_[RIGHT_RADAR_INDEX];
	cout << "robotPose responded to radar service request with right distanceToHome_ = " << distanceToHome_[RIGHT_RADAR_INDEX] << endl; 

	res.distanceToHomeCenter = distanceToHome_[CENTER_RADAR_INDEX];
	//res.deltaDistanceToHome = deltaDistanceToHome_[CENTER_RADAR_INDEX];
	//res.velocityFromHome = velocityFromHome_[CENTER_RADAR_INDEX]; 
	cout << "robotPose responded to radar service request with center distanceToHome_ = " << distanceToHome_[CENTER_RADAR_INDEX] << endl;

   return true;
}

bool radar_service_send(outdoor_bot::radar_service::Request  &req, outdoor_bot::radar_service::Response &res)
{
   int radarNumber = req.radarNumber;
   cout << " robotPose received radar service request for radar number " << radarNumber << endl;
   
   if (radarNumber == LEFT_RADAR_NUMBER)
   {
   	res.distanceToHome = distanceToHome_[LEFT_RADAR_INDEX];
   	res.deltaDistanceToHome = deltaDistanceToHome_[LEFT_RADAR_INDEX];
		res.velocityFromHome = velocityFromHome_[LEFT_RADAR_INDEX]; 
		cout << "robotPose responded to radar service request with left distanceToHome_ = " << distanceToHome_[LEFT_RADAR_INDEX] << endl;
	} 
	
	if (radarNumber == RIGHT_RADAR_NUMBER)
   {
   	res.distanceToHome = distanceToHome_[RIGHT_RADAR_INDEX];
   	res.deltaDistanceToHome = deltaDistanceToHome_[RIGHT_RADAR_INDEX];
		res.velocityFromHome = velocityFromHome_[RIGHT_RADAR_INDEX];
		cout << "robotPose responded to radar service request with right distanceToHome_ = " << distanceToHome_[RIGHT_RADAR_INDEX] << endl; 
	} 
	
	if (radarNumber == CENTER_RADAR_NUMBER)
   {
   	res.distanceToHome = distanceToHome_[CENTER_RADAR_INDEX];
   	res.deltaDistanceToHome = deltaDistanceToHome_[CENTER_RADAR_INDEX];
		res.velocityFromHome = velocityFromHome_[CENTER_RADAR_INDEX]; 
		cout << "robotPose responded to radar service request with center distanceToHome_ = " << distanceToHome_[CENTER_RADAR_INDEX] << endl;
	} 

   return true;
}
*/

void testDataRadarRanges()
{  
	ros::Time last_time;
   ros::Time current_time = ros::Time::now();
	double leftHomeRadarLocation_X =  0., leftHomeRadarLocation_Y =  -HOME_RADAR_SEPARATION / 2.;
	double rightHomeRadarLocation_X =  0., rightHomeRadarLocation_Y = leftHomeRadarLocation_Y + HOME_RADAR_SEPARATION;
	double leftBotRadarLocation_X, leftBotRadarLocation_Y, rightBotRadarLocation_X, rightBotRadarLocation_Y;
	
	// calculated inputs
	// convention is that x axis is perpendicular platform, Y axis is parallel to platform, consistent with ROS directions (X to move forward)
	// ccw angles are positive, the angle being formed by the location of the platform as seen by the robot, so that
	// the eventual angular output is the angle the robot has to turn to face the platform.
	
	double botDistance, botAngle, botOrientation;	
	
	// 20 meters away, centered, facing
		//botDistance = 20000.;
	
	
	// these come out correct
	
		//botAngle = 0.;
		//botOrientation = 0.;
	
		//botAngle = 90.;
		//botOrientation = 0.;

		//botAngle = 0.;
		//botOrientation = 90.;
		
		//botAngle = 45.;
		//botOrientation = 0.;
		
		//botAngle = 0.;
		//botOrientation = 45.;
		
		//botAngle = -90.;
		//botOrientation = 0.;
		
		// this one has some round off error, about 3 degrees off
		//botAngle = 0.;
		//botOrientation = -90.;
		
		//botAngle = 0.;
		//botOrientation = -45.;
		
		//botAngle = -10.;
		//botOrientation = -45.;
		botDistance = 10000.;
		botAngle = 20.;
		botOrientation = 0;
		
	// we will try to sim some of the real-life failures
	// from about 4 meters, angle of -10 degrees, orientation of about -45 degrees
		//botDistance = 3759.;
		//botAngle = -20.;
		//botOrientation = -30.;
		
		// from about 2 meters, angle of -40 degrees, orientation of about -10 degrees
		//botDistance = 1800.;
		//botAngle = -40.;
		//botOrientation = -10;
		//results:
		//distanceToHome, angleToHome, Orientation = 1.8 meters, -50.0456 degrees, -9.90874 degrees, with respect to the platform
		//distanceToStagingPoint, angleToStagingPoint = 3.80503 meters, 72.076 degrees
		//runningAverageDistanceToHome, runningAverageAngleToHome = 1.8 meters, -50.0456 degrees
		//ranges from Bot Left to home left, right = 1651.19, 3027.56
		//ranges from Bot Right to home left, right = 1326.05, 2027.69

		//
		// ranges recorded on the bot:
		// ranges from Bot Left to home left, right = 1610, 2811
		// ranges from Bot Right to home left, right = 1339, 2065
		// distanceToHome, angleToHome, Orientation = 1.80594 meters, -23.5007 degrees, 4.82885 degrees, with respect to the platform
		// distanceToStagingPoint, angleToStagingPoint = 3.51634 meters, 114.077 degrees
		
		// from about 2 meters, angle of -23 degrees, orientation of about 4 degrees (which is what the code reported)
		//botDistance = 1800.;
		//botAngle = -23.;
		//botOrientation = 4.8;
		//results:
		//distanceToHome, angleToHome, Orientation = 1.8 meters, -18.2456 degrees, 4.89126 degrees, with respect to the platform
		//distanceToStagingPoint, angleToStagingPoint = 3.41871 meters, 126.691 degrees
		//runningAverageDistanceToHome, runningAverageAngleToHome = 1.8 meters, -18.2456 degrees
		//ranges from Bot Left to home left, right = 1638.45, 2724.79
		//ranges from Bot Right to home left, right = 1877.31, 2032.04
		

		
	
	double angularPart_Y = cos(botOrientation / 57.3) * (BOT_RADAR_SEPARATION / 2.);
	double angularPart_X = sin(botOrientation / 57.3) * (BOT_RADAR_SEPARATION / 2.);
	
	leftBotRadarLocation_X = (botDistance * cos(botAngle / 57.3)) - angularPart_X;
	rightBotRadarLocation_X = (botDistance * cos(botAngle / 57.3)) + angularPart_X;
	
	leftBotRadarLocation_Y = (botDistance * sin(botAngle / 57.3)) - angularPart_Y;
	rightBotRadarLocation_Y = (botDistance * sin(botAngle / 57.3)) + angularPart_Y;
	
	//cout << "bot distance, angle, orientation = " << botDistance << ", " << botAngle << ", " << botOrientation << endl;

	/*
	// direct inputs
	
	// 20 meters away, centered, facing
	leftBotRadarLocation_X =  20000.;
	leftBotRadarLocation_Y =  (HOME_RADAR_SEPARATION - BOT_RADAR_SEPARATION) / 2.;
	rightBotRadarLocation_X =  20000.;
	rightBotRadarLocation_Y =  leftBotRadarLocation_Y + BOT_RADAR_SEPARATION;
	
	// 10 meters away, shifted the bot 10 meters left, facing
	leftBotRadarLocation_X =  10000.;
	leftBotRadarLocation_Y =  ((HOME_RADAR_SEPARATION - BOT_RADAR_SEPARATION) / 2.) - 10000.;
	rightBotRadarLocation_X = 10000.;
	rightBotRadarLocation_Y =  leftBotRadarLocation_Y + BOT_RADAR_SEPARATION;
	*/

	distanceFromLeftToLeft_ = 
		sqrt((leftBotRadarLocation_X - leftHomeRadarLocation_X) * (leftBotRadarLocation_X - leftHomeRadarLocation_X)
		+ (leftBotRadarLocation_Y - leftHomeRadarLocation_Y) * (leftBotRadarLocation_Y - leftHomeRadarLocation_Y));
	
	distanceFromLeftToRight_ = 
		sqrt((leftBotRadarLocation_X - rightHomeRadarLocation_X) * (leftBotRadarLocation_X - rightHomeRadarLocation_X)
		+ (leftBotRadarLocation_Y - rightHomeRadarLocation_Y) * (leftBotRadarLocation_Y - rightHomeRadarLocation_Y));
	
	distanceFromRightToLeft_ = 
		sqrt((rightBotRadarLocation_X - leftHomeRadarLocation_X) * (rightBotRadarLocation_X - leftHomeRadarLocation_X)
		+ (rightBotRadarLocation_Y - leftHomeRadarLocation_Y) * (rightBotRadarLocation_Y - leftHomeRadarLocation_Y));
	 
	distanceFromRightToRight_ = 
		sqrt((rightBotRadarLocation_X - rightHomeRadarLocation_X) * (rightBotRadarLocation_X - rightHomeRadarLocation_X)
		+ (rightBotRadarLocation_Y - rightHomeRadarLocation_Y) * (rightBotRadarLocation_Y - rightHomeRadarLocation_Y));  

	/*
	cout << "leftBotRadarLocation_X, leftBotRadarLocation_Y = " << leftBotRadarLocation_X << ", " << leftBotRadarLocation_Y << endl;
	cout << "rightBotRadarLocation_X, rightBotRadarLocation_Y = " << rightBotRadarLocation_X << ", " << rightBotRadarLocation_Y << endl;		
	cout << "distanceFromLeftToLeft_, distanceFromLeftToRight_ = " << distanceFromLeftToLeft_ << ", " << distanceFromLeftToRight_ << endl;
	cout << "distanceFromRightToLeft_, distanceFromRightToRight_ = " << distanceFromRightToLeft_ << ", " << distanceFromRightToRight_ << endl;	
	*/
	// inputs from data
	// robot distance, angle, orientation were approx 12m, -20, 0
	//distanceFromLeftToLeft_ = 12391.;
	//distanceFromLeftToRight_ = 11849.;
	//distanceFromRightToLeft_ = 11813.;
	//distanceFromRightToRight_ = 11378.;
	//distanceToHome, angleToHome, Orientation = 11.8781 meters, -21.1842 degrees, -40.0866 degrees, with respect to the platform
	//distanceToStagingPoint, angleToStagingPoint = 7.32894 meters, -8.41565 degrees


	// from about 4 meters, angle of -10 degrees, orientation of about -45 degrees
		
	//distanceFromLeftToLeft_ = 3647.;
	//distanceFromLeftToRight_ = 3381.;
	//distanceFromRightToLeft_ = 4410.;
	//distanceFromRightToRight_ = 3872.;	
	// results:
	// distanceToHome, angleToHome, Orientation = 3.75909 meters, 29.945 degrees, 7.66036 degrees, with respect to the platform
	// distanceToStagingPoint, angleToStagingPoint = 2.08496 meters, 144.541 degrees

	// from about 2 meters, turning the wrong way.  There is a video of this one
	//distanceFromLeftToLeft_ = 1900.;
	//distanceFromLeftToRight_ = 1460.;
	//distanceFromRightToLeft_ = 2690.;
	//distanceFromRightToRight_ = 1620.;	
	// results:
	//distanceToHome, angleToHome, Orientation = 1.7142 meters, 42.6093 degrees, 10.0115 degrees, with respect to the platform
	//distanceToStagingPoint, angleToStagingPoint = 3.67375 meters, 175.466 degrees


	
	last_time = ros::Time::now();
	while ( current_time.toSec() - last_time.toSec() < RADAR_WAIT_TIME) current_time = ros::Time::now();
	
}
	
};

int main(int argc, char** argv)
{
   //init the ROS node
   ros::init(argc, argv, "radar_node");
   ros::NodeHandle nh;

	ros::Publisher radar_pub_ = nh.advertise<outdoor_bot::radar_msg>("radar", 5);
   //ros::ServiceServer radar_serv = n.advertiseService("radar_service", radar_service_send);	
	outdoor_bot::radar_msg radarData;
	
	radar myRadar;

   double runningAverageDistanceToHome = 0., runningAverageAngleToHome = 0.;
   bool firstData = true;   
      
	while(nh.ok())
	{
		myRadar.getRadarRanges();  
		//myRadar.testDataRadarRanges();
		
		if (myRadar.getDistanceFromLeftToRight() < 0.05 
			&& myRadar.getDistanceFromRightToRight() < 0.05
			&& myRadar.getDistanceFromLeftToLeft() < 0.05
			&& myRadar.getDistanceFromRightToRight() < 0.05 )
		{	
			cout << "all radar data is missing" << endl;
			radarData.goodData = false;
			radarData.goodLocation = false;
			radarData.goodOrientation = false;
			radar_pub_.publish(radarData);
		}
		else
		{
   						
			double left_to_left = myRadar.getDistanceFromLeftToLeft();
			double left_to_right = myRadar.getDistanceFromLeftToRight();
		
			double right_to_left = myRadar.getDistanceFromRightToLeft();
			double right_to_right = myRadar.getDistanceFromRightToRight();
		
		
			bool left_bot_radar_good = (left_to_left > 0.05 || left_to_right > 0.05);
			bool right_bot_radar_good = (right_to_left > 0.05 || right_to_right > 0.05);
			bool left_base_radar_good = (left_to_left > 0.05 || right_to_left > 0.05);
			bool right_base_radar_good = (left_to_right > 0.05 || right_to_right > 0.05);
		
			if (!left_bot_radar_good)
			{
				left_to_left = right_to_left;
				left_to_right = right_to_right;
			}
			if (!right_bot_radar_good)
			{
				right_to_left = left_to_left;
				right_to_right = left_to_right;
			}
			if (!left_base_radar_good)
			{
				left_to_left = left_to_right;
				right_to_left = right_to_right;
			}
			if (!right_base_radar_good)
			{
				left_to_right = left_to_left;
				right_to_right = right_to_left;
			}

			double maxDistance1 = fmax(left_to_left, left_to_right);
			double maxDistance2 = fmax(right_to_left, right_to_right);
			double minDistance1 = fmin(left_to_left, left_to_right);
			double minDistance2 = fmin(right_to_left, right_to_right);
		
			radarData.maxDistanceToHome = fmax(maxDistance1, maxDistance2) / 1000.;
			radarData.minDistanceToHome = fmin(minDistance1, minDistance2) / 1000.;
		
			radarData.distanceToHome = myRadar.getDistanceToHome();	// range in meters
			
			if (myRadar.getLocation())
			{
				radarData.goodData = true;
				radarData.goodLocation = true;
				radarData.goodOrientation = true;
				radarData.angleToHome = myRadar.getAngleToHome();
				radarData.orientation = myRadar.getOrientation();     // bot orientation with respect to the platform (in degrees)
				radarData.distanceToStagingPoint = myRadar.getDistanceToStagingPoint();	// range in meters
				radarData.angleToStagingPoint = myRadar.getAngleToStagingPoint();			// azimuth in degrees
				cout << "all radars are reporting data" << endl;
				cout << "Four radar distance, angle, orientation to home =  " << radarData.distanceToHome << " meters, " << radarData.angleToHome << " degrees, " << radarData.orientation << endl;
				cout << "Four radar distance, angle to staging point = " << radarData.distanceToStagingPoint << " meters, " << radarData.angleToStagingPoint << " degrees" << endl << endl;	
			}
			else if (myRadar.getThreeRadarLocation())
			{			
				radarData.goodData = true;
				radarData.goodLocation = true;
				radarData.goodOrientation = false;
				radarData.angleToHome = myRadar.getAngleToHome();
				cout << "Three radar distance, angle to home =  " << radarData.distanceToHome << " meters, " << radarData.angleToHome << " degrees" << endl << endl;
			}
			else
			{
				radarData.goodData = true;
				radarData.goodLocation = false;
				radarData.goodOrientation = false;;
				cout << "Two radar distance to home =  " << radarData.distanceToHome << " meters, " << endl << endl;
			}
					
			if (!firstData)
			{
				runningAverageDistanceToHome += (radarData.distanceToHome - runningAverageDistanceToHome) * 0.1;
				runningAverageAngleToHome += (radarData.angleToHome - runningAverageAngleToHome)  * 0.1;
			}
			else
			{
				runningAverageDistanceToHome = radarData.distanceToHome;
				runningAverageAngleToHome = radarData.angleToHome;
				firstData = false;
			}
		
			radarData.runningAverageDistanceToHome = runningAverageDistanceToHome;	
			radarData.runningAverageAngleToHome = runningAverageAngleToHome;
				

			radar_pub_.publish(radarData);
		}
		
		/*
		if (radarData.goodData) cout << "GoodData is true, ";
		else cout << "goodData is false, ";
		if (radarData.goodLocation) cout << "goodLocation is true" << endl;
		else cout << "goodLocation is false" << endl;
		cout << "distanceToHome, angleToHome, Orientation = " << myRadar.getDistanceToHome() << " meters, "
			<< myRadar.getAngleToHome() << " degrees, " << myRadar.getOrientation() << " degrees, with respect to the platform" << endl;
		cout << "distanceToStagingPoint, angleToStagingPoint = " << myRadar.getDistanceToStagingPoint() << " meters, " << myRadar.getAngleToStagingPoint() << " degrees" << endl;
		cout << "runningAverageDistanceToHome, runningAverageAngleToHome = " << runningAverageDistanceToHome << " meters, " << runningAverageAngleToHome << " degrees" << endl;
		*/
		//cout << "ranges from Bot Left to home left, right = " << myRadar.getDistanceFromLeftToLeft() << ", " << myRadar.getDistanceFromLeftToRight() << std::endl;
		//cout << "ranges from Bot Right to home left, right = " << myRadar.getDistanceFromRightToLeft() << ", " << myRadar.getDistanceFromRightToRight() << std::endl;
		
		
	   
	}
	return EXIT_SUCCESS;
}
