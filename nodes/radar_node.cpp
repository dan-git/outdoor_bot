#include <ros/ros.h>
#include "outdoor_bot/radar_msg.h"
#include "rcmRadar/radar.h"
#include "outdoor_bot_defines.h"

#define RADAR_WAIT_TIME 0.2
#define HOME_RADAR_SEPARATION 1900. // distance between radars on home platform, in mm
#define BOT_RADAR_SEPARATION 1900. // 1092. //*****************************************change for real ops, this is just to do some sim tests********************************

#define STAGING_POINT_DISTANCE 5000. // in mm
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
		: leftRanger_("/dev/ttyRadar_103"),
		  rightRanger_("/dev/ttyRadar_100"),
		  distanceFromLeftToLeft_(0), distanceFromLeftToRight_(0.), distanceFromRightToLeft_(0.), distanceFromRightToRight_(0),
		  distanceToHome_(0.),
		  angleToHome_(0.),
		  botOrientation_(0.)
	{
	}

void getLocation()
{
	if ( distanceFromLeftToLeft_ < 0.1 ||
		distanceFromLeftToRight_ < 0.1 ||
		distanceFromRightToLeft_ < 0.1 ||
		distanceFromRightToRight_ < 0.1) 
	{
			distanceToHome_ = 0.;
			angleToHome_ = 0.;
			cout << "unable to get some radar ranges, cannot calculate location" << endl;
			return;
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
		return;
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
		return;
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
		return;
	}
	
	double angleN = 3.14 - (acos(cosAngleI) + botOrientation_);
	double angleO = angleN - 1.57;
	double distanceToStagingPointSquared = 
		(STAGING_POINT_DISTANCE * STAGING_POINT_DISTANCE)
		+ distanceFromCenterToCenterSquared
		- (2.* cos(angleO) * STAGING_POINT_DISTANCE * distanceToHome_);
		
	double cosAngleToStagingPoint;
	if (distanceToStagingPointSquared > 0.1)
	{
		distanceToStagingPoint_ = sqrt(distanceToStagingPointSquared);
		
		cosAngleToStagingPoint = 
		( (distanceToStagingPointSquared + distanceFromCenterToCenterSquared )
		- (STAGING_POINT_DISTANCE * STAGING_POINT_DISTANCE) )
		/ (2. * distanceToStagingPoint_ * distanceToHome_);
		
		angleToStagingPoint_ = (acos(cosAngleToStagingPoint) * 57.3) + angleToHome_;	// convert to degrees
	}
	else cout << "distanceToStagingPointSquared is negative, = " << distanceToStagingPointSquared << endl;
	
	distanceToHome_ /= 1000.; // convert to meters
	distanceToStagingPoint_ /= 1000.;
	

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
		
	cout << "Home is " << distanceToHome_ << " meters away at an angle = " << angleToHome_ << " degrees" << endl;
	cout << "Bot's orientation with respect to the platform = " << botOrientation_ * 57.3 << " degrees " << endl;
	cout << "Staging point is " << distanceToStagingPoint_ << " meters away at an angle = " <<
		 angleToStagingPoint_  << " degrees" << endl << endl << endl;

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
	ros::Time last_time;
   ros::Time current_time = ros::Time::now();
   
   distanceFromLeftToLeft_ = leftRanger_.getRange(LEFT_RADAR_NUMBER);
	last_time = ros::Time::now();
	while ( current_time.toSec() - last_time.toSec() < RADAR_WAIT_TIME) current_time = ros::Time::now(); // delay needed for radars to get data

	distanceFromLeftToRight_ = leftRanger_.getRange(RIGHT_RADAR_NUMBER);
	last_time = ros::Time::now();
	while ( current_time.toSec() - last_time.toSec() < RADAR_WAIT_TIME) current_time = ros::Time::now(); 
	
	distanceFromRightToLeft_ = rightRanger_.getRange(LEFT_RADAR_NUMBER);
	last_time = ros::Time::now();
	while ( current_time.toSec() - last_time.toSec() < RADAR_WAIT_TIME) current_time = ros::Time::now(); 
	
	distanceFromRightToRight_ = rightRanger_.getRange(RIGHT_RADAR_NUMBER);
	last_time = ros::Time::now();
	while ( current_time.toSec() - last_time.toSec() < RADAR_WAIT_TIME) current_time = ros::Time::now();
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
   
   // radar data sim
	double leftHomeRadarLocation_X =  0., leftHomeRadarLocation_Y =  0., rightHomeRadarLocation_X =  0., rightHomeRadarLocation_Y = leftHomeRadarLocation_Y + HOME_RADAR_SEPARATION;

	// 10 meters away, centered, facing
	double leftBotRadarLocation_X =  20000.;
	double leftBotRadarLocation_Y =  (HOME_RADAR_SEPARATION - BOT_RADAR_SEPARATION) / 2.;
	double rightBotRadarLocation_X =  20000.;
	double rightBotRadarLocation_Y =  leftBotRadarLocation_Y + BOT_RADAR_SEPARATION;

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
		
	cout << "distanceFromLeftToLeft_, distanceFromLeftToRight_ = " << distanceFromLeftToLeft_ << ", " << distanceFromLeftToRight_ << endl;
	cout << "distanceFromRightToLeft_, distanceFromRightToRight_ = " << distanceFromRightToLeft_ << ", " << distanceFromRightToRight_ << endl;	
	
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

   double runningAverageDistanceToHome = 0.;
   double runningAverageAngleToHome = 0.;
   bool firstData = true;
   
   radar myRadar;
   
	while(nh.ok())
	{
		//myRadar.getRadarRanges();  
		myRadar.testDataRadarRanges();
		
		double maxDistance1 = fmax(myRadar.getDistanceFromLeftToLeft(), myRadar.getDistanceFromLeftToRight());
		double maxDistance2 = fmax(myRadar.getDistanceFromRightToLeft(), myRadar.getDistanceFromRightToRight());
		double minDistance1 = fmin(myRadar.getDistanceFromLeftToLeft(), myRadar.getDistanceFromLeftToRight());
		double minDistance2 = fmin(myRadar.getDistanceFromRightToLeft(), myRadar.getDistanceFromRightToRight());
		radarData.maxDistanceToHome = fmax(maxDistance1, maxDistance2) / 1000.;
		radarData.minDistanceToHome = fmin(minDistance1, minDistance2) / 1000.;

		if (myRadar.getDistanceFromLeftToRight() < 0.05 
			&& myRadar.getDistanceFromRightToRight() < 0.05
			&& myRadar.getDistanceFromLeftToRight() < 0.05
			&& myRadar.getDistanceFromRightToRight() < 0.05 )
		{	
			cout << "all radar data is missing" << endl;
			radarData.goodData = false;
		}
		else radarData.goodData = true;	
				
		if (myRadar.getDistanceFromLeftToRight() > 0.05 
			&& myRadar.getDistanceFromRightToRight() > 0.05
			&& myRadar.getDistanceFromLeftToRight() > 0.05
			&& myRadar.getDistanceFromRightToRight() > 0.05 )
		{		
			myRadar.getLocation();
			radarData.distanceToHome = myRadar.getDistanceToHome();	// range in meters
			radarData.angleToHome = myRadar.getAngleToHome();			// azimuth in degrees
			radarData.orientation = myRadar.getOrientation();     // bot orientation with respect to the platform (in degrees)
			radarData.distanceToStagingPoint = myRadar.getDistanceToStagingPoint();	// range in meters
			radarData.angleToStagingPoint = myRadar.getAngleToStagingPoint();			// azimuth in degrees

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
				
			radarData.goodLocation = true;
		}
		else
		{
			cout << "got at least one bad radar range, so we won't calculate location" << endl;
			radarData.goodLocation = false;
		}

		radar_pub_.publish(radarData);
		
		if (radarData.goodData) cout << "GoodData is true, ";
		else cout << "goodData is false, ";
		if (radarData.goodLocation) cout << "goodLocation is true" << endl;
		else cout << "goodLocation is false" << endl;
		cout << "distanceToHome, angleToHome, Orientation = " << myRadar.getDistanceToHome() << " meters, "
			<< myRadar.getAngleToHome() << " degrees, " << myRadar.getOrientation() << " degrees, with respect to the platform" << endl;
		cout << "distanceToStagingPoint, angleToStagingPoint = " << myRadar.getDistanceToStagingPoint() << " meters, " << myRadar.getAngleToStagingPoint() << " degrees" << endl;
		cout << "runningAverageDistanceToHome, runningAverageAngleToHome = " << runningAverageDistanceToHome << " meters, " << runningAverageAngleToHome << " degrees" << endl;
		cout << "ranges from Bot Left to home left, right = " << myRadar.getDistanceFromLeftToLeft() << ", " << myRadar.getDistanceFromLeftToRight() << std::endl;
		cout << "ranges from Bot Right to home left, right = " << myRadar.getDistanceFromRightToLeft() << ", " << myRadar.getDistanceFromRightToRight() << std::endl;
		cout << endl << endl;
	   
	}
	return EXIT_SUCCESS;
}
