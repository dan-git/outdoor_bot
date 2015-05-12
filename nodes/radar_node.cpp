#include <ros/ros.h>
#include "outdoor_bot/radar_msg.h"
#include "rcmRadar/radar.h"
#include "outdoor_bot_defines.h"

#define RADAR_WAIT_TIME 0.5
#define HOME_RADAR_SEPARATION 1800 // distance between radars on home platform, in mm
#define BOT_RADAR_SEPARATION 1400

using namespace std;

radarRanger leftRanger_("/dev/ttyRadar_103");
radarRanger rightRanger_("/dev/ttyRadar_100");
double distanceFromLeftToLeft_ =0., distanceFromLeftToRight_ = 0., distanceFromRightToLeft_ = 0., distanceFromRightToRight_ = 0.;
double distanceToHome_ = 0., angleToHome_ = 0.;

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
	
	double cosLeftBotToRightHome =
	   ( (distanceFromLeftToRight_ * distanceFromLeftToRight_)
		+ (BOT_RADAR_SEPARATION * BOT_RADAR_SEPARATION)
		- (distanceFromRightToRight_ * distanceFromRightToRight_) )
		/  ( 2. * distanceFromLeftToRight_ * BOT_RADAR_SEPARATION);
		
	double botOrientation = 1.57 + acos(cosLeftHomeToLeftBot) + acos(cosLeftBotToRightHome);
	
	double distanceFromLeftToCenter = 
		( (distanceFromLeftToLeft_ * distanceFromLeftToLeft_) 
		+ ((HOME_RADAR_SEPARATION * HOME_RADAR_SEPARATION)/4.) )
		- (distanceFromLeftToLeft_ * HOME_RADAR_SEPARATION * cosLeftHomeToLeftBot);
	
	double cosAngleE = 
		( (distanceFromLeftToLeft_ * distanceFromLeftToLeft_) 
		+ (distanceFromLeftToCenter * distanceFromLeftToCenter)
		- ((HOME_RADAR_SEPARATION * HOME_RADAR_SEPARATION)/4.) )
		/ ( 2. * distanceFromLeftToLeft_ * distanceFromLeftToCenter);
	
	double anglef = acos(cosLeftHomeToLeftBot) - (botOrientation + 1.57);
	double angleh = 3.14 - (acos(cosAngleE) + anglef);
	
	double distanceFromCenterToCenterSquared = 
		((HOME_RADAR_SEPARATION * HOME_RADAR_SEPARATION)/4.)
		+ (distanceFromLeftToCenter * distanceFromLeftToCenter)
		- (HOME_RADAR_SEPARATION * distanceFromLeftToCenter * cos(angleh));
		
	double anglei = 
		( (distanceFromLeftToCenter * distanceFromLeftToCenter)
		+ ((HOME_RADAR_SEPARATION * HOME_RADAR_SEPARATION)/4.) 
		- distanceFromCenterToCenterSquared ) 
		/ (distanceFromLeftToCenter * HOME_RADAR_SEPARATION);
		
	double angleToHome_ = (1.57 - anglei) * 57.3;	// convert to degrees
	
	// convert to meters
	if  (distanceFromCenterToCenterSquared >= 0) distanceToHome_ = sqrt(distanceFromCenterToCenterSquared) / 1000. ;
	else cout << "distanceFromCenterToCenterSquared was negative, = " << distanceFromCenterToCenterSquared << endl;
	
	cout << "Home is " << distanceToHome_ << " meters away at an angle = "
			<< angleToHome_ << endl;	
}

int main(int argc, char** argv)
{
   //init the ROS node
   ros::init(argc, argv, "radar_node");
   ros::NodeHandle nh;

	ros::Publisher radar_pub_ = nh.advertise<outdoor_bot::radar_msg>("radar", 5);
	
	outdoor_bot::radar_msg radarData;
	ros::Time last_time;
   ros::Time current_time = ros::Time::now();
   
	while(nh.ok())
	{
		distanceFromLeftToLeft_ = ((double) leftRanger_.getRange(LEFT_RADAR_NUMBER));  
		radarData.distanceFromBotLeftToHomeLeft = distanceFromLeftToLeft_ / 1000.;	// convert from mm to meters
		last_time = ros::Time::now();
		while ( current_time.toSec() - last_time.toSec() < RADAR_WAIT_TIME) current_time = ros::Time::now(); // delay needed for radars to get data
		
		distanceFromRightToRight_ = ((double) rightRanger_.getRange(RIGHT_RADAR_NUMBER));  
		radarData.distanceFromBotRightToHomeRight = distanceFromRightToRight_ / 1000.;	// convert from mm to meters
		last_time = ros::Time::now();
		while ( current_time.toSec() - last_time.toSec() < RADAR_WAIT_TIME) current_time = ros::Time::now(); // delay needed for radars to get data

		distanceFromLeftToRight_ = ((double) leftRanger_.getRange(RIGHT_RADAR_NUMBER));
		radarData.distanceFromBotLeftToHomeRight = distanceFromLeftToRight_ / 1000.;
		last_time = ros::Time::now();
		while ( current_time.toSec() - last_time.toSec() < RADAR_WAIT_TIME) current_time = ros::Time::now(); 
		
		distanceFromRightToLeft_ = ((double) rightRanger_.getRange(LEFT_RADAR_NUMBER));
		radarData.distanceFromBotRightToHomeLeft = distanceFromRightToLeft_ / 1000.;
		last_time = ros::Time::now();
		
		double maxDistance1 = fmax(distanceFromLeftToLeft_, distanceFromLeftToRight_);
		double maxDistance2 = fmax(distanceFromRightToLeft_, distanceFromRightToRight_);
		double minDistance1 = fmin(distanceFromLeftToLeft_, distanceFromLeftToRight_);
		double minDistance2 = fmin(distanceFromRightToLeft_, distanceFromRightToRight_);
		radarData.maxDistanceToHome = fmax(maxDistance1, maxDistance2);
		radarData.minDistanceToHome = fmin(minDistance1, minDistance2);
		
		getLocation();
		radarData.distanceToHome = distanceToHome_;
		radarData.angleToHome = angleToHome_;		

		radar_pub_.publish(radarData);
		std::cout << "ranges from Bot Left to home left, right = " << distanceFromLeftToLeft_ << ", " << distanceFromLeftToRight_ << std::endl;
		std::cout << "ranges from Bot Right to home left, right = " << distanceFromRightToLeft_ << ", " << distanceFromRightToRight_ << std::endl;
		while ( current_time.toSec() - last_time.toSec() < RADAR_WAIT_TIME) current_time = ros::Time::now(); 	
		
		leftRanger_.getInitStatus();	
		rightRanger_.getInitStatus();	
	}
	return EXIT_SUCCESS;
}
