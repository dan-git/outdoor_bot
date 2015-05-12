#include <ros/ros.h>
#include "outdoor_bot/radar_msg.h"
#include "rcmRadar/radar.h"
#include "outdoor_bot_defines.h"

#define RADAR_WAIT_TIME 0.5
#define HOME_RADAR_SEPARATION 1000 //1800 // distance between radars on home platform, in mm
#define BOT_RADAR_SEPARATION 1000 //1400

using namespace std;

class radar
{

private:

	radarRanger leftRanger_, rightRanger_;
	double distanceFromLeftToLeft_, distanceFromLeftToRight_, distanceFromRightToLeft_, distanceFromRightToRight_;
	double distanceToHome_, angleToHome_;

public:
	radar()
		: leftRanger_("/dev/ttyRadar_103"),
		  rightRanger_("/dev/ttyRadar_100"),
		  distanceFromLeftToLeft_(0), distanceFromLeftToRight_(0.), distanceFromRightToLeft_(0.), distanceFromRightToRight_(0),
		  distanceToHome_(0.),
		  angleToHome_(0)
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
	
	double cosLeftBotToRightHome =
	   ( (distanceFromLeftToRight_ * distanceFromLeftToRight_)
		+ (BOT_RADAR_SEPARATION * BOT_RADAR_SEPARATION)
		- (distanceFromRightToRight_ * distanceFromRightToRight_) )
		/  ( 2. * distanceFromLeftToRight_ * BOT_RADAR_SEPARATION);
		
	double botOrientation = 1.57 + acos(cosLeftHomeToLeftBot) + acos(cosLeftBotToRightHome);
	
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
	
	double anglef = acos(cosLeftHomeToLeftBot) - (botOrientation + 1.57);
	double angleh = 3.14 - (acos(cosAngleE) + anglef);
	
	double distanceFromCenterToCenterSquared = 
		((BOT_RADAR_SEPARATION * BOT_RADAR_SEPARATION)/4.)
		+ (distanceFromLeftToCenter * distanceFromLeftToCenter)
		- (BOT_RADAR_SEPARATION * distanceFromLeftToCenter * cos(angleh));
		
	double anglei = 
		( (distanceFromLeftToCenter * distanceFromLeftToCenter)
		+ ((BOT_RADAR_SEPARATION * BOT_RADAR_SEPARATION)/4.) 
		- distanceFromCenterToCenterSquared ) 
		/ (distanceFromLeftToCenter * BOT_RADAR_SEPARATION);
		
	double angleToHome_ = (1.57 - anglei) * 57.3;	// convert to degrees
	
	// convert to meters
	if  (distanceFromCenterToCenterSquared >= 0) distanceToHome_ = sqrt(distanceFromCenterToCenterSquared) / 1000. ;
	else cout << "distanceFromCenterToCenterSquared was negative, = " << distanceFromCenterToCenterSquared << endl;
	
	cout << "Home is " << distanceToHome_ << " meters away at an angle = "
			<< angleToHome_ << endl;	
}

double getDistanceFromLeftToLeft() { return distanceFromLeftToLeft_; }
double getDistanceFromLeftToRight() { return distanceFromLeftToRight_; }
double getDistanceFromRightToLeft() { return distanceFromRightToLeft_; }
double getDistanceFromRightToRight() { return distanceFromRightToRight_; }
double getDistanceToHome() { return distanceToHome_; }
double getAngleToHome() { return angleToHome_; }


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

void testDataRadarRanges()
{  
   distanceFromLeftToLeft_ = 2000.;
	distanceFromLeftToRight_ = 2236.;
	distanceFromRightToLeft_ = 2236.;
	distanceFromRightToRight_ = 2000.;
}
	
};

int main(int argc, char** argv)
{
   //init the ROS node
   ros::init(argc, argv, "radar_node");
   ros::NodeHandle nh;

	ros::Publisher radar_pub_ = nh.advertise<outdoor_bot::radar_msg>("radar", 5);
	
	outdoor_bot::radar_msg radarData;

   
   radar myRadar;
   
	while(nh.ok())
	{
		//myRadar.getRadarRanges();  
		myRadar.testDataRadarRanges();
		radarData.distanceFromBotLeftToHomeLeft = myRadar.getDistanceFromLeftToLeft() / 1000.;	// convert from mm to meters
 
		radarData.distanceFromBotRightToHomeRight = myRadar.getDistanceFromRightToRight() / 1000.;

		radarData.distanceFromBotLeftToHomeRight = myRadar.getDistanceFromLeftToRight() / 1000.;

		radarData.distanceFromBotRightToHomeLeft = myRadar.getDistanceFromRightToLeft() / 1000.;
		
		double maxDistance1 = fmax(myRadar.getDistanceFromLeftToLeft(), myRadar.getDistanceFromLeftToRight());
		double maxDistance2 = fmax(myRadar.getDistanceFromRightToLeft(), myRadar.getDistanceFromRightToRight());
		double minDistance1 = fmin(myRadar.getDistanceFromLeftToLeft(), myRadar.getDistanceFromLeftToRight());
		double minDistance2 = fmin(myRadar.getDistanceFromRightToLeft(), myRadar.getDistanceFromRightToRight());
		radarData.maxDistanceToHome = fmax(maxDistance1, maxDistance2) / 1000.;
		radarData.minDistanceToHome = fmin(minDistance1, minDistance2) / 1000.;
		
		myRadar.getLocation();
		radarData.distanceToHome = myRadar.getDistanceToHome();	// already in meters
		radarData.angleToHome = myRadar.getAngleToHome();			// in degrees

		radar_pub_.publish(radarData);
		std::cout << "ranges from Bot Left to home left, right = " << myRadar.getDistanceFromLeftToLeft() << ", " << myRadar.getDistanceFromLeftToRight() << std::endl;
		std::cout << "ranges from Bot Right to home left, right = " << myRadar.getDistanceFromRightToLeft() << ", " << myRadar.getDistanceFromRightToRight() << std::endl;

	}
	return EXIT_SUCCESS;
}
