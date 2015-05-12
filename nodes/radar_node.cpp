#include <ros/ros.h>
#include "outdoor_bot/radar_msg.h"
#include "rcmRadar/radar.h"
#include "outdoor_bot_defines.h"

#define RADAR_WAIT_TIME 0.5
#define HOME_RADAR_SEPARATION 1900 // distance between radars on home platform, in mm
#define BOT_RADAR_SEPARATION 1092

using namespace std;

class radar
{

private:

	radarRanger leftRanger_, rightRanger_;
	double distanceFromLeftToLeft_, distanceFromLeftToRight_, distanceFromRightToLeft_, distanceFromRightToRight_;
	double distanceToHome_, angleToHome_, botOrientation_;

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
		
	double angleToHome_ = (1.57 - acos(cosAngleI)) * 57.3;	// convert to degrees
	
	// convert to meters
	if  (distanceFromCenterToCenterSquared >= 0) distanceToHome_ = sqrt(distanceFromCenterToCenterSquared) / 1000. ;
	else cout << "distanceFromCenterToCenterSquared was negative, = " << distanceFromCenterToCenterSquared << endl;
	
	cout << "cosLeftHomeToLeftBot = " << cosLeftHomeToLeftBot << ", an angle of " << acos(cosLeftHomeToLeftBot) * 57.3 << " degrees" << endl;
	cout << "cosAngleB = " << cosAngleB << ", an angle of " << acos(cosAngleB) * 57.3 << " degrees" << endl;
	cout << "cosAngleD = " << cosAngleD << ", an angle of " << acos(cosAngleD) * 57.3 << " degrees" << endl;
	cout << "botOrientation = " << botOrientation_ * 57.3 << " degrees " << endl;
	cout << "distanceFromLeftToCenterSquared = " << distanceFromLeftToCenterSquared << endl;
	cout << "distanceFromLeftToCenter = " << distanceFromLeftToCenter << endl;
	cout << "cosAngleE = " << cosAngleE << ", an angle of " << acos(cosAngleE) * 57.3 << " degrees" << endl;
	cout << "anglef = " << anglef * 57.3 << endl;
	cout << "angleh = " << angleh * 57.3 << endl;
	cout << "distanceFromCenterToCenterSquared = " << distanceFromCenterToCenterSquared << endl;	
	if (distanceFromCenterToCenterSquared >= 0)
		cout << "distanceFromCenterToCenter = " << sqrt(distanceFromCenterToCenterSquared) << endl;
	else cout << "distanceFromCenterToCenterSquared is a negative number " << endl;
	cout << "cosAngleI = " << cosAngleI << ", an angle of " << acos(cosAngleI) * 57.3 << " degrees " << endl;
	cout << "angleToHome_ = " << angleToHome_ << endl;
	
	cout << "Home is " << distanceToHome_ << " meters away at an angle = " << angleToHome_ << endl;
	cout << "Bot's orientation with respect to the platform = " << botOrientation_ * 57.3 << " degrees " << endl;	
}

double getDistanceFromLeftToLeft() { return distanceFromLeftToLeft_; }
double getDistanceFromLeftToRight() { return distanceFromLeftToRight_; }
double getDistanceFromRightToLeft() { return distanceFromRightToLeft_; }
double getDistanceFromRightToRight() { return distanceFromRightToRight_; }
double getDistanceToHome() { return distanceToHome_; }
double getAngleToHome() { return angleToHome_; }
double getOrientation() { return botOrientation_ * 57.3; }


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
   /*
   // 2 meters away, facing the robot, with radar seps both = 1000 (have to set that in the #defines)
   distanceFromLeftToLeft_ = 2000.;
	distanceFromLeftToRight_ = 2236.;
	distanceFromRightToLeft_ = 2236.;
	distanceFromRightToRight_ = 2000.;
	*/
	
	distanceFromLeftToLeft_ = 2393.;
	distanceFromLeftToRight_ = 2335.;
	distanceFromRightToLeft_ = 2706.;
	distanceFromRightToRight_ = 2155.;
	
	
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
		myRadar.getRadarRanges();  
		//myRadar.testDataRadarRanges();
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
		radarData.distanceToHome = myRadar.getDistanceToHome();	// range in meters
		radarData.angleToHome = myRadar.getAngleToHome();			// azimuth in degrees
		radarData.orientation = myRadar.getOrientation();     // bot orientation with respect to the platform (in degrees)

		radar_pub_.publish(radarData);
		std::cout << "ranges from Bot Left to home left, right = " << myRadar.getDistanceFromLeftToLeft() << ", " << myRadar.getDistanceFromLeftToRight() << std::endl;
		std::cout << "ranges from Bot Right to home left, right = " << myRadar.getDistanceFromRightToLeft() << ", " << myRadar.getDistanceFromRightToRight() << std::endl;

	}
	return EXIT_SUCCESS;
}
