// commands directional antenna, results come via robotPose
#include <ros/ros.h>
#include "outdoor_bot/dirAnt_msg.h"

#define MOVE_ANTENNA 0
#define PAN_SWEEP 1
#define PAN_CENTER 127

using namespace std;

class dirAnt
{

private:
	ros::NodeHandle nh_;
	ros::Publisher dirAnt_pub_;
	ros::Subscriber dirAnt_sub_;
	      
public:

	dirAnt(ros::NodeHandle nHandle)
	: nh_(nHandle)
	{
	dirAnt_pub_ = nh_.advertise<outdoor_bot::dirAnt_msg>("dirAnt_send", 5);
	dirAnt_sub_ = nh_.subscribe("dirAnt_cmd", 2, &dirAnt::commandCallback, this);	
	}
	
	void sweepAntenna()
	{
		outdoor_bot::dirAnt_msg msg;
		msg.antennaCommand = PAN_SWEEP;
		msg.antennaPan = PAN_CENTER;
		dirAnt_pub_.publish(msg);
	}

	void commandCallback(const outdoor_bot::dirAnt_msg msg)
	{
		if (msg.antennaCommand == PAN_SWEEP) sweepAntenna();
	}
};

int main(int argc, char** argv)
{
   //init the ROS node
   ros::init(argc, argv, "dirAnt_node");
   ros::NodeHandle nh;
   
   dirAnt myDirAnt(nh);
   
	while(nh.ok())
	{
		ros::spinOnce();
	}
	return EXIT_SUCCESS;
}

