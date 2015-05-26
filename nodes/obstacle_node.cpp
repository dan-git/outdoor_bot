// a node to see obstacles from laser data
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Header.h"

class obstacle_detect
{
	private:
		ros::NodeHandle nh_;
		ros::Subscriber laserSub_;
		double angle_min_, angle_max_, angle_increment_, time_increment_;
		double scan_time_, range_min_, range_max_;
		std::vector<float> ranges_;
		
		void laserCallback(const sensor_msgs::LaserScan msg)
		{
			std_msgs::Header header_;
			// = msg.Header.frame_id;
			angle_min_ = msg.angle_min;
			angle_increment_ = msg.angle_increment;
			ranges_ = msg.ranges;
		}
		
	public:
		obstacle_detect(ros::NodeHandle( &nh))
		: nh_(nh)
		{
			laserSub_ = nh.subscribe("scan", 10, &obstacle_detect::laserCallback, this);
		}
		
		std::vector<float> getRanges() { return ranges_; }
		double getAngleMin() { return angle_min_; }
		double getAngleIncrement() { return angle_increment_; }
};

int main(int argc, char** argv)
{
   //init the ROS node
   ros::init(argc, argv, "obstacle_node");
   ros::NodeHandle nh;
   obstacle_detect oD(nh);
   std::vector<float> localRanges = oD.getRanges();
   for (unsigned int i=0; i < localRanges.size(); i++)
   {
   	std::cout << "laser angle, range: " << oD.getAngleMin() + (i * oD.getAngleIncrement());
   	std::cout << ", " << localRanges[i] << std::endl;
   }
   ros::spin();
}

