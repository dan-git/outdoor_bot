// a node to see obstacles from laser data
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Header.h"
#include <string>
#include <numeric>
#include <vector>

class obstacle_detect
{
	private:
		ros::NodeHandle nh_;
		ros::Subscriber laserSub_;
		double angle_min_, angle_max_, angle_increment_, time_increment_;
		long scanNum_;
		double scan_time_, range_min_, range_max_;
		std::vector<float> ranges_;
		
		void laserCallback(const sensor_msgs::LaserScan msg)
		{
			scanNum_++;
			std_msgs::Header header_ = msg.header;
			std::string headerFrameID = header_.frame_id;
			angle_min_ = msg.angle_min;
			angle_max_ = msg.angle_max;
			angle_increment_ = msg.angle_increment;
			ranges_ = msg.ranges;
		}
		
		
	public:
		obstacle_detect(ros::NodeHandle( &nh))
		: nh_(nh), scanNum_(0)
		{
			laserSub_ = nh.subscribe("scan", 10, &obstacle_detect::laserCallback, this);
		}
		
		std::vector<float> getRanges() { return ranges_; }
		double getAngleMin() { return angle_min_; }
		double getAngleMax() { return angle_max_; }
		double getAngleIncrement() { return angle_increment_; }
		long getScanNum() { return scanNum_; }
};


void laserStats(const std::vector<double>& x, const std::vector<double>& y, double *mean, double *slope)
{
    
    double n    = x.size();
    double s_x  = std::accumulate(x.begin(), x.end(), 0.0);
    double s_y  = std::accumulate(y.begin(), y.end(), 0.0);
    double s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
    double s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
    double a    = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
    *slope = a;
    *mean = s_y / n;
}


int main(int argc, char** argv)
{
   //init the ROS node
   ros::init(argc, argv, "obstacle_node");
   ros::NodeHandle nh;
   obstacle_detect oD(nh);

	std::vector<double> goodRanges;
	std::vector<double> goodAngles;
   long lastScanNum = oD.getScanNum();
   
   std::cout << "Enter any key to analyze scan data: " << std::endl;
	std::string input = "";
   getline(std::cin, input);
   while (oD.getScanNum() == lastScanNum)
   {
   	ros::spinOnce(); // wait for a new scan
   }
	
	lastScanNum = oD.getScanNum();
   std::vector<float> localRanges = oD.getRanges();
   //std::cout << std::endl << std::endl;
	//std::cout << "scan number: " << lastScanNum << " has " << localRanges.size() << " values" << std::endl;
	//std::cout << "angle min, max, increment = " << oD.getAngleMin() << ", " 
	//	<< oD.getAngleMax() << ", " << oD.getAngleIncrement() << std::endl;
	for (unsigned int i=0; i < localRanges.size(); i++)
	{
		if (std::isfinite(localRanges[i]))
		{
			goodRanges.push_back(localRanges[i]);
			goodAngles.push_back(oD.getAngleMin() + (i * oD.getAngleIncrement()));
		}
	}

	//std::cout << "scan number: " << lastScanNum << " has " << localRanges.size() << " values" << std::endl;
	//std::cout << "angle min, max, increment = " << oD.getAngleMin() << ", " 
	//<< oD.getAngleMax() << ", " << oD.getAngleIncrement() << std::endl;
	
	double averageDistance, obstacleSlope;
	laserStats(goodAngles, goodRanges, &averageDistance, &obstacleSlope);
	double closestObstacle = *std::min_element(goodRanges.begin(), goodRanges.end());
	double farthestObstacle = *std::max_element(goodRanges.begin(), goodRanges.end());
	std::cout << "Got a total of " << goodRanges.size() << " good ranges out of " << localRanges.size() << " total values reported., Stats on those are: " << std::endl;
	std::cout << "min, max angles = " << goodAngles[0] * 57.3 << ", " << goodAngles[goodAngles.size() - 1] * 57.3 << " degrees" << std::endl;
	std::cout << "min, max distances = " << closestObstacle << ", " << farthestObstacle << " meters" << std::endl;    
	std::cout << "average distance to laser line, slope of the line = " << averageDistance << ", " << obstacleSlope << std::endl;	

	//ros::spin();
}

