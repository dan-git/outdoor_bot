#include <string>

#include <ros/ros.h>

#include "navigation/ObstacleDetector.h"
#include "outdoor_bot/obstacle_service.h"

namespace OutdoorBot
{
namespace Navigation
{

class ObstacleService
{
 public:
  ObstacleService()
  {
  }

  bool callback(outdoor_bot::obstacle_service::Request& request,
                outdoor_bot::obstacle_service::Response& response)
  {
    detector_.getObstacles(request.params, &(response.obstacles));
    return true;
  }

 private:
  ObstacleDetector detector_;
};

}  // namespace Navigation
}  // namespace OutdoorBot

int main(int argc, char** argv)
{
  // init the ROS node
  ros::init(argc, argv, "obstacles_service_node");

  OutdoorBot::Navigation::ObstacleService obstacle_service;
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService(
      "obstacles_service", &OutdoorBot::Navigation::ObstacleService::callback, &obstacle_service);

  ros::spin();
}

