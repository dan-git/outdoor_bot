#include <string>

#include <ros/ros.h>
#include "outdoor_bot/obstacle_service.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_tester_node");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<outdoor_bot::obstacle_service>("obstacles_service");
  ros::Rate r(2);
  while (ros::ok())
  {
    std::string input;
    outdoor_bot::obstacle_service::Request req;
    req.params.min_angle = -0.707;
    req.params.max_angle = 0.707;
    outdoor_bot::obstacle_service::Response resp;
    client.call(req, resp);

    std::cout << "Obstacles are:\n";
    for (size_t i = 0; i < resp.obstacles.size(); i++)
    {
      std::cout << resp.obstacles[i] << "\n";
    }
  }
}
