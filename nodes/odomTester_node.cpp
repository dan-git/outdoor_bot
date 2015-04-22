// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
//#include <tf/transform_listener.h>

// messages
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// log files
#include <fstream>

using namespace std;
using namespace ros;
//using namespace tf;

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef boost::shared_ptr<geometry_msgs::Twist const> VelConstPtr;

ros::Publisher cmd_vel_pub_;
ros::Subscriber moveCmd_, odom_;
std::ofstream odom_file_;

bool firstMove_ = true, firstTurn_ = true, moving_ = false, turning_ = false;

void stopMoving()
{
      geometry_msgs::Twist base_cmd;
      base_cmd.angular.x = 0.0;
      base_cmd.angular.y = 0.0;
      base_cmd.angular.z = 0.0;
      base_cmd.linear.x = 0.0;
      base_cmd.linear.y = 0.0;
      base_cmd.linear.z = 0.0;
      cmd_vel_pub_.publish(base_cmd);
      moving_ = false;
      turning_ = false;
      firstTurn_ = true;
      firstMove_ = true;
}

void checkMove(double x, double y)
{
  double distanceLimit = 1.0; // 1.0 meter move
  double distanceLimitSquared = distanceLimit * distanceLimit;
  double distanceMovedSquared;
  static double initialX, initialY;
  if (firstMove_)
  {
     firstMove_ = false;
     initialX = x;
     initialY = y;
     odom_file_ << "moved started at x, y = " << x << ", " << ", " << y << endl;
     return;
  }
  distanceMovedSquared = ((x - initialX)*(x - initialX)) + ((y - initialY)*(y - initialY));
  if ( distanceMovedSquared > distanceLimitSquared)
  {
    stopMoving();
    odom_file_ << "moved stopped at x, y, distance moved = " << x << ", " << y << ", " << sqrt(distanceMovedSquared) << endl;
   }
}

void checkTurn(double yaw)
{
  double angle = 1.57; // 90 degree turn
  static double initialYaw;
  if (firstTurn_)
  {
     firstTurn_ = false;
     initialYaw = yaw;
     odom_file_ << "turn started at yaw = " << yaw << endl;
     return;
  }
  if (yaw >= initialYaw + angle  || yaw < initialYaw - angle)
  {
     stopMoving();
     odom_file_ << "turn stopped at yaw = " << yaw << endl;
  }
}



//void moveCommandCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
void moveCommandCallback(const VelConstPtr& cmd_vel)
{
  
  // (vx, vy, vtheta) <==> (cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z
  double vx = cmd_vel->linear.x;
  double vy = cmd_vel->linear.y;
  double vTheta = cmd_vel->angular.z;	

  ROS_DEBUG("odom testing started, vx = %f, vy = %f, vTheta = %f", vx, vy, vTheta);
  odom_file_ << "command:  " << vx << " " << vy << "  " << vTheta << endl;
  if ( abs(vx) + abs(vy) > 0.001) moving_ = true;
  else if ( abs(vTheta) > 0.001) turning_ = true;
}

void odomCallback(const OdomConstPtr& odom)
{
      
  
    // receive data 
    //odom_stamp_ = odom->header.stamp;
    //odom_time_  = Time::now();
    // from EKF methods
    //Quaternion q;
    //tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    //odom_meas_  = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));
    //double tmp, yaw;
    //odom_meas_.getBasis().getEulerYPR(yaw, tmp, tmp);
    //ROS_INFO("odom testing, x = %f, y = %f, Theta = %f", odom_meas_.getOrigin().x(), odom_meas_.getOrigin().y(), yaw);

    // from robotPose directly
    //get the position
    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;
    //double z = odom->pose.pose.position.z;
    //geometry_msgs::Quaternion odom_quat = odom->pose.pose.orientation;
    //double yaw = tf::getYaw(odom_quat);
    double yaw = tf::getYaw(odom->pose.pose.orientation);
    //get the velocity

    double vx = odom->twist.twist.linear.x;
    double vy = odom->twist.twist.linear.y;
    double vYaw = odom->twist.twist.angular.z;

    double vTotal = sqrt((vx * vx) + (vy * vy));

    if (moving_) checkMove(x, y);
    else if (turning_) checkTurn(yaw);

    odom_file_ << "odom:  " << vx << " " << vy << "  " << vTotal << " " << vYaw << "  " << x << "  " << y << "  " << yaw << endl;

}

int main(int argc, char** argv){
  //Initialize ROS
  ros::init(argc, argv, "odomTester");
  ros::NodeHandle n;
  ROS_INFO("odomTester starting");
  odom_file_.open("/home/dbarry/ros/myBot/testing/odom_test.txt");
  odom_file_ << "odom testing file, vx, vy, vTotal, vYaw, x, y, yaw: " << endl;
  moveCmd_ = n.subscribe("cmd_vel", 50, moveCommandCallback);  // subscribe to move commands
  odom_ = n.subscribe("odom", 50, odomCallback);  // subscribe to odometry
  cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); // publish on cmd_vel

  while(n.ok()) ros::spinOnce();  // check for incoming messages
  odom_file_.close();
}
