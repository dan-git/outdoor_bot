// Uses POSIX functions to send and receive data from the virtual serial
// port of a Pololu Simple Motor Controller.
// NOTE: The Simple Motor Controller's Input Mode must be set to Serial/USB.
// NOTE: You must change the 'const char * device' line below.
 
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <string>
 
#define SERIAL_ERROR -9999

ros::Publisher motorCommand;
ros::Subscriber moveCmd, motorResponse;
 
// Reads a variable from the SMC and returns it as number between 0 and 65535.
// Returns SERIAL_ERROR if there was an error.
// The 'variableId' argument must be one of IDs listed in the
// "Controller Variables" section of the user's guide.
// For variables that are actually signed, additional processing is required
// (see smcGetTargetSpeed for an example).

int smcGetVariable(int fd, unsigned char variableId)
{
  unsigned char command[] = {0xA1, variableId};
  if(write(fd, &command, sizeof(command)) == -1)
  {
    perror("error writing");
    return SERIAL_ERROR;
  }
 
  unsigned char response[2];
  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return SERIAL_ERROR;
  }
 
  return response[0] + 256*response[1];
}
 
// Returns the target speed (-3200 to 3200).
// Returns SERIAL_ERROR if there is an error.
int smcGetTargetSpeed(int fd)
{
  int val = smcGetVariable(fd, 20);
  return val == SERIAL_ERROR ? SERIAL_ERROR : (signed short)val;
}
 
// Returns a number where each bit represents a different error, and the
// bit is 1 if the error is currently active.
// See the user's guide for definitions of the different error bits.
// Returns SERIAL_ERROR if there is an error.
int smcGetErrorStatus(int fd)
{
  return smcGetVariable(fd,0);
}
 
// Sends the Exit Safe Start command, which is required to drive the motor.
// Returns 0 if successful, SERIAL_ERROR if there was an error sending.
int smcExitSafeStart(int fd)
{
  const unsigned char command = 0x83;
  if (write(fd, &command, 1) == -1)
  {
    perror("error writing");
    return SERIAL_ERROR;
  }
  return 0;
}
 
// Sets the SMC's target speed (-3200 to 3200).
// Returns 0 if successful, SERIAL_ERROR if there was an error sending.
int smcSetTargetSpeed(int fd, int speed)
{
  unsigned char command[3];
 
  if (speed < 0)
  {
    command[0] = 0x86; // Motor Reverse
    speed = -speed;
  }
  else
  {
    command[0] = 0x85; // Motor Forward
  }
  command[1] = speed & 0x1F;
  command[2] = speed >> 5 & 0x7F;
 
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return SERIAL_ERROR;
  }
  return 0;
}

void calculateMove(double vx, double vTheta)
{
   char intStr[33];	// needs to be large enough to take biggest possible number and /0
   std::string commandString;
   //if (roger_)	// command format for autobot, just send linear and ang vels
  // {
      sprintf(intStr,"%f",vx * 1000.);	// send instruction in mm/sec
      std::string value(intStr);
      commandString = "autoC(";
      commandString.append(intStr);
      commandString.append(",");
      sprintf(intStr,"%f",vTheta  * 57.1); // send instruction in degrees/sec
      commandString.append(intStr);
      commandString.append(");");
   //}
/*
   else	// command format for standard bot
   {
	double turning = vTheta * WHEEL_SEPARATION / 2; // gives the required wheel speed in m/s
			// for example to turn at 1 rotation per second, vTheta would = 2 * pi
			// and the distance the wheel travels is 2 * pi * r
			// because the circumference = 2 pi r
	//ROS_INFO("turning %f", turning);
	double moving = vx;  // we are only able to move forward, not to the side, so we cannot use vy
	//ROS_INFO("moving %f", moving);
	int rightWheelSpeed = (int) ((moving + turning) * 1000.);  // use mm/sec instead of m/s, so we can get integers
	int leftWheelSpeed = (int) ((moving - turning) * 1000.); 
	//ROS_INFO("left wheel speed %d", leftWheelSpeed);
	//ROS_INFO("right wheel speed %d", rightWheelSpeed);
	char intStr[33];	// needs to be large enough to take biggest possible number and /0
	sprintf(intStr,"%d",rightWheelSpeed);
	std::string rightWheelSpeedString(intStr);
	sprintf(intStr,"%d",leftWheelSpeed);
	std::string leftWheelSpeedString(intStr);

	commandString = "M";
	commandString.append(leftWheelSpeedString);
	commandString.append(",");
	commandString.append(rightWheelSpeedString);
	commandString.append("#");
   }   
*/  
   std_msgs::String msg;
   msg.data = commandString;
   motorCommand.publish(msg);	// send out a message to serial comm for transmission to the arduino
   //ROS_INFO("velocity command sent = %s", commandString.c_str());
}

void moveCommandCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  
  // (vx, vy, vtheta) <==> (cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z
  static bool enabled = true, disable_commanded = false;
  double vx = cmd_vel->linear.x;
  //double vy = cmd_vel->linear.y;  // we are not a holonomic robot, so vy is always 0
  double disable_indicator = cmd_vel->linear.z;
  double vTheta = cmd_vel->angular.z;
  //ROS_INFO("command received %f", vx);
  if (disable_indicator > 0.5)
  {
    ROS_INFO("commands re-enabled");
    enabled = true;
  }
  else if (disable_indicator < -0.5)
  {
    ROS_INFO("commands disabled");
    enabled = false;
    // when the disable command is issued, we want to send a single M0,0# command to get the bot to stop
    disable_commanded = true;	
  }
  if (enabled) calculateMove(vx, vTheta);
  else if (disable_commanded) // when the disable command is issued, we want to send a single M0,0# command to get the bot to stop
  {
    calculateMove(0.0,0.0);
    disable_commanded = false;
  }
}
 
int main(int argc, char** argv)
{
    //Initialize ROS
  ros::init(argc, argv, "motorDriver");
  ros::NodeHandle n;
  ROS_INFO("pololu motorDriver starting");

  moveCmd = n.subscribe("cmd_vel", 50, moveCommandCallback);  // subscribe to move commands
  //motorResponse = n.subscribe("motorResponse", 50, motorResponseCallback);

  // advertise commands to be sent to the motor's serial port
  motorCommand = n.advertise<std_msgs::String>("motorCommand", 50); 
  
  // Open the Simple Motor Controller's virtual COM port.
  const char * device = "/dev/ttyACM0";  // Linux
  int fd = open(device, O_RDWR | O_NOCTTY);

  smcExitSafeStart(fd);
 
  printf("Error status: 0x%04x\n", smcGetErrorStatus(fd));
 
  int speed = smcGetTargetSpeed(fd);
  printf("Current Target Speed is %d.\n", speed);
 
  int newSpeed = (speed <= 0) ? 500 : -500;
  printf("Setting Target Speed to %d.\n", newSpeed);
  smcSetTargetSpeed(fd, newSpeed);
 
  while(n.ok()) ros::spinOnce();  // check for incoming messages
}
