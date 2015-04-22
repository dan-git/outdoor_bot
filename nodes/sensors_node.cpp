// sensors node
// ros stuff
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <stdio.h>

// log files
//#include <fstream>

//using namespace std;
//using namespace ros;

ros::Publisher sensors_pub_;

std::string exec(char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
    	if(fgets(buffer, 128, pipe) != NULL)
    		result += buffer;
    }
    pclose(pipe);
    return result;
}


int main(int argc, char** argv){
  //Initialize ROS
  ros::init(argc, argv, "sensors_node");
  ros::NodeHandle n;
  ROS_INFO("sensors_node starting");

  sensors_pub_ = n.advertise<std_msgs::String>("sensor_readings", 50);

  char* getLaptopBattState = "upower -i /org/freedesktop/UPower/devices/battery_BAT0 | grep -E \"state|time\ to|percentage\"";
   double durationValue = 0., percentageValue = 0.;
   std::string batteryString, chargingString, durationString, percentageString;
   std_msgs::String batteryStr;
   
   while (n.ok())
   {
      std::string LaptopBattState = exec(getLaptopBattState);

      // find out if the laptop battery is charging or discharging
      std::size_t found = LaptopBattState.find("discharging");
      if (found != std::string::npos) chargingString = "discharging";
      else
      {
         found = LaptopBattState.find("charging");
         if (found != std::string::npos) chargingString = "charging";
         else
         {
            found = LaptopBattState.find("fully");
            if (found != std::string::npos) chargingString = "fully charged";
            else chargingString = "unknown";
         }
      }

      // get percentage of laptop battery charge
      found = LaptopBattState.find_first_of("%");
      if (found != std::string::npos && found > 7) 
      {
         percentageString = LaptopBattState.substr(found - 7, 7);    
         percentageValue = atof(percentageString.c_str());
      }

      ROS_INFO("Laptop battery is %s, percent charge = %f",chargingString.c_str(), percentageValue);

      std::string pubString;
      pubString.assign(chargingString);
      pubString.append("," + percentageString + "%"); 

      batteryStr.data = pubString.c_str();
      sensors_pub_.publish(batteryStr);
      ros::spinOnce();
      usleep(60000000); // publish once a minute
   }     
}

