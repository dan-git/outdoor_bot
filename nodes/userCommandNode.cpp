// node for entering a string of user commands
#include <ros/ros.h>
#include "std_msgs/String.h"

class userCommands
{
private:
  ros::NodeHandle nh_;
  ros::Publisher userCmd_pub_;
  double userCmdDistance_[32], userCmdTurn_[32], userCmdSpeed_[32], userCmdPickerUpper_[32];
  int userCmdNumDataValues_, userCmdReturnSection_;
  string userCommand_;
  bool userInputReceived_;
  std_msgs::String userString_;

public:
  // ROS node initialization
  userCommands(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for keyboard commands
    userCmd_pub_ = nh_.advertise<std_msgs::String>("user_commands", 2);
    for (int i=0; i < 32; i++)
    {
	  	userCmdDistance_[i] = 0.;
	  	userCmdTurn_[i] = 0.;
	  	userCmdSpeed_[i] = 0.;
	  	userCmdPickerUpper_[i] = 0.;
    }
	  userCmdNumDataValues_ = 0;
	  userCmdReturnSection_ = FIND_TARGETS;
	  userInputReceived_ = false;
	  userCommand_ = "";
  }
  
void parseCommands(const std_msgs::String::ConstPtr& msg)
{  
  string userCommand = msg->data.c_str();
  std::string cmdBuffer[32];
  string substr;

  std::size_t found = userCommand.find_first_of(";");
  userCmdNumDataValues_ = 0;
  while (found!=std::string::npos)
  {
    cmdBuffer[userCmdNumDataValues_] = userCommand.substr(0,found);
    userCmdNumDataValues_++;
    userCommand = userCommand.substr(found + 1, std::string::npos);
    found = userCommand.find_first_of(";");
  }
  
  ROS_INFO("parsed user command strings: ");
  for (int i = 0; i < userCmdNumDataValues_ - 1; i++)
  {
    cout << cmdBuffer[i].c_str() << endl;
    found = cmdBuffer[i].find_first_of(",");
    substr = cmdBuffer[i].substr(0,found);
    userCmdDistance_[i] = atof(substr.c_str());
    cmdBuffer[i] =  cmdBuffer[i].substr(found + 1, std::string::npos);
    found = cmdBuffer[i].find_first_of(",");
    substr = cmdBuffer[i].substr(0,found);
    userCmdTurn_[i] = atof(substr.c_str());
    cmdBuffer[i] = cmdBuffer[i].substr(found + 1, std::string::npos);
    found = cmdBuffer[i].find_first_of(",");
    substr = cmdBuffer[i].substr(0,found);
    userCmdSpeed_[i] = atof(substr.c_str()); 
    found = cmdBuffer[i].find_first_of(",");
    if (found!=std::string::npos)
    {
    	substr = cmdBuffer[i].substr(0,found);
    	userCmdPickerUpper_[i] = atof(substr.c_str()); 
    }    
  }
  
  substr = cmdBuffer[userCmdNumDataValues_].c_str();
  userCmdReturnSection_ = atof(substr.c_str());
 
}

void getUserInput()
{
   string input = "";
   userCommand_ = "";
   int comComplete = 0;
   while (comComplete == 0)
   {
		cout << "Enter distance, turn, speed, optional pickup  " << userCmdNumDataValues << ": " << endl;
		getline(cin, input);
		userCommand_ << input << ";";
		userCmdNumDataValues++;
		
		cout << "Finish up (1) or enter another command (0)?" << endl;
		getline(cin, input);

		// This code converts from string to number safely.
		stringstream myStream(input);
		if ( !(myStream >> comComplete) )
		{
		   cout << "Failed to understand comComplete, finishing up" << endl;
		   comComplete = 1;
		   return;
		}
    }
    
    cout << "Return section choices are as follows: " << endl;
    cout << "0 = BOOT, 1 = FIRST_TARGET, 2 = TARGETS, 3 = HOME, 4 = PLATFORM" << endl;
    cout << "Enter return section: " << endl;
    int sectionSelect;
	 getline(cin, input);
	 // check for valid entry
	 stringstream myStream(input);
	 if (myStream >> sectionSelect)
	 {
	 	if (sesctionSelect < BOOTUP || sectionSelect > PLATFORM)
		{
		   cout "invalid return section, defaulting to TARGETS" << endl;
		   userCommand_ << TARGETS << ";";
		}
		else userCommand_ << input << ";";
	 }
	 else
	 {
		 cout << "Failed to understand return section, defaulting to TARGETS" << endl;
		 userCommand_ << TARGETS << ";";
	 }
    
    userString_.data = userCommand_;
    userInputReceived_ = true;
}

std_msgs::String getuserString() { return userString_; }

void printUserCommands()
{
	cout << "distance, turn, speed, pickup: " << endl;
	for (i=0; i < userCmdNumDataValues; i++)
	{
		userCmdDistance[i] << ", " << userCmdTurn[i] << ", " << userCmdSpeed[i] << ", " << usrCmdPickup[i] << endl;
	}
	cout << "return section = " << userCmdReturnSection_ << "corresponding to ";
	if (userCmdReturnSection_ == BOOTUP) cout << "BOOTUP";
	else if (userCmdReturnSection_ == FIRST_TARGET) cout << "FIRST_TARGET";
	else if (userCmdReturnSection_ == TARGETS) cout << "TARGETS";
	else if (userCmdReturnSection_ == HOME) cout << "HOME";
	else if (userCmdReturnSection_ == PLATFORM) cout << "PLATFORM";
	else cout << "unknown section";
	cout << endl;

};

void publishCommands()
{
    int goPublish;
	 getline(cin, input);
	 stringstream myStream(input);
	 if (myStream >> goPublish)
	 {
	 	if (goPublish == 1)
		{
		   cout "publishing user commands" << endl;
		   userCmd_pub_.publish(userCommand_);
		}
		else cout << "not publishing" << endl;
	 }
	 else
	 {
		 cout << "Failed to understand publish command, not publishing" << endl;
	 }	
}



int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "userCommands");
  ros::NodeHandle nh;
  userCommands cmds(nh);
  while(nh.ok())
   {
   	cmds.getUserInput();	// we block here if there are no user inputs
   	cmds.parseCommands();
   	cmds.printCommands();
   	cmds.publishCommands();
   	ros::spinOnce();  // check for incoming messages
   	//struct timespec ts;
      //ts.tv_sec = 0;
      //ts.tv_nsec = 10000000;
      //nanosleep(&ts, NULL); // update every 10 ms
   }
}
