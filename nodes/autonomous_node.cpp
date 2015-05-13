#include <ros/ros.h>
#include "outdoor_bot/webcams_custom.h"
#include "outdoor_bot/digcams_service.h"
#include "outdoor_bot/digcams_custom.h"
#include "outdoor_bot/NavTargets_service.h"
#include "outdoor_bot/mainTargets_service.h"
#include "outdoor_bot/NavTargets_msg.h"
#include "outdoor_bot/mainTargets_service.h"
#include "outdoor_bot/mainTargets_msg.h"
#include "outdoor_bot/mainTargetsCommand_msg.h"
#include "outdoor_bot/mainTargets_imageReceived_msg.h"
#include "outdoor_bot/radar_service.h"
#include "outdoor_bot/setPose_service.h"
//#include "outdoor_bot/encoders_service.h"
#include "outdoor_bot/servo_msg.h"
//#include "outdoor_bot/pmotor_msg.h"
#include "outdoor_bot/movement_msg.h"
#include "FBFSM/FBFSM.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "outdoor_bot_defines.h"
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <time.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>

using namespace std;

#define PHASE_TWO_TIME_LIMIT 7200   //  7200 seconds in two hours-- phase two challenge
#define PHASE_ONE_TIME_LIMIT 1800 // 1800 seconds in 30 min-- phase one challenge
#define plat1_X -6.07 //2269.5 pixels x = -6.075176, y = 39.607155, yaw = -0.953306
#define plat1_Y 39.61 //832.5
#define plat1_Yaw -0.95 
#define plat2_X  -0.36 // 2203.5 x = -0.357296, y = 46.932579, yaw = -1.099379
#define plat2_Y 46.93 //915.
#define plat2_Yaw -1.10
#define plat3_X 6.11 // 2143.5 x = 6.108997, y = 54.660473, yaw = -1.237705
#define plat3_Y  54.66 // 990.
#define plat3_Yaw -1.23
#define target1_X 28.09 // 2505.  x = 28.092287, y = -8.878185, yaw = 2.011092
#define target1_Y -8.88 //1504.
//#define PIXELS_PER_METER 10.6
//#define METERS_PER_PIXEL 0.0943	use this value for resolution: in global and local_costmap_params.yaml and in field_map.yaml
#define INCREMENTAL_MOVE_TO_TARGET 10.0
#define FIRST_MOVE_REMAINING_DISTANCE 10.0

#define SERVO_WAIT_SECONDS 5.0  // time from servo command until we are sure servo is in position
#define DEGREES_PER_PAN_STEP 0.64
#define PAN_CAMERA_DELTA 20
#define PAN_CAMERA_SEARCH_MAX 40
#define TILT_CAMERA_DELTA 5
#define TILT_CAMERA_SEARCH_MAX 20


ros::ServiceClient NavTargets_client_, radar_client_; //, encoders_client_; //, mainTargets_client_, mainTargetsCheckImage_client_;
ros::ServiceClient setPose_client_; //digcams_client_, 
ros::Publisher digcam_pub_, movement_pub_, webcam_pub_, mainTargetsCommand_pub_, NavTargetsCommand_pub_, servo_pub_; //, pmotor_pub_;
ros::Subscriber home_center_sub_, targetImageReceived_sub_, target_center_sub_, move_complete_sub_, pause_sub_;
outdoor_bot::webcams_custom webcam_command_;
outdoor_bot::digcams_custom digcam_command_; 
int centerX_, centerY_, totalX_, moving_, turning_, cameraType_ = WEBCAM, totalMoveToFirstTarget_ = 0;
int home_image_height_, home_image_width_;
bool homeCenterUpdated_, movementComplete_, triedWebcamAlready_, triedZoomDigcamAlready_;
double range_, approxRangeToTarget_, targetRange_, homeCameraRange_, offsetX_;
bool zoomResult_, writeFileResult_, newMainTargetDigcamImageReceived_, newMainTargetWebcamImageReceived_, rangeUnknown_;
bool targetCenterUpdated_, newNavTargetImageReceived_;
int retCapToMemory_;
double distanceToHome_[NUM_RADARS], deltaDistanceToHome_[NUM_RADARS], velocityFromHome_[NUM_RADARS];
int platformXPose_[3], platPoseX_;
int platformYPose_[3], platPoseY_;
int platformYawPose_[3], platPoseYaw_;
int targetXPose_, targetYPose_;
bool phase1_, pauseNewCommand_, pauseCommanded_, previousState_, recoverTurnedAlready_, firstMoveToFirstTarget_;
int deltaPan_ = PAN_CAMERA_DELTA, deltaTilt_ = TILT_CAMERA_DELTA, searchCounter_ = 0, currentPan_ = 0; 
int maxSearchPan_ = PAN_CAMERA_SEARCH_MAX, maxSearchTilt_ = TILT_CAMERA_SEARCH_MAX;
int lastCamType_ = WEBCAM;
std::string movementResult_;
//int encoderPickerUpper_, encoderDropBar_, encoderBinShade_;
bool prepping_, placing_, driving_, pushing_, scooping_, dropping_, retrieving_;
ros::Time overallTimer_;
double startTime_, totalTime_, secondsRemaining_;


FBFSM fsm_;
int BootupState_, CheckLinedUpState_, CheckFirstTargetState_, SearchForFirstTargetState_;
int MoveToFirstTargetState_;
int FindTargetState_, MoveToTargetState_, PickupTargetState_;
int CheckHomeState_, SearchForHomeState_;
int HeadForHomeState_, MoveOntoPlatformState_;
int MoveToRecoverState_, MoveToRecoverFailedState_, ReEntryState_, PauseState_, AllDoneState_;
int MoveToRecoverCounter_ = 0, TurnToRecoverCounter_ = 0;
int platformNumber_;

bool getUserInput();

void imageCapture(string command, int camNum, int camType, bool writeFile = false);

class targetAquireFSM
{
   private:
      FBFSM tAfsm_;

      int moveCameraState_, captureImageState_, analyzeImageState_, acquireDoneState_, acquireReEntryState_, acquirePauseState_;
      int acquirePreviousState_;
      string camCommand_;
      int servoNumber_;
      bool firstTarget_, homeTarget_;
      int camType_, cam_, pan_, tilt_; // servo full range goes from 0 to 256
      ros::Time servoTimer_;

      void on_enter_MoveCamera()
      {
         if (pauseCommanded_) return;
         outdoor_bot::servo_msg msg;
         msg.servoNumber = servoNumber_;
         msg.servoPan = pan_;
         msg.servoTilt = tilt_;
         servo_pub_.publish(msg);
         servoTimer_ = ros::Time::now(); // record time that the command was sent
      }

      int on_update_MoveCamera()
      {
         if (pauseCommanded_)
         {
            acquirePreviousState_ = moveCameraState_;
            return acquirePauseState_;
         }
         // we dont get servo feedback, so we just have to use a fixed delay for servos to get there
         double secondsSinceServoCmdSent = ros::Time::now().toSec() - servoTimer_.toSec();
         if (secondsSinceServoCmdSent < SERVO_WAIT_SECONDS)	return moveCameraState_;
         else return acquireDoneState_;
      }

      void on_enter_CaptureImage()
      {
         if (homeTarget_) newNavTargetImageReceived_ = false;
         else if (camType_ == DIGCAM) newMainTargetDigcamImageReceived_ = false;
         else newMainTargetWebcamImageReceived_ = false;
         imageCapture(camCommand_, cam_, camType_, false);  // command an image capture to memory
      }

      int on_update_CaptureImage()
      {
            if (!homeTarget_)
            {
            	if ( (newMainTargetDigcamImageReceived_ && camType_ == DIGCAM)
            		|| (newMainTargetWebcamImageReceived_ && camType_ == WEBCAM) ) return analyzeImageState_; // image received by mainTargets
            }
            else if (newNavTargetImageReceived_) return analyzeImageState_; // image analyzed by NavTargets
            return captureImageState_;
      }

      //void on_exit_CaptureImage();

      void on_enter_AnalyzeImage()
      {
         if (homeTarget_)
         {
         	cout << "sending command to analyze image for home target" << endl;
         	newNavTargetImageReceived_ = false;
         	std_msgs::String msg;
         	msg.data = "home";
         	NavTargetsCommand_pub_.publish(msg); 
         }
         else
         {
		      cout << "sending command to mainTargets to analyze image" << endl;
		      if (newMainTargetDigcamImageReceived_ && camType_ == DIGCAM) newMainTargetDigcamImageReceived_ = false;
		      if (newMainTargetWebcamImageReceived_ && camType_ == WEBCAM) newMainTargetWebcamImageReceived_ = false;
		      outdoor_bot::mainTargetsCommand_msg msg; 
		      msg.cameraType = camType_;
		      msg.firstTarget = firstTarget_;  
		      mainTargetsCommand_pub_.publish(msg);
		   }
      }  


      int on_update_AnalyzeImage()
      {
         if (homeCenterUpdated_)
         {
         	homeCenterUpdated_ = false;
         	return acquireDoneState_;
         }
         else if (targetCenterUpdated_)
         {
            targetCenterUpdated_ = false;
            return acquireDoneState_;
         }
         return analyzeImageState_;
      }

      //void on_exit_AnalyzeImage();
      void on_enter_acquirePauseState()
      {
         // if the current state is a recovery state, then we must not change previousState_
         // otherwise, set previousState_ = currentState
         // also remember variables and other things needed to recover when motion is resumed
      }
         
      // provides a way for a state to run its on_enter code again Note to be careful-- its on_exit code will get run too!!
      int on_update_acquireReEntryState()
      {
         return acquirePreviousState_;
      }
      
      int on_update_acquirePauseState()
      {
         if (pauseCommanded_) return acquirePauseState_;
         return acquirePreviousState_;
      }

      void on_enter_state_done()
      {
         cout << "entered targetAquire DoneState" << endl;
      }

      int on_update_state_done()
      {
         cout << "updating targetAquire DoneState" << endl;
	      return acquireDoneState_;
      }

      void on_transition(int from_state, int to_state)
      {
	      ROS_INFO("tAfsm: Transitioning from state %d (%s) to state %d (%s).",
		      from_state, tAfsm_.state_name(from_state).c_str(), to_state,
		      tAfsm_.state_name(to_state).c_str());
      }

      void setupFSM()
      {
         moveCameraState_ = tAfsm_.add_state("MoveCameraState");
         captureImageState_ = tAfsm_.add_state("CaptureImageState");
         analyzeImageState_ = tAfsm_.add_state("AnalyzeImageState");
         acquireReEntryState_ = tAfsm_.add_state("AcquireReEntryState");
         acquirePauseState_ = tAfsm_.add_state("AcquirePauseState");
         acquireDoneState_ = tAfsm_.add_state("AcquireDoneState");

	      tAfsm_.set_transition_function(boost::bind(&targetAquireFSM::on_transition, this, _1, _2));

         tAfsm_.set_entry_function(moveCameraState_, boost::bind(&targetAquireFSM::on_enter_MoveCamera, this));
         tAfsm_.set_update_function(moveCameraState_, boost::bind(&targetAquireFSM::on_update_MoveCamera, this));

         tAfsm_.set_entry_function(captureImageState_, boost::bind(&targetAquireFSM::on_enter_CaptureImage, this));
         tAfsm_.set_update_function(captureImageState_, boost::bind(&targetAquireFSM::on_update_CaptureImage, this));
         //tAfsm_.set_exit_function(captureImageState_, boost::bind(&targetAquireFSM::on_exit_CaptureImage, this));

         tAfsm_.set_entry_function(analyzeImageState_, boost::bind(&targetAquireFSM::on_enter_AnalyzeImage, this));
         tAfsm_.set_update_function(analyzeImageState_, boost::bind(&targetAquireFSM::on_update_AnalyzeImage, this));

         tAfsm_.set_update_function(acquireReEntryState_, boost::bind(&targetAquireFSM::on_update_acquireReEntryState, this));
         
         tAfsm_.set_entry_function(acquirePauseState_, boost::bind(&targetAquireFSM::on_enter_acquirePauseState, this));
         tAfsm_.set_update_function(acquirePauseState_, boost::bind(&targetAquireFSM::on_update_acquirePauseState, this));

         tAfsm_.set_entry_function(acquireDoneState_, boost::bind(&targetAquireFSM::on_enter_state_done, this));
         tAfsm_.set_update_function(acquireDoneState_, boost::bind(&targetAquireFSM::on_update_state_done, this));
      }

   public:
      targetAquireFSM()
      {
         camType_ = DIGCAM;
         cam_ = ZOOM_DIGCAM;
         firstTarget_ = true;
         homeTarget_ = false;
         camCommand_ = "capture";
         servoNumber_ = DIGCAM_PAN;
         pan_ = 0;
         tilt_ = 0;
         setupFSM();
      }

      int getCaptureImageState() { return captureImageState_; }
      int getMoveCameraState() { return moveCameraState_; }
      int getAcquireDoneState() { return acquireDoneState_; }
      int current_state() { return tAfsm_.current_state(); }
      void update() { tAfsm_.update(); }
      void set_state(int value) { tAfsm_.set_state(value); }
      void set_camType(int value) { camType_ = value; }
      void set_cam(int value) { cam_ = value; }
      void set_camCommand(string value) { camCommand_ = value; }
      void set_firstTarget(int value) { firstTarget_ = value; }
      void set_homeTarget(int value) { homeTarget_ = value; }
      void set_servoNumber(int value) { servoNumber_ = value; }
      void set_pan(int value) { pan_ = value; }
      void set_tilt(int value) { tilt_ = value; }
   
};

targetAquireFSM tAF_;

bool askUser()
{
	string input = "";
	int userValue = 1;
   cout << "Enter 1 to move on, 0 to retry: " << endl;
   getline(cin, input);

   // This code converts from string to number safely.
   stringstream myStream(input);
   if (myStream >> userValue)
   {
   	if (userValue == 0) return false;
   	return true;
   }	
   cout << "Failed to get a user value, returning true to move on" << endl;
   return true;
}
   
bool getUserInput()
{
   string input = "";
   cout << "Please enter the platform number we are starting on: " << endl << endl;
   getline(cin, input);

   // This code converts from string to number safely.
   stringstream myStream(input);
   if (myStream >> platformNumber_)
   {
      if (platformNumber_ == 1 || platformNumber_ == 2 || platformNumber_ == 3) 
         cout << "We are one platform number " << platformNumber_ << endl << endl;
      else
      {
         cout << "Invalid platform number, please try again" << endl;
         return false;
      }
   }
   else
   {
      cout << "Failed to get an input platform number" << endl;
      return false;
   }
   

   phase1_ = true;
   int compNumber;
   cout << "Please enter the competition phase (1 or 2): " << endl << endl;
   getline(cin, input);
   stringstream myStream1(input);
   if (myStream1 >> compNumber)
   {
      if (compNumber == 1) phase1_ = true;
      else if (compNumber == 2) phase1_ = false;   // we are in phase 2
      else
      {
         cout << "Invalid competition phase number, please try again" << endl;
         return false;
      }
      cout << "We are in competition phase " << compNumber << endl;
      return true;
   }
   cout << "Failed to get an input competition phase number" << endl;
   return false;
}

void imageCapture(string command, int camNum, int camType, bool writeFile)
{
  if (camType == WEBCAM)
   {
      webcam_command_.command = command;
      webcam_command_.camera_number = camNum;
      if (writeFile) webcam_command_.write_file = true;
      else webcam_command_.write_file = false;
      webcam_pub_.publish(webcam_command_);
   }
   else if (camType == DIGCAM)
   {
      digcam_command_.command = command;
      digcam_command_.camera_number = camNum;
      if (writeFile) digcam_command_.write_file = true;
      else digcam_command_.write_file = false;
      digcam_pub_.publish(digcam_command_); 
   }
}     

void setZoom(int camNum, float zoom)
{
   digcam_command_.command = "setZoom";
   digcam_command_.camera_number = camNum;
   digcam_command_.zoom = zoom;
   digcam_pub_.publish(digcam_command_); 
}  

void testCameras()
{ 
   cout << "testing cameras" << endl;
   // the digital cams:
   
   cout << "capturing image on zoom digcam..." << endl;
   imageCapture("capture", ZOOM_DIGCAM, DIGCAM);
   while (!askUser())
   {
   	cout << "capturing another image on zoom digcam..." << endl;
   	imageCapture("capture", ZOOM_DIGCAM, DIGCAM);
   }
   
   cout << "capturing image on right digcam..." << endl;
   imageCapture("capture", DIGCAM, DIGCAM);
   while (!askUser()) 
   {
   	cout << "capturing another image on right digcam..." << endl;
   	imageCapture("capture", DIGCAM, DIGCAM);
   }
   
   // the webcams:
   cout << "capturing image on front webcam..." << endl;
   imageCapture("capture", FRONT_WEBCAM + LAPTOP_HAS_BUILTIN_WEBCAM, WEBCAM);
   while (!askUser())
   {
   	cout << "capturing another image on front webcam..." << endl;
   	imageCapture("capture", FRONT_WEBCAM + LAPTOP_HAS_BUILTIN_WEBCAM, WEBCAM);
   }
   
   /*
   cout << "capturing image on rear webcam..." << endl;
   imageCapture("capture", REAR_WEBCAM + LAPTOP_HAS_BUILTIN_WEBCAM, WEBCAM); 
   while (!askUser())
   {
   	cout << "capturing another image on rearcam..." << endl;
   	imageCapture("capture", REAR_WEBCAM + LAPTOP_HAS_BUILTIN_WEBCAM, WEBCAM);
   }
   */
   
   cout << "setting zoom on zoom digital camera... " << endl;
   setZoom(ZOOM_DIGCAM, 7.);
   while (!askUser())
   {
   	cout << "setting zoom again on zoom digital camera... " << endl;
   	setZoom(ZOOM_DIGCAM, 7.);
   }
   
   cout << "setting zoom on right digital camera... " << endl;
   setZoom(DIGCAM, 7.);	// change to 10 *****************************************
   while (!askUser())
   {
   	cout << "setting zoom again on right digital camera... " << endl;
   	setZoom(DIGCAM, 7.);// change to 10 *****************************************
   }	
   
   cout << "capturing image on zoom digcam..." << endl;
   imageCapture("capture", ZOOM_DIGCAM, DIGCAM);
   while (!askUser())
   {
   	cout << "capturing another image on zoom digcam..." << endl;
   	imageCapture("capture", ZOOM_DIGCAM, DIGCAM);
   }
   
   cout << "capturing image on right digcam..." << endl;
   imageCapture("capture", DIGCAM, DIGCAM);
   while (!askUser()) 
   {
   	cout << "capturing another image on right digcam..." << endl;
   	imageCapture("capture", DIGCAM, DIGCAM);
   }
}  

void pauseCallback(const std_msgs::Int32::ConstPtr& msg)
{
   bool static outputSent1 = false, outputSent2 = false;	// dont want a million couts
   if (msg->data == 0) 
   {
   	if (!outputSent2) cout << "pause released" << endl;
   	outputSent1 = false;
   	outputSent2 = true;
   	pauseCommanded_ = false;
  	}
   else
   {
   	if (!outputSent1) cout << "pause commanded" << endl;
   	outputSent1 = true;
   	outputSent2 = false;
   	pauseCommanded_ = true;
   }   	
}

void moveCompleteCallback(const std_msgs::String::ConstPtr& msg)
{
   movementResult_ = msg->data.c_str();
   movementComplete_ = true;
}
  
void homeCenterCallback(const outdoor_bot::NavTargets_msg::ConstPtr &msg)
{
   centerX_ = msg->centerX;
   centerY_ = msg->centerY;
   totalX_ = msg->totalX;
   homeCameraRange_ = msg->range;
   cameraType_ = msg->cameraType;
   homeCenterUpdated_ = true;
   if (homeCameraRange_ > 0)
   {
   	cout << "NavTargets data: camType, center, homeCameraRange: " << cameraType_ << ", ("
   		<< centerX_ << ", " << centerY_ << "), " << homeCameraRange_ << endl;
   }
   else cout << "NavTargets returned no home target" << endl;
}

void targetCenterCallback(const outdoor_bot::mainTargets_msg::ConstPtr &msg)
{
   centerX_ = msg->centerX;
   centerY_ = msg->centerY;
   totalX_ = msg->totalX;
   targetRange_ = msg->range;
   cameraType_ = msg->cameraType;
   targetCenterUpdated_ = true;
   ROS_INFO("target center updated in control: center, range = (%d, %d), %f", centerX_, centerY_, targetRange_);
   cout << "target center callback received data" << endl;  
}

void targetImageReceivedCallback(const outdoor_bot::mainTargets_imageReceived_msg msg)
{
   int cameraType = msg.cameraType;
   if (cameraType == DIGCAM) 
   {
   	newMainTargetDigcamImageReceived_ = true;
   	cout << "digcam image was received by mainTargets" << endl;
   }
   else if (cameraType == WEBCAM) 
   {
   	newMainTargetWebcamImageReceived_ = true;
   	cout << "webcam image was received by mainTargets" << endl;
   }
   else if (cameraType == HOMECAM)
   {
   	newNavTargetImageReceived_ = true;
   	cout << "image was received by NavTargets" << endl;
   }
}

/*
void imageSentCallback(const std_msgs::String::ConstPtr& msg)
{
   ROS_INFO("image filename received");
   string camFilename = msg->data.c_str();
   if (!camFilename.compare("memory")) newMainTargetDigcamImageReceived_ = true;
   cout << "image was sent by digcams" << endl;
}

bool CallNavTargetsService(string filename)
{
   outdoor_bot::NavTargets_service::Request req;
   outdoor_bot::NavTargets_service::Response resp;
   req.image_filename = filename;
   bool success = NavTargets_client_.call(req,resp);
   if (success)
   {
      centerX_ = resp.centerX;
      centerY_ = resp.centerY;
      range_ = resp.range;
      totalX_ = resp.totalX;
      homeCenterUpdated_ = true;
      return true; // true here just means the service call succeeded, not that a target was found
   }
   ROS_WARN("NavTargets service call failed");
   return false; // false if service call failed
}

bool CallMainTargetsService(string filename, bool firstTarget = false)
{
   outdoor_bot::mainTargets_service::Request req;
   outdoor_bot::mainTargets_service::Response resp;
   req.image_filename = filename;
   req.firstTarget = firstTarget;
   req.approxRange = approxRangeToTarget_;
   bool success = mainTargets_client_.call(req,resp);
   
   if (success)
   {
      newMainTargetDigcamImageReceived_ = resp.newDigcamImageReceived;
      newMainTargetWebcamImageReceived_ = resp.newWebcamImageReceived;
      if ((!filename.compare("memory")) && (!(newMainTargetDigcamImageReceived_ || newMainTargetWebcamImageReceived_))) return true;
      centerX_ = resp.centerX;
      centerY_ = resp.centerY;
      float rangeSquared = resp.rangeSquared;
      if (rangeSquared >= 0) range_ = sqrt(rangeSquared);
      else
      {
         range_ = 20;
         rangeUnknown_ = true;
      }
      totalX_ = resp.totalX;
      return true; // true here just means the service call succeeded, not that a target was found
   }
   ROS_WARN("mainTargets service call failed");
   return false; // false if service call failed
}


bool CallMainTargetsCheckImageReceivedService()
{
   outdoor_bot::mainTargets_service::Request req;
   outdoor_bot::mainTargets_service::Response resp;  
   bool success = mainTargetsCheckImage_client_.call(req,resp);
   if (!success) return false;
   newMainTargetDigcamImageReceived_ = resp.newDigcamImageReceived;
   newMainTargetWebcamImageReceived_ = resp.newWebcamImageReceived;
   return true;
}

bool CallDigcamsService(int camNum, string command, string filename, float zoom, bool writeFile = false)
{
   outdoor_bot::digcams_service::Request req;
   outdoor_bot::digcams_service::Response resp;
   req.filename = filename;
   req.command = command;
   req.camera_number = camNum;
   req.zoom = zoom;
   req.write_file = writeFile;
   bool success = digcams_client_.call(req,resp);
   if (success)
   {
      retCapToMemory_ = resp.captureResult;
      zoomResult_ = resp.zoomResult;
      writeFileResult_ = resp.writeFileResult;
      return true; // true here just means the service call succeeded, not that a target was found
   }
   ROS_WARN("mainTargets service call failed");
   return false; // false if service call failed
}
*/
bool callRadarService()
{
   outdoor_bot::radar_service::Request req;
   outdoor_bot::radar_service::Response resp;
   cout << "Calling radar service" << endl;
   req.enableRadarData = true;
   bool success = radar_client_.call(req,resp);
   if (success)
   { 
      distanceToHome_[LEFT_RADAR_INDEX] = resp.distanceToHomeLeft;
      distanceToHome_[RIGHT_RADAR_INDEX] = resp.distanceToHomeRight;
      distanceToHome_[CENTER_RADAR_INDEX] = resp.distanceToHomeCenter;
      return true;
   }
   return false;
}

bool callSetPoseService(double x, double y, double yaw, bool setHome)
{
   outdoor_bot::setPose_service::Request req;
   outdoor_bot::setPose_service::Response resp;
   req.setHome = setHome;
   req.x = x;
   req.y = y;
   req.yaw = yaw; 
   if (setPose_client_.call(req,resp)) return true;
   return false;
}

/*
bool callEncodersService()
{
	outdoor_bot::encoders_service::Request req;
   outdoor_bot::encoders_service::Response resp;
   cout << "Calling encoders service" << endl;
   bool success = radar_client_.call(req,resp);
   if (success)
   {
   	encoderPickerUpper_ = resp.encoderPickerUpper;
   	encoderDropBar_ = resp.encoderDropBar;
   	encoderBinShade_ = resp.encoderBinShade;
   	return true;
   }
   return false;
}
*/
void on_enter_BootupState()
{
   approxRangeToTarget_ = 71.; // we start at about 70m from the first target
   targetRange_ = 71.;
   homeCameraRange_ = 10.;
   range_ = 71.;
   rangeUnknown_ = false;
   retCapToMemory_ = -1;
   centerX_ = -1; // indicator for when target centers are updated
   offsetX_ = 0.;
   zoomResult_ = false;
   writeFileResult_ = false;
   newMainTargetDigcamImageReceived_ = false;
   newMainTargetWebcamImageReceived_ = false;
   newNavTargetImageReceived_ = false;
   targetCenterUpdated_ = false;
   homeCenterUpdated_ = false;
   moving_ = false;
   turning_ = false;
   movementComplete_ = false;
   movementResult_ = "";
   triedWebcamAlready_ = false;
   triedZoomDigcamAlready_ = false;
   recoverTurnedAlready_ = false;
   firstMoveToFirstTarget_ = false;
   for (int i=0; i < NUM_RADARS; i++)
   {
      distanceToHome_[i] = 0.;
      deltaDistanceToHome_[i] = 0.;
      velocityFromHome_[i] = 0.;
   }

   platformXPose_[0] = plat1_X;
   platformXPose_[1] = plat2_X;
   platformXPose_[2] = plat3_X;
   platformYPose_[0] = plat1_Y;
   platformYPose_[1] = plat2_Y;
   platformYPose_[2] = plat3_Y;
   platformYawPose_[0] = plat1_Yaw;
   platformYawPose_[1] = plat2_Yaw;
   platformYawPose_[2] = plat3_Yaw;
   targetXPose_ = target1_X;
   targetYPose_ = target1_Y;
   
   //encoderPickerUpper_ = 0;
   //encoderDropBar_ = 0;
   //encoderBinShade_= 0;
   
   cout << "finished enter bootupState" << endl;

}

int on_update_BootupState()
{
   cout << "now in update bootupState" << endl;
   // give some time for everyone to setup and get started, then ask what platform we are assigned
   cout << "Do you want to move on to autonomous ops or go through bootstate stuff?" << endl;
   if (askUser()) return CheckLinedUpState_;;
   if (getUserInput())
   {
		   platPoseX_ = platformXPose_[platformNumber_ - 1];
		   platPoseY_ = platformYPose_[platformNumber_ - 1];
		   platPoseYaw_ = platformYawPose_[platformNumber_ - 1];
			if (phase1_) totalTime_ = PHASE_ONE_TIME_LIMIT;
			else totalTime_ = PHASE_TWO_TIME_LIMIT;				
	}
   else return BootupState_;
   
   if (!callSetPoseService(platPoseX_, platPoseY_, platPoseYaw_, true)) // set home pose in robotPose_node
   {
      cout << "failed to set home pose, need to retry BootupState?" << endl;
      if (!askUser()) return BootupState_;
   }
   if (!callSetPoseService(platPoseX_, platPoseY_, platPoseYaw_, false)) // and place us there
   {
      cout << "failed to place us at home pose on the map, need to retry BootupState?" << endl;
      if (!askUser()) return BootupState_;
   }
	
   // get radar ranges
   if (callRadarService())
   { 
		cout << "left radar distance to home = " << distanceToHome_[LEFT_RADAR_INDEX] << endl;
		cout << "center radar distance to home = " << distanceToHome_[CENTER_RADAR_INDEX] << endl;
		cout << "right radar distance to home = " << distanceToHome_[RIGHT_RADAR_INDEX] << endl;
	}
   else
   {
      cout << "radar service failed, do you want to retry bootstate?" << endl;
      if (!askUser()) return BootupState_;
   }
   
   // start by setting zooms and taking a photo from each camera
   testCameras();
   
   // now get the target centered, or at least the robot pointed in the right direction

 	string input = "";
 	int inputNum = 0;
   while (inputNum == 0)
   {
   	imageCapture("capture", ZOOM_DIGCAM, DIGCAM);	
   	cout << "check image and center the robot on the target.  Do you want to move on or retry?" << endl;
   	if (askUser()) inputNum = 1;
	}
   
   cout << "bootupState is finished.  Do you want to move on or retry?" << endl;
   if (!askUser()) return BootupState_;
   startTime_ = ros::Time::now().toSec();
   secondsRemaining_ = totalTime_ - (ros::Time::now().toSec() - startTime_);
   if (phase1_) return CheckLinedUpState_;
   else
   {
   	previousState_ = CheckFirstTargetState_;
   	return PauseState_;
   }
}

void on_exit_BootupState()
{
	newMainTargetDigcamImageReceived_ = false;
   newMainTargetWebcamImageReceived_ = false;
   newNavTargetImageReceived_ = false;
   targetCenterUpdated_ = false;
   homeCenterUpdated_ = false;
   moving_ = false;
   turning_ = false;
   movementComplete_ = false;
   movementResult_ = "";
   centerX_ = -1; // indicator for when target centers are updated
}

void on_enter_CheckLinedUpState()
{
   // start by capturing an image using fsm
   tAF_.set_camType(DIGCAM);
   tAF_.set_cam(ZOOM_DIGCAM);
   tAF_.set_camCommand("capture");
   tAF_.set_firstTarget(true);
   tAF_.set_homeTarget(false);
   tAF_.set_state(tAF_.getCaptureImageState());
}

int on_update_CheckLinedUpState()
{    
   tAF_.update();
   if (tAF_.current_state() != tAF_.getAcquireDoneState()) return CheckLinedUpState_;  // check to see if the image got analyzed 
          
   bool targetFound = false;
   if (centerX_ > 0)
   {
      cout << "possible target found: x, y, range = " << centerX_ << ", " << centerY_ << ", " << targetRange_ << endl;
      targetFound= true;
   }
   else
   {
      cout << "no target found yet" << endl;
   }  
   
   cout << "Do you want to move on to autonomous ops or retry LineUp?" << endl;
   if (askUser())
   {  	
     	//return MoveToFirstTargetState_; //********************  use this for real ops
     	// comment out the section below for real  ops *************************************
     	if (targetFound) return MoveToFirstTargetState_;
   	else
   	{
   		centerX_ = -1;
   		return SearchForFirstTargetState_;  
   	}
   }
   
   // decided to try again, so we will capture an image using fsm
   tAF_.set_camType(DIGCAM);
   tAF_.set_cam(ZOOM_DIGCAM);
   tAF_.set_camCommand("capture");
   tAF_.set_firstTarget(true);
   tAF_.set_homeTarget(false);
   tAF_.set_state(tAF_.getCaptureImageState());
   return CheckLinedUpState_;
}

void on_enter_CheckFirstTargetState()
{
   // start by capturing an image using fsm
   centerX_ = -1;
   if (!triedZoomDigcamAlready_)
   {
		tAF_.set_camType(DIGCAM);
		tAF_.set_cam(ZOOM_DIGCAM);
		tAF_.set_camCommand("capture");
		tAF_.set_firstTarget(true);
		tAF_.set_homeTarget(false);
		tAF_.set_state(tAF_.getCaptureImageState());
		
   }
   else if (approxRangeToTarget_ > 15.)  // too far for webcam
   {
   	tAF_.set_camType(DIGCAM);
   	tAF_.set_cam(DIGCAM);
   	lastCamType_ = DIGCAM;
   }
   else
   {
   	tAF_.set_camType(WEBCAM);
   	tAF_.set_cam(FRONT_WEBCAM);
   	lastCamType_ = WEBCAM;  
   } 	
   tAF_.set_camCommand("capture");
   tAF_.set_homeTarget(false);
   tAF_.set_firstTarget(true);
   tAF_.set_state(tAF_.getCaptureImageState()); 
}

int on_update_CheckFirstTargetState()
{    
   //cout << "acquire target state before update = " << tAF_.current_state() << endl;
   tAF_.update();
   //cout << "current acquire target state = " << tAF_.current_state() << endl;
   if (tAF_.current_state() != tAF_.getAcquireDoneState()) return CheckFirstTargetState_;  // check to see if the image got analyzed 
          
   if (centerX_ > 0)
   {
      cout << "possible target found: x, y, range = " << centerX_ << ", " << centerY_ << ", " << targetRange_ << endl;
      triedZoomDigcamAlready_ = false;
      searchCounter_ = 0;
      tAF_.set_pan(0);
		tAF_.set_tilt(0);
		tAF_.set_state(tAF_.getMoveCameraState());
      return MoveToFirstTargetState_;
   }
   else
   {
      cout << "no target found yet" << endl;
      return SearchForFirstTargetState_;
   }  
}

// we did not see the target in the camera image, so we need to move the camera and look again
void on_enter_SearchForFirstTargetState()
{
      deltaPan_ = -deltaPan_; // switch directions each time
		currentPan_ += deltaPan_ * searchCounter_;
		if (abs(currentPan_) > PAN_CAMERA_SEARCH_MAX)
		{
		   tAF_.set_pan(0);
		   tAF_.set_tilt(0);
		   tAF_.set_state(tAF_.getMoveCameraState());         		   
		   while (tAF_.current_state() != tAF_.getAcquireDoneState()) 
		   {
			   tAF_.update();
   			ros::spinOnce();
		   }
		   
		   if (triedZoomDigcamAlready_) return; // no camera sees the target, so center the camera and then move ahead a bit
		   
		   // move on to the next camera
		   triedZoomDigcamAlready_ = true;
		   searchCounter_ = 0;
		   currentPan_ = 0;
		}
		
		searchCounter_++;
		tAF_.set_pan(currentPan_);
		tAF_.set_tilt(0);
		
		if (!triedZoomDigcamAlready_)
		{		
			tAF_.set_servoNumber(ZOOM_DIGCAM_PAN);
			tAF_.set_camType(DIGCAM);
			tAF_.set_cam(ZOOM_DIGCAM);
		}
		
		else if (approxRangeToTarget_ > 15.)	// too far for webcam
		{
			tAF_.set_servoNumber(DIGCAM_PAN);
			tAF_.set_camType(DIGCAM);
			tAF_.set_cam(DIGCAM);	
		}	
		else
		{
			tAF_.set_servoNumber(FRONT_WEBCAM_PAN);
			tAF_.set_camType(WEBCAM);
			tAF_.set_cam(WEBCAM);
		}
		tAF_.set_state(tAF_.getMoveCameraState());		
}

int on_update_SearchForFirstTargetState()
{
   if (abs(currentPan_) > PAN_CAMERA_SEARCH_MAX)
   {
      currentPan_ = 0;
      return MoveToFirstTargetState_; // can't find the target, so move ahead a bit 
   }
   tAF_.update();
   if (tAF_.current_state() != tAF_.getAcquireDoneState()) return SearchForFirstTargetState_; // waiting for servo delay
   return CheckFirstTargetState_; // go see if there is a target in this camera direction
}

void on_enter_MoveToFirstTargetState()
{
   triedZoomDigcamAlready_ = false;
   searchCounter_ = 0;
   if (pauseCommanded_) return;
   outdoor_bot::movement_msg msg;
   
   if (centerX_ < 0) // we don't have a target in view yet, so we will just move forward a bit
   {    
      // if this is the very first move, it should be a long one
      if (firstMoveToFirstTarget_)
      {
      	msg.distance = approxRangeToTarget_ - FIRST_MOVE_REMAINING_DISTANCE;
      }
      else msg.distance = INCREMENTAL_MOVE_TO_TARGET;
      msg.command = "move";      
      msg.angle = 0.;
 //     movement_pub_.publish(msg);
 //     movementComplete_ = false;
      return;
   }
   
   // we found a target	
	// center the target
	offsetX_ = ((double) ((totalX_ / 2) - centerX_)) / ((double) totalX_);  // fraction that the image is off-center
	cout << "totalX_, centerX_, center offset ratio = " << totalX_ << ", " << centerX_ << ", " << offsetX_ << endl;
	centerX_ = totalX_/ 2; // reset centerX, assuming we turn correctly.
	if (lastCamType_ == WEBCAM) offsetX_ *= WEBCAM_FOV;
	else offsetX_ *= DIGCAM_FOV;
	cout << "center offset degrees = " << offsetX_ << endl;
	double servoOffset = currentPan_;
	currentPan_ = 0.;	// the servos were centered earlier, but we waited to reset currentPan_ so we could use it here
	cout << "servo offset degrees = " << servoOffset << endl;
	//double offsetY = ((double) (totalY_ - centerY_)) / ((double) totalY_);
	offsetX_ += servoOffset;
	cout << "total offset degrees = " << offsetX_ << endl;
	/*
	if (fabs(offsetX_) > 10.)
	{
		// send turn command to center the target
   	msg.command = "turn";
   	msg.angle = offsetX_;
   	msg.distance = 0.;
			// positive offset means turn to the left, negative to the right.
		movement_pub_.publish(msg);
      turning_ = true;
      movementComplete_ = false;
		ROS_INFO("centering target");
		return MoveToFirstTargetState_;
	}
	*/
	
	
	
	      movementComplete_ = true;	// remove this line ******************************************************
	      
	      
	      

}

int on_update_MoveToFirstTargetState()
{
   if (pauseCommanded_)
   {
      previousState_ = MoveToFirstTargetState_;
      return PauseState_;
   }

   // check to see if we have arrived at the new pose   
   if (!movementComplete_)
   {
   	ros::spinOnce();
   	return MoveToFirstTargetState_;  // move has not completed yet
   } 
   ROS_INFO("move completed in MoveToFirstTargetState");
   
   if (centerX_ < 0)
   {
      if (movementResult_.find("done") != string::npos)
      {
      	if (firstMoveToFirstTarget_)
      	{
      		totalMoveToFirstTarget_ += approxRangeToTarget_ - FIRST_MOVE_REMAINING_DISTANCE;
      		approxRangeToTarget_ = FIRST_MOVE_REMAINING_DISTANCE;
      		firstMoveToFirstTarget_ = false;
      	}
      	else
      	{
      		totalMoveToFirstTarget_ += INCREMENTAL_MOVE_TO_TARGET;	
      		if (approxRangeToTarget_ > INCREMENTAL_MOVE_TO_TARGET) approxRangeToTarget_ -= INCREMENTAL_MOVE_TO_TARGET;	
      		else approxRangeToTarget_ = 0.;
      	}
      	return CheckFirstTargetState_; // move succeeded, see if we can image the target
      }
      else
      {
         firstMoveToFirstTarget_ = false;
         previousState_ = MoveToFirstTargetState_; // come back to here when we have recovered movement
         return MoveToRecoverState_;   // the move failed for some reason, so we have to see what's up
      }
   }

   outdoor_bot::movement_msg msg;
	if (targetRange_ > 1.0)
	{
		range_ = targetRange_;
		approxRangeToTarget_ = targetRange_;
   }		
	else if (approxRangeToTarget_ > INCREMENTAL_MOVE_TO_TARGET) range_ = approxRangeToTarget_;
	else range_ = (2. * INCREMENTAL_MOVE_TO_TARGET) + 1.;	// if we just cannot figure out a range, then try this
   
   cout << "Moving to target, range = " << range_ << endl;  
   /* 
	if (range_ > 60.)	
	{
	   msg.command = "move";
	   msg.distance = 50.0;
	   approxRangeToTarget_ -= 50.;
	   ROS_INFO("moving forward 50m");
	   
	   //cout << "moving to target1 pose " << endl;
	   //msg.command = "pose";
	   //msg.poseX = target1_X;
	   //msg.poseY = target1_Y;
	   //msg.poseThetaDegrees = 0.; //plat1_Yaw * (3.14/180.);

	 }

	else if (range_ > 20.)
	{
	   msg.command = "move";
	   msg.distance = 10.;
	   approxRangeToTarget_ -= 10.;
	   ROS_INFO("moving forward 10m");
	}
		
	else if (range_ > 10.)
	{
	   msg.command = "move";
	   msg.distance = 5.0;
	   approxRangeToTarget_ -= 5.;
	   ROS_INFO("moving forward 5m");
	}
	
	else if (range_ > 5.)
	{
	   msg.command = "autoMove";
		msg.distance = 3000;	// mm
		approxRangeToTarget_ -= 3.;
		msg.speed = 200;   	// mm/sec or deg/sec 
	   ROS_INFO("auto moving forward 3m");
	}     
	
	else 
	
	if (range_ > 3.)
	{	   
	   msg.command = "autoMove";
		msg.distance = 1000;	// mm
		approxRangeToTarget_ -= 1.;
		msg.speed = 200;   	// mm/sec or deg/sec 
		ROS_INFO("auto moving forward 1m"); 
	}    
	   
	else if (range_ > 0.01)
	{
	   // final approach
	   cout << "moving to home spot, range_ = " << range_ << endl;
	   return PickupTargetState_;
	}
	
	else return CheckFirstTargetState_;
	
   
   msg.angle = 0.;
   movement_pub_.publish(msg);
   moving_ = true;
   movementComplete_ = false;   
   return MoveToFirstTargetState_;
   */
   
   cout << "In MoveToFirstTargetState, range and angle = " << range_ << ", " << offsetX_ << endl;
   return PauseState_;
}

/*
void on_enter_PickupFirstTargetState()
{
   if (pauseCommanded_) return;
}
int on_update_PickupFirstTargetState()
{
   if (pauseCommanded_)
   {
      previousState_ = PickupFirstTargetState_;
      return PauseState_;
   }
   // here we do final staging, which means line up with the target and drive straight to it and 
   // proceed with picking it up
   // center target, estimate range
   // lower front loader
   // move forward to target contact with loader
   // decide if drop bar is needed, if so, drop it and move forward to engage it
   // lift loader, move shade to next bin slot
   // verify that target is in loader
   // if drop bar deployed, move in reverse and wind it back up
   // dump target into bin
   // move forward 10m or perhaps farther to find flat ground and to get out of target area range
   // verify that target is in bin
   if (phase1_) return CheckHomeState_;
   return FindTargetState_;
}
*/
 
int on_update_FindTargetState()
{
   cout << "updating FindTargetState" << endl;
   // capture an image using fsm   
   tAF_.set_state(tAF_.getCaptureImageState());
   while (tAF_.current_state() != tAF_.getAcquireDoneState())
   {
      tAF_.update();
      ros::spinOnce();
      //if (!nh.ok()) }
   }
   //cout << "current state = " << tAF_.current_state() << endl;
   // sweep cameras
   // if target found, go to approach target
   // rotate 90 degrees and loop to sweep cameras
   // rotate 180 degrees and loop to sweep cameras

   // look for any target
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/middle_school_field/photos/red_target_close.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/middle_school_field/photos/pink_target_very_close_small.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/middle_school_field/photos/pink_target_closer.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/300mm_pink.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/200mm_scan12.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/300mm_yellow_close.JPG";
   //filename = "/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/300mm_yellow.JPG";
   /*
   for (int i=0; i < 75; i++)
   {
      centerX_ = -1;
      std::stringstream buffer;
      buffer << "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/webcam/webcam1_" << i << ".jpg";
      std::string filenm = buffer.str();
      CallMainTargetsService(filenm, false);
      cout << " filename: " << filenm << endl; 
      if (centerX_ > 0)
      {
         cout << "possible target found: x, y, range = " << centerX_ << ", " << centerY_ << ", " << range_ << endl;
      }
      else
      {
         cout << "no target found yet" << endl;
      }
      cout << endl;

   }
   */
   if (centerX_ > 0) return MoveToTargetState_;
   return FindTargetState_;
   //return BootupState_;
}

int on_update_MoveToTargetState()
{
   // we found a target, we now move the robot to center the target in the camera, take another image
   // and then move forward half the distance to the target until we are inside about 3 to 5 meters   
   return PickupTargetState_;
}

void on_enter_PickupTargetState()
{
	//callEncodersService();  // get initial locations
	outdoor_bot::movement_msg msg;   
   msg.command = "PDmotor";
   msg.PDmotorNumber = MOTOR_PICKER_UPPER;
   msg.speed = PICKER_UPPER_DOWN_PREP;
	movement_pub_.publish(msg); 
   movementComplete_ = false; 
   prepping_ = true;
   placing_ = false;
   driving_ = false;
   dropping_ = false;
   pushing_ = false;
   scooping_ = false;
   retrieving_ = false;	
}

int on_update_PickupTargetState()
{
   // here we do final staging, which means line up with the target and drive straight to it and 
   // proceed with picking it up
	outdoor_bot::movement_msg msg;   
   msg.command = "PDmotor";
     
   if (movementComplete_)
   {   	
   	if (prepping_) // finished putting scooper in place, now move forward
   	{
   		prepping_ = false;
   		driving_ = true;
   		movementComplete_ = false;
	      cout << "driving forward 1m to scoop target" << endl;
		   msg.command = "autoMove";
		   msg.distance = 1000;	// mm
		   msg.angle = 0;			// degrees
		   msg.speed = 200;   	// mm/sec or deg/sec  
   		movement_pub_.publish(msg);
   		return PickupTargetState_;
   	}
   	
   	if (driving_) // finished driving to target, now drop bar
   	{
   		driving_ = false;
   		dropping_ = true;
   		movementComplete_ = false;
   	   msg.PDmotorNumber = MOTOR_DROP_BAR;
   		msg.speed = DROP_BAR_DOWN;
   		movement_pub_.publish(msg); 
   		return PickupTargetState_;
   	}
   	
   	if (dropping_) // finished dropping bar, now place scooper for scooping
   	{
   		dropping_ = false;
   		placing_ = true;
			//movementComplete_ = false;
   	   msg.PDmotorNumber = MOTOR_PICKER_UPPER;
   		msg.speed = PICKER_UPPER_DOWN;
   		//movement_pub_.publish(msg); 
   		return PickupTargetState_;
   	}
   	
   	
   	if (placing_) // finished placing scooper, now push target onto scooper
   	{
   		placing_ = false;
   		pushing_ = true;
   		movementComplete_ = false;
	      cout << "driving forward 1m to push target into scoop" << endl;
		   msg.command = "autoMove";
		   msg.distance = 500;
		   msg.angle = 0; 
		   msg.speed = 200;  	  
   		movement_pub_.publish(msg);
   		return PickupTargetState_;
   	}
   	  	
   	if (pushing_) // finished placing scoop, now pick up target
   	{
   		pushing_ = false;
   		scooping_ = true;
   		movementComplete_ = false;
   	   msg.PDmotorNumber = MOTOR_PICKER_UPPER;
   		msg.speed = PICKER_UPPER_UP;
   		movement_pub_.publish(msg); 
   		return PickupTargetState_;
   	}
   	
   	if (scooping_)	// finished scooping, now retrieve the drop bar
   	{
   		scooping_ = false;
   		retrieving_ = true;
   		movementComplete_ = false;
   	   msg.PDmotorNumber = MOTOR_DROP_BAR;
   		msg.speed = DROP_BAR_UP;
   		movement_pub_.publish(msg); 
   		return PickupTargetState_;  
   	} 
   	retrieving_ = false;
   	return PauseState_;
   }			   
   return PickupTargetState_;
}


void on_enter_CheckHomeState()
{
   // start by capturing an image using fsm
   centerX_ = -1;
   if (!triedWebcamAlready_)
   {
   	tAF_.set_camType(WEBCAM);
   	tAF_.set_cam(FRONT_WEBCAM + LAPTOP_HAS_BUILTIN_WEBCAM);
   	triedWebcamAlready_ = true;
   	lastCamType_ = WEBCAM;
   }
   else
   {
   	tAF_.set_camType(DIGCAM);
   	tAF_.set_cam(DIGCAM);
   	lastCamType_ = DIGCAM;
   }
   tAF_.set_camCommand("cap_home");
   tAF_.set_homeTarget(true);
   tAF_.set_firstTarget(false);
   tAF_.set_state(tAF_.getCaptureImageState());
}

int on_update_CheckHomeState()
{  
   tAF_.update();
   
   if (tAF_.current_state() != tAF_.getAcquireDoneState()) return CheckHomeState_;  // check to see if the image got analyzed 
          
   if (centerX_ > 0)
   {
      cout << "possible target found: x, y, range = " << centerX_ << ", " << centerY_ << ", " << homeCameraRange_ << endl;     	
      triedWebcamAlready_ = false;
      searchCounter_ = 0;
      return HeadForHomeState_;
   }
   else
   {
      cout << "no home target found yet" << endl; 
      return HeadForHomeState_;
   }  
}

// we did not see the target in the camera image, so we need to move the camera and look again
void on_enter_SearchForHomeState()
{
   	deltaPan_ = -deltaPan_; // switch directions each time
		currentPan_ += deltaPan_ * searchCounter_;
		if (abs(currentPan_) > PAN_CAMERA_SEARCH_MAX) return; // can't find the target, so center the camera and then move ahead a bit
		searchCounter_++;
		tAF_.set_pan(currentPan_);
		tAF_.set_tilt(0);
		
		tAF_.set_servoNumber(DIGCAM_PAN);
		tAF_.set_camType(DIGCAM);
		tAF_.set_cam(DIGCAM);
		tAF_.set_state(tAF_.getMoveCameraState());
}

int on_update_SearchForHomeState()
{
   if (abs(currentPan_) > PAN_CAMERA_SEARCH_MAX)
   {
      return HeadForHomeState_; // can't find the target, so move ahead a bit 
   }
   tAF_.update();
   if (tAF_.current_state() != tAF_.getAcquireDoneState()) return SearchForHomeState_; // waiting for servo delay
   return CheckHomeState_; // go see if there is a target in this camera direction
}

void on_enter_HeadForHomeState()
{
   if (lastCamType_ == DIGCAM && abs(currentPan_) > 0.1) // need to put digcam servo back to the center
   {
	   searchCounter_ = 0;
	   tAF_.set_pan(0);
	   tAF_.set_tilt(0);
	   tAF_.set_state(tAF_.getMoveCameraState());	
	   if (abs(currentPan_) > PAN_CAMERA_SEARCH_MAX) currentPan_ = 0;	// we don't want to center on this servo pan setting   
	} 
	triedWebcamAlready_ = false;
	moving_ = false;
	turning_ = false;
	movementComplete_ = false;
}

int on_update_HeadForHomeState()
{
   if (movementComplete_)
   {
   	movementComplete_ = false;
   	if (moving_)
   	{
   		moving_ = false;
   		cout << "move finished, ranges: " << endl;
			if (callRadarService())
			{ 
				cout << "left radar distance to home = " << distanceToHome_[LEFT_RADAR_INDEX] << endl;
				cout << "center radar distance to home = " << distanceToHome_[CENTER_RADAR_INDEX] << endl;
				cout << "right radar distance to home = " << distanceToHome_[RIGHT_RADAR_INDEX] << endl; 
			}
			else cout << "call to radar service failed" << endl;
			return CheckHomeState_;
		}
    	turning_ = false;
    	cout << "turn finished" << endl;
   }
   if (moving_ || turning_) return HeadForHomeState_;
   
   //char moveCmd[32];
   // now move forward
   outdoor_bot::movement_msg msg;

/*
	if (centerX_ > 0)	
	{
   	// center the target and move forward
   	offsetX_ = ((double) ((totalX_ / 2) - centerX_)) / ((double) totalX_);  // fraction that the image is off-center
    	centerX_ = -1;   	// only want to turn once for each pass through HeadForHomeState
   	cout << "center offset ratio = " << offsetX_ << endl;
   	if (lastCamType_ == WEBCAM) offsetX_ *= WEBCAM_FOV;
   	else offsetX_ *= DIGCAM_FOV;
   	cout << "center offset degrees = " << offsetX_ << endl;
   	double servoOffset = currentPan_;
   	currentPan_ = 0.;	// the servos were centered earlier, but we waited to reset currentPan_ so we could use it here
   	cout << "servo offset degrees = " << servoOffset << endl;
   	//double offsetY = ((double) (totalY_ - centerY_)) / ((double) totalY_);
   	offsetX_ += servoOffset;
   	cout << "total offset degrees = " << offsetX_;
   	if (fabs(offsetX_) > 10.)
   	{
   		// send turn command to center the target
      	msg.command = "turn";
      	msg.angle = offsetX_;
      	msg.distance = 0.;
   			// positive offset means turn to the left, negative to the right.
   		movement_pub_.publish(msg);
         turning_ = true;
         movementComplete_ = false;
   		ROS_INFO("centering target");
   		return HeadForHomeState_;
   	}
	}
*/
	msg.command = "move"; 
	if (callRadarService())
	{ 
		cout << "left radar distance to home = " << distanceToHome_[LEFT_RADAR_INDEX] << endl;
		cout << "center radar distance to home = " << distanceToHome_[CENTER_RADAR_INDEX] << endl;
		cout << "right radar distance to home = " << distanceToHome_[RIGHT_RADAR_INDEX] << endl;

		if (distanceToHome_[CENTER_RADAR_INDEX] > 0.1) range_ = distanceToHome_[CENTER_RADAR_INDEX];
		else if (distanceToHome_[LEFT_RADAR_INDEX] > 0.1) range_ = distanceToHome_[LEFT_RADAR_INDEX];
		else if (distanceToHome_[RIGHT_RADAR_INDEX] > 0.1) range_ = distanceToHome_[RIGHT_RADAR_INDEX];
	}
	else 
	{
		cout << "call to radar service failed" << endl;    
		if (homeCameraRange_ > 1.0)  range_ = homeCameraRange_; // use optical range if radar fails
   	else 
   	{
   		cout << "unable to determine range in HeadForHome.  going back to CheckHomeState" << endl;
   		return CheckHomeState_;	// without any idea what the range is, we need to go back and check where we are
   	}
   }
   
   cout << "Heading for Home, range = " << range_ << endl;  
   
/*   if (range_ > 30.)
   {
      //move forward 10m and turn by the angle needed to center the target in X
      msg.command = "move";
      ROS_INFO("moving forward 15m");
   }
   else if (range_ > 15.)
   {
      //move forward 10m
      moveCmd[0] = 'n';
      moveCmd[1] = 0; // end with null character
     // msgMovement.data = cmd;
     // movement_pub_.publish(msgMotion);
      ROS_INFO("moving forward 10m");
   }
   else if (range_ > 10.)
   {
      //move forward 5 m
      ROS_INFO("moving forward 5m");
   }
   else
   */
    if (range_ > 12.)
   {
      //move forward 3 m
      
      cout << "moving forward 1m, range_ = " << range_ << endl;
      msg.command = "move";
      msg.distance = 1.0;
      msg.angle = 0.;
      
      /*
      cout << "moving to target1 pose " << endl;
      msg.command = "pose";
      msg.poseX = target1_X;
      msg.poseY = target1_Y;
      msg.poseThetaDegrees = 0.; //plat1_Yaw * (3.14/180.);
      */
      movement_pub_.publish(msg);
      moving_ = true;
      movementComplete_ = false;

      
   }
   else if (range_ > 0.01)
   {
      // final approach
      cout << "moving to home spot, range_ = " << range_ << endl;
      return MoveOntoPlatformState_;
   }
   
   return HeadForHomeState_;   
}

void on_enter_MoveOntoPlatformState()
{
	ROS_INFO("moving onto home platform");
   outdoor_bot::movement_msg msg;
   msg.command = "platform";
   //msg.distance = 0.0;
   //msg.angle = 0.;
   movement_pub_.publish(msg);
   moving_ = true;
   movementComplete_ = false;
}

int on_update_MoveOntoPlatformState()
{
   // drive onto the platform and then pause
   if (movementComplete_)
   {
   	moving_ = false;
   	movementComplete_ = false;
   	if (!movementResult_.compare("failed")) return HeadForHomeState_;
   	return AllDoneState_;
   }
   return MoveOntoPlatformState_;
}

void on_enter_AllDoneState()
{
	ROS_INFO("All done, going into pause mode");
   outdoor_bot::movement_msg msg;   
   msg.command = "all_done";
   msg.distance = 0.0;
   msg.angle = 0.;
   movement_pub_.publish(msg);
   moving_ = true;
   movementComplete_ = false;	
   previousState_ = AllDoneState_;	// so if we enter pause state via the telemetry switch, we come back to AllDoneState.
}

int on_update_AllDoneState()
{
   if (movementComplete_)
   {
   	moving_ = false;
   	movementComplete_ = false;
   }
   return AllDoneState_;
}

// make a movement to get unstuck or to reset the action server
void on_enter_MoveToRecoverState()
{
   // publish move command to back up 1 meter and then turn 10 degrees
   outdoor_bot::movement_msg msg;
   movementComplete_ = false;
   msg.command = "autoMove";
	msg.distance = 1000;	// mm
	msg.speed = -200;   	// mm/sec or deg/sec 
	ROS_INFO("auto moving backward 1m"); 
	movement_pub_.publish(msg);
	recoverTurnedAlready_ = false;	// we still have to turn
   return;
}
   
int on_update_MoveToRecoverState()
{
   if (!movementComplete_)
   {
   	ros::spinOnce();
   	return MoveToRecoverState_;
   }
   
   ROS_INFO("move or turn completed in MoveToRecover");
   if (recoverTurnedAlready_)
   {		   
	   if (movementResult_.find("turn_done") != string::npos)
	   {
	      MoveToRecoverCounter_ = 0;
	      TurnToRecoverCounter_ = 0;
	      return previousState_; // move succeeded
	   }
	   else
	   {
	      return MoveToRecoverFailedState_;   // the move failed for some reason, so we will try again
	   }
	 }
	 
	// we need to turn
	outdoor_bot::movement_msg msg;
	movement_pub_.publish(msg);
	movementComplete_ = false;
	msg.command = "autoMove";
	msg.distance = 0;	// mm
	msg.angle = 20;
	msg.speed = 20;   	// mm/sec or deg/sec 
	ROS_INFO("auto turning clockwise 20 degrees");
	recoverTurnedAlready_ = true;		 
   return MoveToRecoverState_;  // move has not completed yet
}

/* 
void on_enter_MoveToRecoverFailedState()
{
   if (TurnToRecoverCounter_ <= TURN_TO_RECOVER_MAX)
   {
      TurnToRecoverCounter_++;
      deltaRecoverAngle_ = -deltaRecoverAngle_; // try turning the other way
      recoverAngle_ += deltaRecoverAngle_ * MoveToRecoverCounter_;
   }
   else if (MoveToRecoverCounter_ <= MOVE_TO_RECOVER_MAX)
   {
      MoveToRecoverCounter_++;
      deltaRecoverDistance_ = -deltaRecoverDistance_; // try moving the other way
      recoverDistance_ += deltaRecoverDistance_ * MoveToRecoverCounter_;
   }
}
*/

int on_update_MoveToRecoverFailedState() // without an on_enter method, this allows us to rerun the on_enterMoveToRecovery
{
   return MoveToRecoverState_;
}

void on_enter_PauseState()
{
   // if the current state is a recovery state, then we must not change previousState_
   // otherwise, set previousState_ = currentState
   // also remember variables and other things needed to recover when motion is resumed
}
  
int on_update_PauseState()
{
   if (pauseCommanded_) return PauseState_;
   return previousState_;
}

 // allows a state to run its on_enter code.  Note that its on-exit code will run!! 
int on_update_ReEntryState()
{
   return previousState_;
}


void on_transition(int from_state, int to_state)
{
  ROS_INFO("AutoFSM: Transitioning from state %d (%s) to state %d (%s).",
	   from_state, fsm_.state_name(from_state).c_str(), to_state, fsm_.state_name(to_state).c_str());
}

void setupStates()
{
   BootupState_ = fsm_.add_state("BootupState");  // 0
   CheckLinedUpState_ = fsm_.add_state("CheckLinedUpState");  
   CheckFirstTargetState_ = fsm_.add_state("CheckFirstTargetState");  // 1
   SearchForFirstTargetState_ = fsm_.add_state("SearchForFirstTargetState");
   MoveToFirstTargetState_ = fsm_.add_state("MoveToFirstTargetState");
   FindTargetState_ = fsm_.add_state("FindTargetState");
   PickupTargetState_ = fsm_.add_state("PickupTargetState");
   MoveToTargetState_ = fsm_.add_state("MoveToTargetState");
   CheckHomeState_ = fsm_.add_state("CheckHomeState");
   SearchForHomeState_ = fsm_.add_state("SearchForHomeState");
   HeadForHomeState_ = fsm_.add_state("HeadForHomeState");
   MoveOntoPlatformState_ = fsm_.add_state("MoveOntoPlatformState");
   MoveToRecoverState_ = fsm_.add_state("MoveOntoRecoverState");
   MoveToRecoverFailedState_ = fsm_.add_state("MoveOntoRecoverFailedState");
   ReEntryState_ = fsm_.add_state("ReEntryState");
   PauseState_ = fsm_.add_state("PauseState");
   AllDoneState_ = fsm_.add_state("AllDoneState");

   fsm_.set_transition_function(on_transition);

   fsm_.set_entry_function(BootupState_, boost::bind(&on_enter_BootupState));
   fsm_.set_update_function(BootupState_, boost::bind(&on_update_BootupState));
   fsm_.set_exit_function(BootupState_, boost::bind(&on_exit_BootupState));
   
   fsm_.set_entry_function(CheckLinedUpState_, boost::bind(&on_enter_CheckLinedUpState));
   fsm_.set_update_function(CheckLinedUpState_, boost::bind(&on_update_CheckLinedUpState));
   //fsm_.set_exit_function(CheckLinedUpState_, boost::bind(&on_exit_CheckLinedUpState));

   fsm_.set_entry_function(CheckFirstTargetState_, boost::bind(&on_enter_CheckFirstTargetState));
   fsm_.set_update_function(CheckFirstTargetState_, boost::bind(&on_update_CheckFirstTargetState));
   //fsm_.set_exit_function(CheckFirstTargetState_, boost::bind(&on_exit_CheckFirstTargetState));

   fsm_.set_entry_function(SearchForFirstTargetState_, boost::bind(&on_enter_SearchForFirstTargetState));
   fsm_.set_update_function(SearchForFirstTargetState_, boost::bind(&on_update_SearchForFirstTargetState));
   //fsm_.set_exit_function(SearchForFirstTargetState_, boost::bind(&on_exit_SearchForFirstTargetState));

   fsm_.set_entry_function(MoveToFirstTargetState_, boost::bind(&on_enter_MoveToFirstTargetState));
   fsm_.set_update_function(MoveToFirstTargetState_, boost::bind(&on_update_MoveToFirstTargetState));
   //fsm_.set_exit_function(MoveToFirstTargetState_, boost::bind(&on_exit_MoveToFirstTargetState))

   //fsm_.set_entry_function(FindTargetState_, boost::bind(&on_enter_FindTargetState));
   fsm_.set_update_function(FindTargetState_, boost::bind(&on_update_FindTargetState));
   //fsm_.set_exit_function(FindTargetState_, boost::bind(&on_exit_FindTargetState));

   fsm_.set_entry_function(PickupTargetState_, boost::bind(&on_enter_PickupTargetState));
   fsm_.set_update_function(PickupTargetState_, boost::bind(&on_update_PickupTargetState));
   //fsm_.set_exit_function(PickupTargetState_, boost::bind(&on_exit_PickupTargetState));

   fsm_.set_entry_function(CheckHomeState_, boost::bind(&on_enter_CheckHomeState));
   fsm_.set_update_function(CheckHomeState_, boost::bind(&on_update_CheckHomeState));
   //fsm_.set_exit_function(CheckHomeState_, boost::bind(&on_exit_CheckHomeState));

   fsm_.set_entry_function(SearchForHomeState_, boost::bind(&on_enter_SearchForHomeState));
   fsm_.set_update_function(SearchForHomeState_, boost::bind(&on_update_SearchForHomeState));
   //fsm_.set_exit_function(SearchForHomeState_, boost::bind(&on_exit_SearchForHomeState));

   fsm_.set_entry_function(HeadForHomeState_, boost::bind(&on_enter_HeadForHomeState));
   fsm_.set_update_function(HeadForHomeState_, boost::bind(&on_update_HeadForHomeState));
   //fsm_.set_exit_function(HeadForHomeState_, boost::bind(&on_exit_HeadForHomeState));

   fsm_.set_entry_function(MoveOntoPlatformState_, boost::bind(&on_enter_MoveOntoPlatformState));
   fsm_.set_update_function(MoveOntoPlatformState_, boost::bind(&on_update_MoveOntoPlatformState));
   //fsm_.set_exit_function(MoveOntoPlatformState_, boost::bind(&on_exit_MoveOntoPlatformState));

   fsm_.set_entry_function(MoveToRecoverState_, boost::bind(&on_enter_MoveToRecoverState));
   fsm_.set_update_function(MoveToRecoverState_, boost::bind(&on_update_MoveToRecoverState));
   //fsm_.set_exit_function(PauseState_, boost::bind(&on_exit_MoveToRecoverState));

   //fsm_.set_entry_function(MoveToRecoverFailedState_, boost::bind(&on_enter_MoveToRecoverFailedState));
   fsm_.set_update_function(MoveToRecoverFailedState_, boost::bind(&on_update_MoveToRecoverFailedState));
   //fsm_.set_exit_function(MoveToRecoverFailedState_, boost::bind(&on_exit_MoveToRecoverFailedState));

   //fsm_.set_entry_function(ReEntryState_, boost::bind(&on_enter_ReEntryState));
   fsm_.set_update_function(ReEntryState_, boost::bind(&on_update_ReEntryState));
   //fsm_.set_exit_function(ReEntryState_, boost::bind(&on_exit_ReEntryState));

   fsm_.set_update_function(ReEntryState_, boost::bind(&on_update_ReEntryState));
      
   fsm_.set_entry_function(PauseState_, boost::bind(&on_enter_PauseState));
   fsm_.set_update_function(PauseState_, boost::bind(&on_update_PauseState));
   //fsm_.set_exit_function(PauseState_, boost::bind(&on_exit_PauseState));

   fsm_.set_entry_function(AllDoneState_, boost::bind(&on_enter_AllDoneState));
   fsm_.set_update_function(AllDoneState_, boost::bind(&on_update_AllDoneState));
   //fsm_.set_exit_function(AllDoneState_, boost::bind(&on_exit_AllDoneState));

}


            

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "autonomous_node");
   ros::NodeHandle nh;

   //digcams_client_ = nh.serviceClient<outdoor_bot::digcams_service>("digcams_service");
   radar_client_ = nh.serviceClient<outdoor_bot::radar_service>("radar_service");
   //encoders_client_ = nh.serviceClient<outdoor_bot::encoders_service>("encoders_service");
   setPose_client_ = nh.serviceClient<outdoor_bot::setPose_service>("setPose_service");
   NavTargets_client_ = nh.serviceClient<outdoor_bot::NavTargets_service>("NavTargetsService");
   //mainTargets_client_ = nh.serviceClient<outdoor_bot::mainTargets_service>("mainTargetsService");
   webcam_pub_ = nh.advertise<outdoor_bot::webcams_custom>("webcam_cmd", 25);
   digcam_pub_ = nh.advertise<outdoor_bot::digcams_custom>("digcam_cmd", 25);
   servo_pub_ = nh.advertise<outdoor_bot::servo_msg>("servo_cmd", 5);
   //pmotor_pub_ = nh.advertise<outdoor_bot::pmotor_msg>("pmotor_cmd", 5);
   movement_pub_ = nh.advertise<outdoor_bot::movement_msg>("movement_command", 2);
   mainTargetsCommand_pub_ = nh.advertise<outdoor_bot::mainTargetsCommand_msg>("mainTargets_cmd", 25);
   NavTargetsCommand_pub_ = nh.advertise<std_msgs::String>("NavTargets_cmd", 5);
   target_center_sub_ = nh.subscribe("target_center", 10, targetCenterCallback);
   home_center_sub_ = nh.subscribe("Home_target_center", 10, homeCenterCallback); 
   move_complete_sub_ = nh.subscribe("movement_complete", 4, moveCompleteCallback);
   pause_sub_ = nh.subscribe("pause_state", 4, pauseCallback);
   //image_sent_ = nh.subscribe("digcam_file", 50, imageSentCallback);
   targetImageReceived_sub_ = nh.subscribe("target_image_received", 5, targetImageReceivedCallback);

   pauseCommanded_ = false;
  
   setupStates();

   // first step is to see if we have a target to track
   // if so, we will track it and then try to pick it up
   // we will keep track of the number of targets we have and also the time
   // we will also listen for command inputs
   // if we have the correct number of targets and time is getting short
   // we will head for home and dock
   // all along we need to monitor for pause command

   // we are ready to enter bootup state   
   previousState_ = BootupState_;
   fsm_.set_state(BootupState_);
   
   // now we keep updating and nicely move through the states

   while (nh.ok())
   {
      ros::spinOnce();
      struct timespec ts;
      ts.tv_sec = 0;
      ts.tv_nsec = 10000000;
      nanosleep(&ts, NULL); // update every 10 ms
      //cout << "state before update = " << fsm_.current_state() << endl;
      fsm_.update();
      //cout << "current state = " << fsm_.current_state() << endl;
   }

   cout << "all done" << endl;

   ros::spin();
   return EXIT_SUCCESS;
}

