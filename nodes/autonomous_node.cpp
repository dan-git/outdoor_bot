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
#include "outdoor_bot/radar_msg.h"
#include "outdoor_bot/dirAnt_msg.h"
#include "outdoor_bot/dirAnt_service.h"
#include "outdoor_bot/accelerometers_service.h"
#include "outdoor_bot/setPose_service.h"
//#include "outdoor_bot/encoders_service.h"
#include "outdoor_bot/servo_msg.h"
//#include "outdoor_bot/pmotor_msg.h"
#include "outdoor_bot/movement_msg.h"
#include "FBFSM/FBFSM.h"
#include "navigation/ObstacleDetector.h"
#include "navigation/WallFollower.h"
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


#define PLAT1_X -6.07 //2269.5 pixels x = -6.075176, y = 39.607155, yaw = -0.953306
#define PLAT1_Y 39.61 //832.5
#define PLAT1_YAW -0.95 
#define PLAT1_DISTANCE 68.9
#define PLAT2_X  -0.36 // 2203.5 x = -0.357296, y = 46.932579, yaw = -1.099379
#define PLAT2_Y 46.93 //915.
#define PLAT2_YAW -1.10
#define PLAT2_DISTANCE 64.8
#define PLAT3_X 6.11 // 2143.5 x = 6.108997, y = 54.660473, yaw = -1.237705
#define PLAT3_Y  54.66 // 990.
#define PLAT3_YAW -1.23
#define PLAT3_DISTANCE 61.6
#define TARGET1_X 28.09 // 2505.  x = 28.092287, y = -8.878185, yaw = 2.011092
#define TARGET1_Y -8.88 //1504.
#define ANGLE_FROM_DOWNHILL_TO_TARGET 70
//#define PIXELS_PER_METER 10.6
//#define METERS_PER_PIXEL 0.0943	use this value for resolution: in global and local_costmap_params.yaml and in field_map.yaml
//#define DISTANCE_TO_FIRST_TARGET_STAGING 2.0
//#define DISTANCE_TO_FIRST_TARGET 12.0
#define TARGET_STAGING_DISTANCE 10.
#define INCREMENTAL_MOVE_TO_TARGET 10.0
#define INCREMENTAL_TURN_TO_FIND_TARGET -20.0
#define SHORT_MOVE_TO_TARGET 1.0
//#define FIRST_MOVE_REMAINING_DISTANCE 10.0
#define PARKING_DISTANCE 1.5
#define PLATFORM_DISTANCE 3.0
#define SERVO_WAIT_SECONDS 3.0  // time from servo command until we are sure servo is in position *****************untested change*************
#define DEGREES_PER_PAN_STEP 0.64
#define PAN_CAMERA_DELTA 20
#define PAN_CAMERA_SEARCH_MAX 20
#define TILT_CAMERA_DELTA 10
#define TILT_CAMERA_SEARCH_MAX 20

#define WEBCAM_DISTANCE_LIMIT 2.
#define TARGET_DISTANCE_FOR_WEBCAM_TILT_DOWN 2.

#define REGULAR_DIGCAM_FAR_ZOOM 10
#define REGULAR_DIGCAM_ZOOM5 5
#define REGULAR_DIGCAM_SHORT_ZOOM 0
//#define ZOOM_DIGCAM_ZOOM 7

// OBSTACLE AVOIDANCE DEFINES
#define STOP_IF_OBSTACLE_WITHIN_DISTANCE 1.5  // If there is an obstacle within this distance, react.
#define ROBOT_RADIUS 0.63

// namespace aliases for shorter typing
//namespace obn = OutdoorBot::Navigation;

ros::ServiceClient NavTargets_client_, accelerometers_client_, dirAnt_client_; //, encoders_client_; //, mainTargets_client_, mainTargetsCheckImage_client_;
ros::ServiceClient setPose_client_; //digcams_client_, 
ros::Publisher digcam_pub_, movement_pub_, webcam_pub_, mainTargetsCommand_pub_, NavTargetsCommand_pub_, servo_pub_, dirAnt_pub_; //, pmotor_pub_;
ros::Subscriber home_center_sub_, mainTargetImageReceived_sub_, navTargetImageReceived_sub_, target_center_sub_, move_complete_sub_, radar_sub_;
ros::Subscriber userDesignatedTargets_sub_, pause_sub_;
outdoor_bot::webcams_custom webcam_command_;
outdoor_bot::digcams_custom digcam_command_; 
int distanceToFirstTarget_, distanceToFirstTargetStaging_;
int centerX_, centerY_, totalX_, moving_, turning_, alreadyTurned_, picking_, orienting_, alreadyOriented_, parking_, totalMoveToFirstTarget_ = 0;
int home_image_height_, home_image_width_;
bool homeCenterUpdated_, movementComplete_, triedWebcamAlready_, triedZoomDigcamAlready_, triedRegularDigcamAlready_;
double range_, approxRangeToTarget_, targetRange_, homeCameraRange_, offsetX_, webcamTilt_, regularDigcamZoom_, zoomDigcamZoom_, regularDigcamFOV_;
double accelX_, accelY_, accelZ_, x_, y_, yaw_, odomDistanceToHome_;
bool zoomResult_, writeFileResult_, newMainTargetDigcamImageReceived_, newMainTargetWebcamImageReceived_, rangeUnknown_;
bool targetCenterUpdated_, newNavTargetImageReceived_;
int retCapToMemory_;
double distanceToHomeRadar_, angleToHomeRadar_, orientationToHomeRadar_;
double distanceToRadarStagingPoint_, angleToRadarStagingPoint_;
bool radarGoodAngle_, radarGoodData_, radarNewData_, usingRadar_, pastStagingPoint_, platforming_, atPlatform_;
//double velocityToHomeRadar_ = 0., previousdistanceToHomeRadar_ = 0.; 
int platformXPose_[3], platPoseX_;
int platformYPose_[3], platPoseY_;
int platformYawPose_[3], platPoseYaw_;
int targetXPose_, targetYPose_;
bool phase1_, pauseNewCommand_, pauseCommanded_, previousState_, recoverTurnedAlready_, firstMoveToFirstTarget_, firstMoveToTarget_, finalMoveToTarget_;
int deltaServoDegrees_ = PAN_CAMERA_DELTA, deltaTilt_ = TILT_CAMERA_DELTA, searchCounter_ = 0, currentServoDegrees_ = 0; 
int maxSearchPan_ = PAN_CAMERA_SEARCH_MAX, maxSearchTilt_ = TILT_CAMERA_SEARCH_MAX, regularDigCamFOV_, zoomDigcamFOV_;
int lastCamName_ = WEBCAM, camName_ = WEBCAM;
std::string movementResult_;
//int encoderPickerUpper_, encoderDropBar_, encoderBinShade_;
bool prepping_, placing_, driving_, pushing_, scooping_, dropping_, retrieving_, verifying_, alreadyTriedVerifying_, binShading_;
ros::Time overallTimer_;
double startTime_, totalTime_, secondsRemaining_, numTargets_;
double userCmdDistance_[32], userCmdTurn_[32], userCmdSpeed_[32], userCmdPickup_[32];
int userCmdNumDataValues_, currentUserCommandNumber_, userCmdReturnSection_;
bool userCommandReceived_;
int dirAntMaxAngle_, dirAntSweepNumber_, dirAntLevel_, usingDirAnt_;
int currentSection_;

OutdoorBot::Navigation::ObstacleDetector *obstacle_detector_;
OutdoorBot::Navigation::WallFollower     *obstacle_avoider_;

FBFSM fsm_;
int BootupState_, CheckLinedUpState_, PhaseTwoFirstState_, CheckFirstTargetState_, SearchForFirstTargetState_;
int MoveToFirstTargetState_;
int AvoidObstacleState_;
int NewTargetState_, CheckTargetState_, FindTargetState_, MoveToTargetState_, PickupTargetState_, PhaseOneHomeState_;
int CheckHomeState_, SearchForHomeState_;
int HeadForHomeState_, MoveOntoPlatformState_, UserCommandState_;
int MoveToRecoverState_, MoveToRecoverFailedState_, ReEntryState_, PauseState_, AllDoneState_;
int MoveToRecoverCounter_ = 0, TurnToRecoverCounter_ = 0;
int platformNumber_;

bool getUserInput();

void imageCapture(string command, int camName, bool writeFile = false);

class targetAquireFSM
{
   private:
      FBFSM tAfsm_;

      int moveCameraState_, captureImageState_, analyzeImageState_, acquireDoneState_, acquireReEntryState_, acquirePauseState_;
      int acquirePreviousState_;
      string camCommand_;
      int servoNumber_;
      bool firstTarget_, homeTarget_, fileWrite_;
      int acquireCamName_, servoDegrees_; // servo position in degrees (ccw is positive)
      ros::Time servoTimer_;

      void on_enter_MoveCamera()
      {
         if (pauseCommanded_) return;
         outdoor_bot::servo_msg msg;
         msg.servoNumber = servoNumber_;
         msg.servoDegrees = servoDegrees_;
         //msg.servoTilt = tilt_;
         cout << "servo command sent, servoNumber, degrees: " << servoNumber_ << ", " << servoDegrees_ << endl;
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
         else if (acquireCamName_ == REGULAR_DIGCAM || acquireCamName_ == ZOOM_DIGCAM) newMainTargetDigcamImageReceived_ = false;
         else newMainTargetWebcamImageReceived_ = false;
         imageCapture(camCommand_, acquireCamName_, fileWrite_);  // command an image capture
         //cout << "image capture commanded for camera " << acquireCamName_ << ", fileWrite_ =" << fileWrite_ << endl;
      }

      int on_update_CaptureImage()
      {
            if (!homeTarget_)
            {
            	if ( (newMainTargetDigcamImageReceived_ && (acquireCamName_ == REGULAR_DIGCAM || acquireCamName_ == ZOOM_DIGCAM) )
            		|| (newMainTargetWebcamImageReceived_ && acquireCamName_ == WEBCAM) ) return analyzeImageState_; // image received by mainTargets
            }
            else if (newNavTargetImageReceived_) return analyzeImageState_; // image analyzed by NavTargets
            ros::spinOnce();
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
		      if (newMainTargetDigcamImageReceived_ && (acquireCamName_ == REGULAR_DIGCAM || acquireCamName_ == ZOOM_DIGCAM)) newMainTargetDigcamImageReceived_ = false;
		      if (newMainTargetWebcamImageReceived_ && acquireCamName_ == WEBCAM) newMainTargetWebcamImageReceived_ = false;
		      outdoor_bot::mainTargetsCommand_msg msg; 
		      msg.cameraName = acquireCamName_;
		      msg.firstTarget = firstTarget_;  
		      msg.approxRange = approxRangeToTarget_;
		      if (acquireCamName_ == REGULAR_DIGCAM) msg.regularDigcamZoom = regularDigcamZoom_;
		      else if (acquireCamName_ == ZOOM_DIGCAM) msg.zoomDigcamZoom = zoomDigcamZoom_;
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
         ros::spinOnce();
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
         acquireCamName_ = ZOOM_DIGCAM;
         firstTarget_ = true;
         homeTarget_ = false;
         camCommand_ = "capture";
         servoNumber_ = DIGCAM_PAN;
         servoDegrees_ = 0;
         //tilt_ = 0;
         fileWrite_ = false;
         setupFSM();
      }

      int getCaptureImageState() { return captureImageState_; }
      int getMoveCameraState() { return moveCameraState_; }
      int getAcquireDoneState() { return acquireDoneState_; }
      int current_state() { return tAfsm_.current_state(); }
      void update() { tAfsm_.update(); }
      void set_state(int value) { tAfsm_.set_state(value); }
      void set_acquireCamName(int value) { acquireCamName_ = value; }
      int get_acquireCamName() { return acquireCamName_; }
      void set_camCommand(string value) { camCommand_ = value; }
      void set_firstTarget(int value) { firstTarget_ = value; }
      void set_homeTarget(int value) { homeTarget_ = value; }
      void set_servoNumber(int value) { servoNumber_ = value; }
      void set_servoDegrees(int value) { servoDegrees_ = value; }
      //void set_tilt(int value) { tilt_ = value; }
      void set_fileWrite(int value) { fileWrite_ = value; }
   
};

targetAquireFSM tAF_;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

bool askUser()
{
	string input = "";
	int userValue = 1;
   cout << "Enter 1 to move on, 0 to retry: " << endl;
   getline(cin, input);
   ros::spinOnce(); // listen to hear pause command

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

bool askUserForGo()
{
	string input = "";
	int userValue = 0;
   cout << "WE ARE ABOUT TO MOVE TO FULLY AUTONOMOUS OPERATIONS!  SET RC CONTROL TO PAUSE (may need to cycle unpause/pause once)." << endl;
   cout << "when that is done, we will continue......." << endl;
   while (!pauseCommanded_)
   {
      struct timespec ts;
      ts.tv_sec = 0;
      ts.tv_nsec = 10000000;
      nanosleep(&ts, NULL); // update every 10 ms, saves cpu from going to 100%
      ros::spinOnce();
    }
   cout << "pause is acknowledged as set." << endl;
   cout << "WHEN YOU ARE READY TO GO, HIT 1.  TO ABORT AND RETURN TO LINEUP STATE, HIT 0" << endl;
   getline(cin, input);

   // This code converts from string to number safely.
   stringstream myStream(input);
   if (myStream >> userValue)
   {
   	if (userValue == 1) return true;
   	return false;
   }	
   cout << "Failed to get a user value, returning false from askUserForGo" << endl;
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
 
 

bool callAccelerometersService()
{
	outdoor_bot::accelerometers_service::Request req;
	outdoor_bot::accelerometers_service::Response resp;
	//double aX = 0., aY = 0., aZ = 0.;
	cout << "calling accelerometers service"  << endl;
	/* this would just yield 10 of the same number, as the accels updat at only 50 Hz
	for (int i=0; i < 10; i++)
	{
		bool success = accelerometers_client_.call(req, resp);
		if (success)
		{
			aX += resp.accelX;
			aY += resp.accelY;
			aZ += resp.accelZ;			
		}
	}
	accelX_ = aX / 10.;
	accelY_ = aY / 10.;
	accelZ_ = aZ / 10.;
	*/
	
	bool success = accelerometers_client_.call(req, resp);
	if (success)
	{
		accelX_ = resp.accelX;
		accelY_ = resp.accelY;
		accelZ_ = resp.accelZ;	
		yaw_ = resp.yaw * 57.3; // degrees
		x_ = resp.x; // meters
		y_ = resp.y;	
		odomDistanceToHome_ = sqrt((x_ * x_) + (y_ * y_));	
	}
	cout << "accels, x, y, z = " << accelX_ << ", " << accelY_ << ", " << accelZ_ << endl;
	cout << "x, y, yaw (radians), yaw (degrees), odomDistanceToHome_ (meters)  = " << x_ << ", " << y_ << ", " << yaw_ / 57.3 << ", " << yaw_ << ", " << odomDistanceToHome_ << endl;
	//if (fabs(accelZ_) > 3.) return true;
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
      if (platformNumber_ == 1) distanceToFirstTarget_ = PLAT1_DISTANCE;
      else if (platformNumber_ == 2) distanceToFirstTarget_ = PLAT2_DISTANCE;
      else if (platformNumber_ == 3) distanceToFirstTarget_ = PLAT3_DISTANCE;
      else distanceToFirstTarget_ = platformNumber_;
      distanceToFirstTargetStaging_ = distanceToFirstTarget_ - TARGET_STAGING_DISTANCE;
      cout << "We are on platform number " << platformNumber_  << ", a distance = " << distanceToFirstTarget_ << " meters from the first target" << endl;
      cout << "Staging distance = " << distanceToFirstTargetStaging_ << endl;
      
		         
		//******************************************use this with a blank map, decide later for real ops*************
		// makes using odometry in autonmous node a lot easier***************
		if (!callSetPoseService(0,0,0,true)) //platPoseX_, platPoseY_, platPoseYaw_, true)) // set home pose in robotPose_node
		{
		   cout << "failed to set home pose" << endl;
		}
		if (!callSetPoseService(0,0,0,false)) //platPoseX_, platPoseY_, platPoseYaw_, false)) // and place us there
		{
		   cout << "failed to place us at home pose on the map" << endl;
		}
		//******************************************use this with a blank map, decide later for real ops********
		
		cout << "current nav parameters are as follows: " << endl;
		callAccelerometersService();
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


void imageCapture(string command, int camName, bool writeFile)
{
  if (camName == WEBCAM)
   {
      cout << "publishing command for webcam capture" << endl;
      webcam_command_.command = command;
      webcam_command_.camera_number = camName;
      webcam_command_.tilt = webcamTilt_;
      if (writeFile) webcam_command_.write_file = true;
      else webcam_command_.write_file = false;
      webcam_pub_.publish(webcam_command_);
   }
   else if (camName == ZOOM_DIGCAM || camName == REGULAR_DIGCAM)
   {
      cout << "publishing command to capture image from  ";
      if (camName == ZOOM_DIGCAM)
      {
      	digcam_command_.zoom = zoomDigcamZoom_;
      	cout << "ZOOM_DIGCAM" << endl;
      }
      else if (camName == REGULAR_DIGCAM)
      {
      	digcam_command_.zoom = regularDigcamZoom_;
      	cout << "REGULAR_DIGCAM" << endl;
      }
      else cout << "unknown camera" << endl;
      digcam_command_.command = command;
      digcam_command_.cameraName = camName;
      if (writeFile) digcam_command_.write_file = true;
      else digcam_command_.write_file = false;
      digcam_pub_.publish(digcam_command_); 
   }
   else cout << "request made for capture from unknown camera, camera number = " << camName << endl;
}     

void setZoom(int camName, float zoomSet)
{
   digcam_command_.command = "setZoom";
   digcam_command_.cameraName = camName;
   digcam_command_.zoomSet = zoomSet;
   digcam_pub_.publish(digcam_command_); 
}  

void testCameras()
{ 
   cout << "testing cameras" << endl;
   // the digital cams:
   
   bool fileWrite = true;
   
   cout << "capturing image on zoom digcam..." << endl;
   imageCapture("capture", ZOOM_DIGCAM, fileWrite);
   while (!askUser())
   {
   	cout << "capturing another image on zoom digcam..." << endl;
   	imageCapture("capture", ZOOM_DIGCAM, fileWrite);
   }
   
   cout << "capturing image on right digcam..." << endl;
   imageCapture("capture", REGULAR_DIGCAM, fileWrite);
   while (!askUser()) 
   {
   	cout << "capturing another image on right digcam..." << endl;
   	imageCapture("capture", REGULAR_DIGCAM, fileWrite);
   }
   
   // the webcams:
   cout << "capturing image on front webcam..." << endl;
   imageCapture("capture", WEBCAM, fileWrite);
   while (!askUser())
   {
   	cout << "capturing another image on front webcam..." << endl;
   	imageCapture("capture", WEBCAM, fileWrite);
   }
   
   /*
   cout << "capturing image on rear webcam..." << endl;
   imageCapture("capture", REAR_WEBCAM); 
   while (!askUser())
   {
   	cout << "capturing another image on rearcam..." << endl;
   	imageCapture("capture", REAR_WEBCAM);
   }
   */
   
   cout << "setting zoom on zoom digital camera to " << zoomDigcamZoom_ << endl;

   setZoom(ZOOM_DIGCAM, (float) zoomDigcamZoom_ );
   while (!askUser())
   {
   	cout << "setting zoom again on zoom digital camera to " << zoomDigcamZoom_ << endl;
   	setZoom(ZOOM_DIGCAM, (float) zoomDigcamZoom_ );
   }
   
   cout << "setting zoom on right digital camera to " << regularDigcamZoom_ << endl;

   setZoom(REGULAR_DIGCAM, (float) regularDigcamZoom_);
   while (!askUser())
   {
   	cout << "setting zoom again on right digital camera to " << regularDigcamZoom_ << endl;
   	setZoom(REGULAR_DIGCAM, (float) regularDigcamZoom_);
   }	
      
   cout << "capturing image on zoom digcam..." << endl;
   imageCapture("capture", ZOOM_DIGCAM, fileWrite);
   while (!askUser())
   {
   	cout << "capturing another image on zoom digcam..." << endl;
   	imageCapture("capture", ZOOM_DIGCAM, fileWrite);
   }
   
   cout << "capturing image on right digcam..." << endl;
   imageCapture("capture", REGULAR_DIGCAM, fileWrite);
   while (!askUser()) 
   {
   	cout << "capturing another image on right digcam..." << endl;
   	imageCapture("capture", REGULAR_DIGCAM, fileWrite);
   }
}  

void pauseCallback(const std_msgs::Int32::ConstPtr& msg)
{
   bool static outputSent1 = false, outputSent2 = false;	// dont want a million couts
   if (msg->data == 0) 
   {
   	if (!outputSent2) cout << "pause released in pauseCallback" << endl;
   	outputSent1 = false;
   	outputSent2 = true;
   	pauseCommanded_ = false;
  	}
   else
   {
   	if (!outputSent1) cout << "pause commanded in pauseCallback" << endl;
   	outputSent1 = true;
   	outputSent2 = false;
   	pauseCommanded_ = true;
   }   	
}

void userDesignatedTargetsCallback(const std_msgs::String::ConstPtr& msg)
{
  // make a set of user designated moves
  // first parse the string
  string parseCommand = msg->data.c_str();  
  std::string cmdBuffer[32];
  string substr;

  std::size_t found = parseCommand.find_first_of(";");
  userCmdNumDataValues_ = 0;
  while (found!=std::string::npos)
  {
    cmdBuffer[userCmdNumDataValues_] = parseCommand.substr(0,found);
    parseCommand = parseCommand.substr(found + 1, std::string::npos);
    cout << " cmdBuffer[" << userCmdNumDataValues_ << "] = " << cmdBuffer[userCmdNumDataValues_] << endl;
    cout << "UserCommand = " << parseCommand << endl;
    userCmdNumDataValues_++;
    found = parseCommand.find_first_of(";");
  }
  
  userCmdNumDataValues_ -= 1;
  
  ROS_INFO("parsed user command strings: ");
  for (int i = 0; i < userCmdNumDataValues_; i++)
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
    cmdBuffer[i] = cmdBuffer[i].substr(found + 1, std::string::npos);
    found = cmdBuffer[i].find_first_of(",");
    if (found!=std::string::npos)
    {
    	substr = cmdBuffer[i].substr(0,found);
    	userCmdPickup_[i] = atof(substr.c_str()); 
    }  
    cout << "parsed values for dataset " << i << " = " << userCmdDistance_[i] << ", "
    	<< userCmdTurn_[i] << ", " << userCmdSpeed_[i] << ", " << userCmdPickup_[i] << endl;  
  }
  
  substr = cmdBuffer[userCmdNumDataValues_].c_str();
  userCmdReturnSection_ = atof(substr.c_str());
  cout << "commanded return section number = " << userCmdReturnSection_ << endl;
  
  currentUserCommandNumber_ = 0;
  userCommandReceived_ = true;
}

void moveCompleteCallback(const std_msgs::String::ConstPtr& msg)
{
   movementResult_ = msg->data.c_str();
   movementComplete_ = true;
   cout << "movement complete callback message received" << endl;
}
  
void homeCenterCallback(const outdoor_bot::NavTargets_msg::ConstPtr &msg)
{
   centerX_ = msg->centerX;
   centerY_ = msg->centerY;
   totalX_ = msg->totalX;
   homeCameraRange_ = msg->range;
   camName_ = tAF_.get_acquireCamName(); //msg->cameraName;
   homeCenterUpdated_ = true;
   if (homeCameraRange_ > 0)
   {
   	cout << "NavTargets data: camName, center, homeCameraRange: " << camName_ << ", ("
   		<< centerX_ << ", " << centerY_ << "), " << homeCameraRange_ << endl;
   }
   else cout << "NavTargets returned no home target" << endl;
}

void targetCenterCallback(const outdoor_bot::mainTargets_msg::ConstPtr &msg)
{
   /*if (msg->centerX == -99)
   {
      int cameraName = msg->cameraName;
      if (cameraName == REGULAR_DIGCAM || cameraName == ZOOM_DIGCAM) 
      {
   		newMainTargetDigcamImageReceived_ = true;
   		cout << "digcam image was received by mainTargets, cameraName = " << cameraName << endl;
   	}
   	else if (cameraName == WEBCAM) 
   	{
   		newMainTargetWebcamImageReceived_ = true;
   		cout << "webcam image was received by mainTargets, cameraName = " << cameraName << endl;
   	}
   	return;
   }
   */
   centerX_ = msg->centerX;
   centerY_ = msg->centerY;
   totalX_ = msg->totalX;
   targetRange_ = msg->range;
   camName_ = tAF_.get_acquireCamName();
   //camName_ = msg->cameraName;
   targetCenterUpdated_ = true;
   ROS_INFO("target center updated in control: center, range = (%d, %d), %f", centerX_, centerY_, targetRange_);
   cout << "target center callback received data" << endl;  
}

void mainTargetImageReceivedCallback(const std_msgs::Int32::ConstPtr& msg)
{
   int cameraName = msg->data;
   if (cameraName == REGULAR_DIGCAM || cameraName == ZOOM_DIGCAM) 
   {
   	newMainTargetDigcamImageReceived_ = true;
   	cout << "digcam image was received by mainTargets" << endl;
   }
   else if (cameraName == WEBCAM) 
   {
   	newMainTargetWebcamImageReceived_ = true;
   	cout << "webcam image was received by mainTargets, cameraName = " << cameraName << endl;
   }
   else cout << "image was received but unknown camera" << endl;
}

void navTargetImageReceivedCallback(const std_msgs::Int32::ConstPtr& msg)
{
	int cameraName = msg->data;
	if (cameraName == HOMECAM)
   {
   	newNavTargetImageReceived_ = true;
   	cout << "image was received by NavTargets, cameraName = " << cameraName << endl;
   }
   else cout << "image was received from NavTargets but unknown camera" << endl;
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
void radarCallback(const outdoor_bot::radar_msg::ConstPtr& msg)
{
	radarGoodData_ = true;
	radarGoodAngle_ = true;
	radarNewData_ = true;
	if (!msg->goodData) // none of the radars are reporting
	{
		distanceToHomeRadar_ = 0.;
	   radarGoodData_ = false;
	   radarGoodAngle_ = false;
		return;
	}
	
	if (!msg->goodLocation) // at least one radar is reporting and at least one is not
	{
		distanceToHomeRadar_ = (msg->minDistanceToHome + msg->maxDistanceToHome) / 2.;
		radarGoodAngle_ = false;		
		return;
	}
	
	distanceToHomeRadar_ = msg->distanceToHome;
	angleToHomeRadar_ = msg->angleToHome;
	//distanceToHomeRadar_ = msg->runningAverageDistanceToHome;	
	//angleToHomeRadar_ = msg->runningAverageAngleToHome;	
	orientationToHomeRadar_ = msg->orientation;
	distanceToRadarStagingPoint_ = msg->distanceToStagingPoint;
	angleToRadarStagingPoint_ = msg->angleToStagingPoint;
	
	//velocityToHome_ = (previousdistanceToHomeRadar_ - distanceToHomeRadar_ ) / deltaTime;
	
	//previousdistanceToHomeRadar_ = distanceToHomeRadar_;

}


/*
bool callEncodersService()
{
	outdoor_bot::encoders_service::Request req;
   outdoor_bot::encoders_service::Response resp;
   cout << "Calling encoders service" << endl;
   bool success = encoders_client_.call(req,resp);
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
	currentSection_ = BOOTUP;
	numTargets_ = 0;
	approxRangeToTarget_ = 65; 
   targetRange_ = 65;
   homeCameraRange_ = 10.;
   range_ = 65;
   x_ = 0.;
   y_ = 0.;
   yaw_ = 0.;
   rangeUnknown_ = false;
   webcamTilt_ = WEBCAM_TILT_LEVEL;
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
   alreadyTurned_ = 0;
   picking_ = false;
   orienting_ = false;
   alreadyOriented_ = false;
   parking_ = false;
   movementComplete_ = false;
   movementResult_ = "";
   triedWebcamAlready_ = false;
   triedZoomDigcamAlready_ = false;	
   triedRegularDigcamAlready_ = false;
   zoomDigcamZoom_ = ZOOM_DIGCAM_ZOOM7;		// these get set together
   zoomDigcamFOV_ = ZOOM_DIGCAM_ZOOM7_FOV;	// these two	
   regularDigcamZoom_ = REGULAR_DIGCAM_ZOOM5;	// these get set together
   regularDigcamFOV_ = REGULAR_DIGCAM_ZOOM5_FOV;		// these two
   recoverTurnedAlready_ = false;
   firstMoveToFirstTarget_ = true;
   firstMoveToTarget_ = false;
   finalMoveToTarget_ = false;
   verifying_ = false;
   alreadyTriedVerifying_ = false;
   distanceToHomeRadar_ = 0.;
   orientationToHomeRadar_ = 0;
   angleToHomeRadar_ = 0.; 
   distanceToRadarStagingPoint_ = 0.;
   angleToRadarStagingPoint_ = 0.;
   odomDistanceToHome_ = 0.;
   radarGoodData_ = false;
   radarNewData_ = false;
   radarGoodAngle_ = false;
   pastStagingPoint_ = false;
   platforming_ = false;
   atPlatform_ = false;  
	usingRadar_ = false;
	usingDirAnt_ = false;
   platformXPose_[0] = PLAT1_X;
   platformXPose_[1] = PLAT2_X;
   platformXPose_[2] = PLAT3_X;
   platformYPose_[0] = PLAT1_Y;
   platformYPose_[1] = PLAT2_Y;
   platformYPose_[2] = PLAT3_Y;
   platformYawPose_[0] = PLAT1_YAW;
   platformYawPose_[1] = PLAT2_YAW;
   platformYawPose_[2] = PLAT3_YAW;
   targetXPose_ = TARGET1_X;
   targetYPose_ = TARGET1_Y;
   
   //encoderPickerUpper_ = 0;
   //encoderDropBar_ = 0;
   //encoderBinShade_= 0;
   
  for (int i=0; i < 32; i++)
  {
  	userCmdDistance_[i] = 0.;
  	userCmdTurn_[i] = 0.;
  	userCmdSpeed_[i] = 0.;
  	userCmdPickup_[i] = 0.;
  }
  userCmdNumDataValues_ = 0;
  currentUserCommandNumber_ = 0;
  userCmdReturnSection_ = TARGETS;
  userCommandReceived_ = false;
  
  cout << "sending command to move the webcam servo tilt down " << endl;
   // the first input to a newly booted arudino sometimes has garbage, so we send this twice
   // which makes sure it gets done and also is a clear indicator that the servos are working OK
	tAF_.set_servoNumber(FRONT_WEBCAM_TILT);
	tAF_.set_servoDegrees( WEBCAM_TILT_DOWN);
	tAF_.set_state(tAF_.getMoveCameraState());         		   
	while (tAF_.current_state() != tAF_.getAcquireDoneState()) 
	{
		tAF_.update();
		ros::spinOnce();
	}

	// second one
	cout << "sending the same command again" << endl;
	tAF_.set_servoNumber(FRONT_WEBCAM_TILT);
	tAF_.set_servoDegrees( WEBCAM_TILT_DOWN);
	tAF_.set_state(tAF_.getMoveCameraState());         		   
	while (tAF_.current_state() != tAF_.getAcquireDoneState()) 
	{
		tAF_.update();
		ros::spinOnce();
	}
		  
  cout << "finished enter bootupState" << endl;

}

int on_update_BootupState()
{	 		
  	cout << "now in update bootupState" << endl;

   if (pauseCommanded_) return PauseState_;
	
   // get radar ranges
   if (radarGoodData_)
   {
   	usingRadar_ = true;
   	radarNewData_ = false;
   	cout << "radar distance, angle, orientation = " <<	 distanceToHomeRadar_ << ", " << angleToHomeRadar_ << ", " << orientationToHomeRadar_ << endl;
   }
   else
   {
      cout << "radar failed, do you want to retry bootstate?" << endl;
      if (!askUser()) return BootupState_;
      usingRadar_ = false;
      radarNewData_ = false;
   }
   
   // give some time for everyone to setup and get started, then ask what platform we are assigned
   if (getUserInput())
   {
		   //platPoseX_ = platformXPose_[platformNumber_ - 1];
		   //platPoseY_ = platformYPose_[platformNumber_ - 1];
		   //platPoseYaw_ = platformYawPose_[platformNumber_ - 1];
			if (phase1_) totalTime_ = PHASE_ONE_TIME_LIMIT;
			else totalTime_ = PHASE_TWO_TIME_LIMIT;				
	}
   else return BootupState_;
   
   cout << "Do you want to move on to autonomous ops or go through bootstate stuff?" << endl;
   if (askUser()) 			// delete for real ops, but may want to keep the dirant = false ***************************************************************************************
   {
	   usingDirAnt_ = false;			// ************************************************************************ check for real ops********
   	return CheckLinedUpState_; 
   }
   
   // start by setting zooms and taking a photo from each camera
   testCameras();
   
   cout << "bootupState is finished.  Do you want to move on or retry?" << endl;
   if (!askUser()) return BootupState_;
   startTime_ = ros::Time::now().toSec();
   secondsRemaining_ = totalTime_ - (ros::Time::now().toSec() - startTime_);
   if (phase1_) return CheckLinedUpState_;
   else
   {
   	currentSection_ = FIRST_TARGET_MOVE;
   	return PhaseTwoFirstState_;
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
   alreadyTurned_ = 0;
   picking_ = false;
   orienting_ = false;
   alreadyOriented_ = false;
   parking_ = false;
   movementComplete_ = false;
   movementResult_ = "";
   centerX_ = -1; // indicator for when target centers are updated
}

void on_enter_CheckLinedUpState()
{
   // start by capturing an image using fsm
   tAF_.set_acquireCamName(ZOOM_DIGCAM);	// if we change here, also change in on_update below
   tAF_.set_camCommand("capture");
   tAF_.set_firstTarget(true);
   tAF_.set_homeTarget(false);
   tAF_.set_state(tAF_.getCaptureImageState());
}

int on_update_CheckLinedUpState()
{    
   tAF_.update();
   if (tAF_.current_state() != tAF_.getAcquireDoneState())
   {
   	//ros::spinOnce();
   	return CheckLinedUpState_;  // check to see if the image got analyzed 
   }
          
   if (centerX_ > 0)
   {
   	 cout << "possible target found: x, y, range = " << centerX_ << ", " << centerY_ << ", " << targetRange_ << endl;
   	 
   	cout << "since this is phase one, we are manually lined up and will not turn but, based on the camera image,";
   	if (tAF_.get_acquireCamName() == WEBCAM) cout << " we would turn an angle = " << ((320. - centerX_) / 640.) * WEBCAM_FOV << endl;
   
   	else if (tAF_.get_acquireCamName() == REGULAR_DIGCAM) 
   	cout << " we would turn an angle = " << ((1136. - centerX_) / 2272.) * regularDigcamFOV_ << " with zoom set to " << regularDigcamZoom_ << endl;
   	
   	else cout << " we would turn an angle = " << ((1136. - centerX_) / 2272.) * zoomDigcamFOV_ << " with zoom set to " << zoomDigcamZoom_ << endl;
   }
   else cout << "no target found yet" << endl; 
   
   
   
   cout << "Do you want to move on to autonomous ops or retry LineUp?" << endl;
   if (askUserForGo())
   {  	
     	currentSection_ = FIRST_TARGET_MOVE;
     	centerX_ = -1; // we have manually centered the robot, we do not want it to turn!     	
     	return MoveToFirstTargetState_;
   }
   
   // decided to try again, so we will capture an image using fsm
   tAF_.set_acquireCamName(ZOOM_DIGCAM);			//if we change here , change above in on_enter too
   tAF_.set_camCommand("capture");
   tAF_.set_firstTarget(true);
   tAF_.set_homeTarget(false);
   tAF_.set_state(tAF_.getCaptureImageState());
   return CheckLinedUpState_;
}

void on_enter_PhaseTwoFirstState()
{
	// start by just moving off the platform 5 meters
	currentSection_ = FIRST_TARGET_MOVE;
	outdoor_bot::movement_msg msg;
	msg.command = "autoMove";
	msg.angle = 0.;
	msg.speed = 1000.;
	msg.distance = 5000.;
	cout << "starting blind move, distance = " << msg.distance << " mm" << endl;
	moving_ = true;
	movement_pub_.publish(msg);
	movementComplete_ = false;
}

int on_update_PhaseTwoFirstState()
{
   if (!movementComplete_) return PhaseTwoFirstState_;
   
   if (moving_ || turning_)	//  move is complete
   {	      	      
		if (turning_)	// we have completed the first move and the downhill turn
		{
			turning_ = false;
			movementComplete_ = false;
			approxRangeToTarget_ = 60.;
			centerX_ = -1;
			firstMoveToFirstTarget_ = true;
			cout << "finshed initial move and turn in phase two" << endl;
			pauseCommanded_ = true;
			return PauseState_;
		}
	}
	moving_ = false;
	
	// now we turn downhill
	double targetAngle = 0.;
	if (callAccelerometersService())
	{
		double downhillDirection;
		if (accelX_ > 0.01) downhillDirection = atan(accelY_ / accelX_) * 57.3;
		else if (accelX_ < -0.01)
		{
			if (accelY_ >= 0) downhillDirection = 180 - (atan(accelY_ / -accelX_) * 57.3);
			else downhillDirection = -180 + (atan(accelY_ / -accelX_) * 57.3);
		}
		else downhillDirection = 90. * sgn(accelY_);
		cout << "downhill direction = " << downhillDirection << endl;
		targetAngle = ANGLE_FROM_DOWNHILL_TO_TARGET - downhillDirection;
	}
	outdoor_bot::movement_msg msg;
	msg.command = "autoMove";
	msg.angle = targetAngle;
	msg.speed = 20.;
	msg.distance = 0.;
	cout << "starting blind turn, angle = " << msg.angle << " degrees" << endl;
	turning_ = true;
	movement_pub_.publish(msg);
	movementComplete_ = false; 
   return PhaseTwoFirstState_;  // check to see if the image got analyzed 
}

void on_enter_CheckFirstTargetState()
{
   currentSection_ = FIRST_TARGET_CHECK;
   // start by capturing an image using fsm
   centerX_ = -1;
   cout << "entering CheckFirstTargetState, approximate range to target = " << approxRangeToTarget_ << endl;
   if (!triedZoomDigcamAlready_ && !verifying_)
   {
		tAF_.set_acquireCamName(ZOOM_DIGCAM);
		lastCamName_ = ZOOM_DIGCAM;		
   }
   else if (!triedRegularDigcamAlready_ && !verifying_) //approxRangeToTarget_ > WEBCAM_DISTANCE_LIMIT)  // too far for webcam *****************might add for real ops to avoid false positives
   {
   	tAF_.set_acquireCamName(REGULAR_DIGCAM);
   	lastCamName_ = REGULAR_DIGCAM;
   }
   else
   {
   	tAF_.set_acquireCamName(WEBCAM);
   	lastCamName_ = WEBCAM;  
   } 	
   tAF_.set_camCommand("capture");
   tAF_.set_homeTarget(false);
   tAF_.set_firstTarget(true);
   tAF_.set_state(tAF_.getCaptureImageState()); 
}

int on_update_CheckFirstTargetState()
{    
   tAF_.update();
   if (tAF_.current_state() != tAF_.getAcquireDoneState()) return CheckFirstTargetState_;  // check to see if the image got analyzed 
          
   if (centerX_ > 0)
   {
      cout << "possible target found: x, y, range = " << centerX_ << ", " << centerY_ << ", " << targetRange_ << endl;
      if (lastCamName_ == WEBCAM)	// once we got it in the webcam, no need to search with other cameras
      {
      	triedZoomDigcamAlready_ = true;
      	triedRegularDigcamAlready_ = true;
      }
      else if (lastCamName_ == REGULAR_DIGCAM) triedZoomDigcamAlready_ = true;  // one we got it in the regular digcam, no need to check the zoom digcam
      else // if it was found with the zoom digcam, the we still want to try them all
      {
         triedZoomDigcamAlready_ = false;
      	triedRegularDigcamAlready_ = false;
      }
      	
      searchCounter_ = 0;
      if (fabs(currentServoDegrees_) > 0) // return the servo to 0 if it was panned
      {
		   tAF_.set_servoDegrees(0);
			tAF_.set_state(tAF_.getMoveCameraState());
			while (tAF_.current_state() != tAF_.getAcquireDoneState()) 
			{
				tAF_.update();
				ros::spinOnce();
			}
		}
      return MoveToFirstTargetState_;
   }
   else if (finalMoveToTarget_ && lastCamName_ == WEBCAM) // the webcam does not use the pan servo, so if we didn't see it here, just move on
   {
   	cout << "did final move to target, so it is OK that we did not see a target here" << endl;
   	triedZoomDigcamAlready_ = false;	// these are in case we need to redo
      triedRegularDigcamAlready_ = false;
   	if (verifying_)	// we were checking to see if we left the target on the ground, but we did not see it, so hopefully we got it
   	{
   		verifying_ = false;
   		alreadyTriedVerifying_ = false;
   		finalMoveToTarget_ = false;
   		return PhaseOneHomeState_;
   	}
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
      deltaServoDegrees_ = -deltaServoDegrees_; // switch directions each time
		currentServoDegrees_ += deltaServoDegrees_ * searchCounter_;
		searchCounter_++;
		
		if (abs(currentServoDegrees_) > PAN_CAMERA_SEARCH_MAX)
		{
		   tAF_.set_servoDegrees(0);
		   tAF_.set_state(tAF_.getMoveCameraState());         		   
		   while (tAF_.current_state() != tAF_.getAcquireDoneState()) 
		   {
			   tAF_.update();
   			ros::spinOnce();
		   }
		   
		   if (triedZoomDigcamAlready_ && triedRegularDigcamAlready_) // then the camera pan that exceeded max must be the webcam, so we have tried them all
			{
				cout << "no camera sees a target in SeachForFirstTargetState" << endl;	
				// we will reset currentServoDegrees_ later, when we use it to know that we exceeded PAN_CAMERA_SEARCH_MAX	 
				triedRegularDigcamAlready_ = false;
				triedZoomDigcamAlready_ = false;  
				return; // no camera sees the target, so move a bit
			
			}
		   
		   // move on to the next camera
		   if (triedZoomDigcamAlready_)
		   {
		   	cout << "no target with regular digcam, we'll try the webcam" << endl;
		   	triedRegularDigcamAlready_ = true;
				tAF_.set_acquireCamName(WEBCAM); // this just signals to the updater to return to checkfirsttargetstate_
				searchCounter_ = 0;
		   	currentServoDegrees_ = 0;
		   	// first set tilt
				/*
				if (approxRangeToTarget_ > TARGET_DISTANCE_FOR_WEBCAM_TILT_DOWN)
				{
					if (webcamTilt_ == WEBCAM_TILT_DOWN)
					{
						cout << "tilting webcam up to be level" << endl;
						tAF_.set_servoNumber(FRONT_WEBCAM_TILT);
						webcamTilt_ = WEBCAM_TILT_LEVEL;
						tAF_.set_servoDegrees(WEBCAM_TILT_LEVEL);
						tAF_.set_state(tAF_.getMoveCameraState());
						while (tAF_.current_state() != tAF_.getAcquireDoneState())
						{
		   				tAF_.update();
		   				ros::spinOnce();
						}
					}
				}
				else if (webcamTilt_ == WEBCAM_TILT_LEVEL)
				{
					cout << "tilting webcam down" << endl;
					tAF_.set_servoNumber(FRONT_WEBCAM_TILT);
					webcamTilt_ = WEBCAM_TILT_DOWN;
					tAF_.set_servoDegrees(WEBCAM_TILT_DOWN);
					tAF_.set_state(tAF_.getMoveCameraState());
					while (tAF_.current_state() != tAF_.getAcquireDoneState())
					{
	   				tAF_.update();
	   				ros::spinOnce();
					}
				}
				*/	
				return;	// since the webcam does not use the pan servo, we can just return now
			}		   
		   cout << "no target with the zoom digcam, we'll try the regular digcam" << endl;
		   triedZoomDigcamAlready_ = true;
		   searchCounter_ = 1;	// we take this image at 0 pan and then will pan camera
		   //*******************************************untested change*********
		   currentServoDegrees_ = 0;
		}
		
		
		if (!triedZoomDigcamAlready_)
		{		
			tAF_.set_servoNumber(ZOOM_DIGCAM_PAN);
			tAF_.set_acquireCamName(ZOOM_DIGCAM);
			cout << "setting zoom digcam pan = " << currentServoDegrees_ << endl;
		}
		
		else if (!triedRegularDigcamAlready_)
		{
			tAF_.set_servoNumber(DIGCAM_PAN);
			tAF_.set_acquireCamName(REGULAR_DIGCAM);
			cout << "setting regular digcam pan = " << currentServoDegrees_ << endl;	
		}	
		
		//proceed with pan
		if (fabs(currentServoDegrees_) > 0)
		{
			tAF_.set_servoDegrees(currentServoDegrees_);
			tAF_.set_state(tAF_.getMoveCameraState());
		}		
}

int on_update_SearchForFirstTargetState()
{
   if (tAF_.get_acquireCamName() == WEBCAM && searchCounter_ == 0) return CheckFirstTargetState_;
   if (tAF_.get_acquireCamName() == WEBCAM && searchCounter_ > 0)
   {
   	currentServoDegrees_ = 0;
   	if (centerX_ > 0)
   	{
   		cout << "got a target with the webcam" << endl;
   	}
   	else
   	{
   		cout << "no camera saw any targets, moving on " << endl;
   		triedZoomDigcamAlready_ = false; 
   		triedRegularDigcamAlready_ = false;
   		return MoveToFirstTargetState_;
   	}
   }
   
   if (fabs(currentServoDegrees_) > 0)
	{
   	tAF_.update();
   	if (tAF_.current_state() != tAF_.getAcquireDoneState()) return SearchForFirstTargetState_; // waiting for servo delay
   }
   return CheckFirstTargetState_; // go see if there is a target in this camera direction
}

void on_enter_MoveToFirstTargetState()
{
      obstacle_detector_->activate(STOP_IF_OBSTACLE_WITHIN_DISTANCE, ROBOT_RADIUS);
      previousState_ = MoveToFirstTargetState_;
      if (firstMoveToFirstTarget_) 
      {
      	currentSection_ = FIRST_TARGET_MOVE; // after the first move, it is better for pause to return to checkFirstTargetState
      	moving_ = false;
			turning_ = false;
			movementComplete_ = false;	
			centerX_ = -1;	
      	return;
      }
      else currentSection_ = FIRST_TARGET_CHECK;
      
     if (lastCamName_ == WEBCAM && centerX_ > 0) // if we have seen it in the webcam stay with the webcam
   {
   	triedZoomDigcamAlready_ = true; 
   	triedRegularDigcamAlready_ = true;
   	cout << "we have seen the target in the webcam, so we will skip the other cameras" << endl;
   }
   if (lastCamName_ == REGULAR_DIGCAM && centerX_ > 0) // if we have seen it in the regular digcam, then dont check the zoom digcam
   {
   	triedZoomDigcamAlready_ = true; 
   	cout << "we have seen the target in the regular digcam, so we will skip the zoom digcam" << endl;
   }   
   movementComplete_ = false;
   moving_ = false;
   turning_ = false;
}

int on_update_MoveToFirstTargetState()
{
   if (pauseCommanded_)	// we go here as the very first step in autonomous ops
   {
      return PauseState_;
   }

   if (obstacle_detector_->update())
   {
      // We've seen an obstacle!
      cout << "obstacle detected in moveToFirstTargetState" << endl;
    // return AvoidObstacleState_; ************change for real ops to have an action *************
   }

   // check to see if we have arrived at the new pose   
   if ( (moving_ || turning_) && (!movementComplete_ ))
   {
   	ros::spinOnce();
   	return MoveToFirstTargetState_;  // move has not completed yet
   } 
   
   if (moving_ || turning_)	//  move is complete
   {	      	      
		moving_ = false;
		turning_ = false;
		movementComplete_ = false;		
	   if (movementResult_.find("done") != string::npos)
	   {
	   	if (firstMoveToFirstTarget_)
	   	{
	   		ROS_INFO("move or turn completed in MoveToFirstTargetState");
	   		cout << "approxRangeToTarget_ = " << approxRangeToTarget_ << endl;
	   		return MoveToFirstTargetState_;	// keep moving until we get there.
	   	}
	   	else if (centerX_ < 0)	// we did not have a target and are not in the first moves, just moved forward with hope
	   	{
	   		cout << "blind move completed in MoveToFirstTargetState, approxRangeToTarget =  " << approxRangeToTarget_ << endl;
	   	}
	   	else cout << "move or turn toward the target image completed, now let's check if we can see it from here" << endl;
	   	
	   	return CheckFirstTargetState_; // move succeeded, see if we can image the target
	   }
	   else
	   {
	      firstMoveToFirstTarget_ = false;
	      previousState_ = MoveToFirstTargetState_; // come back to here when we have recovered movement
	      ROS_INFO("move failed in MoveToFirstTargetState, going to recover state");
	      return MoveToRecoverState_;   // the move failed for some reason, so we have to see what's up
	   }
	}
   
   
   // time to get moving
   
   outdoor_bot::movement_msg msg;
     
   // if this is the very first move, we hope we can use the radar to get close
   if (firstMoveToFirstTarget_)
   {
   	double distanceToHome = -1;
   	if (callAccelerometersService())
		{
			distanceToHome = odomDistanceToHome_;
			// turn to zero the yaw
			cout << "yaw_ = " << yaw_ << endl;
			if (fabs(yaw_) > 3.0 && alreadyTurned_ < 1)
			{
				msg.command = "autoMove";
				msg.distance = 0.;
			   msg.speed = -20. * sgn(yaw_); // go in the opposite direction of the yaw
			   msg.angle = yaw_ + (sgn(yaw_) * 2);  // we turn a little extra to compensate for having moved forward with yaw drift.  we could calculate this better
				//***************************
				
				cout << "turning to zero yaw = " << msg.angle << " degrees" << endl;
				turning_ = true;
				movement_pub_.publish(msg);
				movementComplete_ = false;
				alreadyTurned_++;
				return MoveToFirstTargetState_;
			}
			cout << "yaw is small enough to skip correction turn" << endl;
		}
		turning_ = false;
		alreadyTurned_ = 0;
				
   	if (radarGoodData_ && distanceToHomeRadar_ > 4)
   	{
   		radarNewData_ = false;
   		distanceToHome = distanceToHomeRadar_;
   		cout << "radar data is good" << endl;
   	}
   	else distanceToHome = odomDistanceToHome_;
   	if (distanceToHome >= 0)
   	{
   		if (distanceToHome < distanceToFirstTargetStaging_ - INCREMENTAL_MOVE_TO_TARGET)
   		{
   			msg.distance = INCREMENTAL_MOVE_TO_TARGET * 1000.;
   			approxRangeToTarget_ = distanceToFirstTarget_ - (distanceToHome + INCREMENTAL_MOVE_TO_TARGET);
   			cout << "making incremental move to target, approxRangeToTarget = " << approxRangeToTarget_ << endl;
   			cout << "odom and radar distances to home = " << odomDistanceToHome_ << ", " << distanceToHomeRadar_ << endl;
   			cout << " we are using " << distanceToHome << " as our distance to home" << endl;
   			cout << " with a distance to FirstTarget = " << distanceToFirstTarget_ << endl;
   		}
   		else if (distanceToHome < distanceToFirstTargetStaging_)
   		{
   			msg.distance  = (distanceToFirstTargetStaging_ - distanceToHome) * 1000.;
   			approxRangeToTarget_ = distanceToFirstTarget_ - distanceToFirstTargetStaging_;
   			firstMoveToFirstTarget_ = false;
   			cout << "we just finished firstMoveToFirstTarget" << endl;
   		}
   		else 
   		{
   			firstMoveToFirstTarget_ = false;
   			ROS_INFO("move completed in MoveToFirstTargetState");
	   		cout << "approxRangeToTarget_ = " << approxRangeToTarget_ << endl;
   			return CheckFirstTargetState_;
   		}
   	}
   	else
   	{
			cout << "unable to get either radar or odom data, so we have to wander a bit" << endl;
			msg.distance = SHORT_MOVE_TO_TARGET * 1000.;
			approxRangeToTarget_ -= SHORT_MOVE_TO_TARGET;
   	}
   	msg.command = "autoMove";
   	msg.speed = 1000.;
		cout << "starting moves to first target, distance = " << msg.distance << " mm" << endl;
		moving_ = true;
		movement_pub_.publish(msg);
		movementComplete_ = false;
		return MoveToFirstTargetState_;
   }
   
    if (finalMoveToTarget_ && centerX_ < 0)
	{
		cout << "we made a final move to target and hoped to get in one more centering turn, but must be too close, going to try picking up" << endl;
		return PickupTargetState_; // we hoped to get one more centering turn in, but must have overrun
	}
	     
   if (centerX_ < 0 && (!finalMoveToTarget_)) // we don't have a target in view yet, so we will just turn or move forward a bit
	{    
		if (approxRangeToTarget_ > 2.)
		{
			msg.command = "autoMove";
			msg.angle = 0.;
			msg.speed = 1000.;
			msg.distance = INCREMENTAL_MOVE_TO_TARGET * 1000.;
			cout << "starting blind move, distance = " << msg.distance << " mm" << endl;
			cout << "approximate range to target = " << approxRangeToTarget_ << endl;
			moving_ = true;
			movement_pub_.publish(msg);
			movementComplete_ = false;
			return MoveToFirstTargetState_;
		}
		msg.command = "autoMove";
		msg.angle = INCREMENTAL_TURN_TO_FIND_TARGET;
		msg.speed = -20.;
		msg.distance = 0.;
		cout << "starting blind turn, distance = " << msg.angle * sgn(msg.speed) << " degrees" << endl;
		turning_ = true;
		movement_pub_.publish(msg);
		movementComplete_ = false;
		return MoveToFirstTargetState_;		
   }
   
   // we found a target, if we have not already centered it, do that now
   	
   if (!turning_ && alreadyTurned_ < 2 && centerX_ > 0)
   {
		offsetX_ = ((double) ((totalX_ / 2) - centerX_)) / ((double) totalX_);  // fraction that the image is off-center
		cout << "totalX_, centerX_, center offset ratio = " << totalX_ << ", " << centerX_ << ", " << offsetX_ << endl;
		centerX_ = totalX_/ 2; // reset centerX, assuming we turn correctly.
		if (lastCamName_ == WEBCAM) offsetX_ *= WEBCAM_FOV;
		else if (lastCamName_ == REGULAR_DIGCAM)
		{
			if (approxRangeToTarget_ <= 2)  offsetX_ *= (regularDigcamFOV_ * 0.8) + 4.;  // camera is panned a bit to the left and located a bit to the right, so when we get close, we have to correct for the offset
			else offsetX_ *= regularDigcamFOV_;
		}
		else if (lastCamName_ == ZOOM_DIGCAM) offsetX_ *= zoomDigcamFOV_;
		else offsetX_ = 0.;
		cout << "center offset degrees = " << offsetX_ << endl;
		double servoOffset = currentServoDegrees_ * 0.9; //*******************************
		currentServoDegrees_ = 0.;	// the servos were centered earlier, but we waited to reset currentServoDegrees_ so we could use it here
		cout << "servo offset degrees = " << servoOffset << endl;
		//double offsetY = ((double) (totalY_ - centerY_)) / ((double) totalY_);
		offsetX_ += servoOffset; //*********************************************check this*****************************************************************************
		cout << "total offset degrees = " << offsetX_ << endl;
		
		if (fabs(offsetX_) > 5 || (approxRangeToTarget_  < 3 && fabs(offsetX_) > 3.))
		{
			// send turn command to center the target
			msg.command = "autoMove";
			msg.angle = offsetX_;	// sign here does not matter, direction is set by the speed
			// positive offset means turn to the left (ccw), negative to the right.
			msg.speed = 20. * sgn(offsetX_);	// degrees per second
			msg.distance = 0.;
			movement_pub_.publish(msg);
		   turning_ = true;
		   movementComplete_ = false;
			ROS_INFO("centering target");
			alreadyTurned_++;
			return MoveToFirstTargetState_;
		}
		else cout << "centered well enough, now will move" << endl;
		
	}
	
	if (finalMoveToTarget_)
	{
		cout << "we made a final move to target and got in one more centering turn, now let's go pick it up" << endl;
		return PickupTargetState_; 
	}
	
	// we already centered the target
	alreadyTurned_ = 0;
	currentServoDegrees_ = 0.;
	msg.angle = 0.;
	turning_ = false;
		
	if (targetRange_ > 0.1)	
	{
		range_ = targetRange_;
		approxRangeToTarget_ = targetRange_;
		cout << "setting both range and approximate range to target to the range determined by the camera image = " << approxRangeToTarget_ << endl;
   }		
	else if (approxRangeToTarget_ > INCREMENTAL_MOVE_TO_TARGET)
	{
		cout << " setting range_ to = approxRangeToTarget_ = " << approxRangeToTarget_ << endl;
		range_ = approxRangeToTarget_;
	}
	else
	{
		range_ = (2. * INCREMENTAL_MOVE_TO_TARGET) + 1.;	// if we just cannot figure out a range, then try this
		cout << "As a last option, we are just setting the range = (2* incremental move to target) + 1 = " << range_ << endl;
	}
   
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
	   msg.command = "autoMove";
	   msg.distance = 10000.;
	   msg.speed = 1000.; 
	   approxRangeToTarget_ -= 10.;
	   ROS_INFO("moving forward 10m");
	}
		
	else if (range_ > 10.)
	{
	   msg.command = "autoMove";
	   msg.distance = 5000.;
	   msg.speed = 1000.; 
	   approxRangeToTarget_ -= 5.;
	   ROS_INFO("moving forward 5m");
	}
	
	else
	*/
	 if (range_ > 5.)
	{
	   msg.command = "autoMove";
		msg.distance = 1000;	// mm
		approxRangeToTarget_ -= 3.;
		msg.speed = 800.;   	// mm/sec or deg/sec 
	   ROS_INFO("auto moving forward 1m");
	}     
	
	else if (range_ > 2.)
	{	   
	   msg.command = "autoMove";
		msg.distance = 500;	// mm
		approxRangeToTarget_ = 2.;
		msg.speed = 800.;   	// mm/sec or deg/sec 
		ROS_INFO("auto moving forward 1m"); 
	}
	
	else if (range_ > 1.)
	{
	   msg.command = "autoMove";
		msg.distance = (range_ - 0.9) * 1000.;	// mm
		approxRangeToTarget_ = 0.9;
		finalMoveToTarget_ = true;
		msg.speed = 800.;   	// mm/sec or deg/sec 
		ROS_INFO("auto moving forward less than 1m, final move to target");    
	}
	  
	else if (range_ > 0.01)
	{
	   // final approach
	   cout << "range is very short, time to pick up the target, range_ = " << range_ << endl;
	   
	   moving_ = false;
	   turning_ = false;
	   alreadyTurned_ = false;	   
	   return PickupTargetState_;
	}
	
	else return CheckFirstTargetState_;
	
   
   msg.angle = 0.;
	movement_pub_.publish(msg);
   moving_ = true;
   movementComplete_ = false;   
   return MoveToFirstTargetState_;
}

void on_enter_AvoidObstacleState()
{
   cout << "Avoiding an obstacle.";
	moving_ = false;
	turning_ = false;
	movementComplete_ = false;
	obstacle_avoider_->activate(OutdoorBot::Navigation::WallFollower::Goal(OutdoorBot::Navigation::WallFollower::Goal::LEFT));
}

int on_update_AvoidObstacleState()
{  
   OutdoorBot::Navigation::WallFollower::Input input(OutdoorBot::Navigation::WallFollower::Input::READY_FOR_NEW_COMMAND);  
   OutdoorBot::Navigation::WallFollower::Output output;
   
   // check to see if we have arrived at the new pose   
	if ( (moving_ || turning_) && (!movementComplete_ ))
	{
		ros::spinOnce();
		input = OutdoorBot::Navigation::WallFollower::Input(OutdoorBot::Navigation::WallFollower::Input::EXECUTING_COMMAND);
	} 

	else if (moving_ || turning_)	//  move is complete
	{	      	      
		moving_ = false;
		turning_ = false;
		movementComplete_ = false;
  		input = OutdoorBot::Navigation::WallFollower::Input(OutdoorBot::Navigation::WallFollower::Input::READY_FOR_NEW_COMMAND);	
  	}
	
  // move has not begun
  //input = OutdoorBot::Navigation::WallFollower::Input(OutdoorBot::Navigation::WallFollower::Input::EXECUTING_COMMAND);
  output = obstacle_avoider_->update(input);

  outdoor_bot::movement_msg msg;

  switch (output.mode())
  {
    case OutdoorBot::Navigation::WallFollower::Output::TIMEOUT:
      // TODO: The obstacle avoider failed because something timed out.  Fill this in with something appropriate
      cout << "wall follower output is TIMEOUT" << endl;
      
      /*
      if (currentSection_ == FIRST_TARGET_CHECK) previousState_ =  CheckFirstTargetState_;
		if (currentSection_ == FIRST_TARGET_MOVE) previousState_ =  MoveToFirstTargetState_;
		if (currentSection_ == TARGETS) previousState_ =  CheckTargetState_;
		if (currentSection_ == HOME) previousState_ =  CheckHomeState_;
		if (currentSection_ == PLATFORM) previousState_ =  MoveOntoPlatformState_;
		if (currentSection_ == ALL_DONE) previousState_ = AllDoneState_;
   	return MoveToRecoverState_;  // something is seriously wrong if we get to here 
   	*/
   	return AvoidObstacleState_;
   	
   	
    case OutdoorBot::Navigation::WallFollower::Output::SUCCESSFUL_COMPLETION:
      cout << "wall follower output is SUCCESSFUL_COMPLETION" << endl;
      
      return AvoidObstacleState_;
      
      if (currentSection_ == FIRST_TARGET_CHECK) return CheckFirstTargetState_;
		if (currentSection_ == FIRST_TARGET_MOVE) return MoveToFirstTargetState_;
		if (currentSection_ == TARGETS) return CheckTargetState_;
		if (currentSection_ == HOME) return CheckHomeState_;
		if (currentSection_ == PLATFORM) return MoveOntoPlatformState_;
		if (currentSection_ == ALL_DONE) return AllDoneState_;
   	return MoveToRecoverState_;  // something is seriously wrong if we get to here 
    case OutdoorBot::Navigation::WallFollower::Output::WAIT_FOR_READY:
      // We're waiting for the last command to complete.  Do nothing.
      cout << "wall follower output is WAIT_FOR_READY" << endl;
      return AvoidObstacleState_;
    case OutdoorBot::Navigation::WallFollower::Output::OBSTACLE_AHEAD:
      // TODO: Slam on brakes here!
      cout << "wall follower output is OBSTACLE_AHEAD" << endl;
     	msg.command = "autoMove";
     	msg.angle = 0.;
		msg.distance = 0.;	// mm
		msg.speed = 0.;   	// mm/sec or deg/sec 
	//	movement_pub_.publish(msg);
   	moving_ = true;
   	movementComplete_ = false; 		
		ROS_INFO("hitting brakes for obstacle"); 
      return AvoidObstacleState_;
    case OutdoorBot::Navigation::WallFollower::Output::MOVE_FORWARD:
    	cout << "wall follower output is MOVE_FORWARD" << endl;
      ROS_INFO("I should send a command to move %f meters.", output.distance());
      // TODO: Send the arduino a command to move forward a distance of output.distance():
      msg.command = "autoMove";
     	msg.angle = 0.;
		msg.distance = output.distance() * 1000.;	// mm
		msg.speed = 1000. * sgn(output.distance());   	// mm/sec
	//	movement_pub_.publish(msg);
   	moving_ = true;
   	movementComplete_ = false; 	
      return AvoidObstacleState_;
    case OutdoorBot::Navigation::WallFollower::Output::TURN:
      cout << "wall follower output is TURN" << endl;
      ROS_INFO("I should send a command to turn %f radians.", output.distance());
      // TODO: send the arduino a command to turn a distance of output.distance().
      msg.command = "autoMove";
     	msg.angle =  output.distance() * 57.3;
		msg.distance = 0;	// mm
		msg.speed = 20. * sgn(output.distance());   	// deg/sec
	//	movement_pub_.publish(msg);
   	turning_ = true;
   	movementComplete_ = false;       
      return AvoidObstacleState_;
    case OutdoorBot::Navigation::WallFollower::Output::STOP:
      cout << "wall follower output is STOP" << endl;
      // We're presumably already stopped and want to stay so.  Do nothing.
      return AvoidObstacleState_;
  }

  return AvoidObstacleState_;
}


void on_enter_NewTargetState()
{
	// check time remaining
	
	// we just picked up a target and are ready to get another
	// we know that the targets are at least 20m apart, so we will take that as our approx range
	// we will go downhill for our direction
	currentSection_ = TARGETS;
	firstMoveToTarget_ = true;
	binShading_ = false;
	approxRangeToTarget_ = 40.;
	startTime_ = ros::Time::now().toSec();
   secondsRemaining_ = totalTime_ - (ros::Time::now().toSec() - startTime_);
   numTargets_++;
   cout << " we have " << secondsRemaining_ << " seconds remaining on the clock" << endl;
   cout << " we have gathered " << numTargets_;
   
   // have to clear the area of the target
   
	// start by turning 180 degrees
	double targetAngle = 180.;
	/*
	if (callAccelerometersService())
	{
		double downhillDirection;
		if (accelX_ > 0.01) downhillDirection = atan(accelY_ / accelX_) * 57.3;
		else if (accelX_ < -0.01)
		{
			if (accelY_ >= 0) downhillDirection = 180 - (atan(accelY_ / -accelX_) * 57.3);
			else downhillDirection = -180 + (atan(accelY_ / -accelX_) * 57.3);
		}
		else downhillDirection = 90. * sgn(accelY_);
		cout << "downhill direction = " << downhillDirection << endl;
		targetAngle = ANGLE_FROM_DOWNHILL_TO_TARGET - downhillDirection;
	}
	*/
	outdoor_bot::movement_msg msg;
	msg.command = "autoMove";
	msg.angle = targetAngle;
	msg.speed = 20.;
	msg.distance = 0.;
	cout << "starting blind turn, angle = " << msg.angle << " degrees" << endl;
	turning_ = true;
	movement_pub_.publish(msg);
	movementComplete_ = false; 
	obstacle_detector_->activate(STOP_IF_OBSTACLE_WITHIN_DISTANCE, ROBOT_RADIUS);

}

int on_update_NewTargetState()
{
   if (!movementComplete_)
   {
   	if (obstacle_detector_->update())
   	{
      	// We've seen an obstacle!
      	cout << "obstacle detected in NewTargetState" << endl;
 //    		return AvoidObstacleState_;  // ***************uncomment for real ops**********************8
   	} 
   	return NewTargetState_;
   }
   
   if (moving_ || turning_)	//  move is complete
   {	      	      
		if (moving_)	// we have completed the first move and the downhill turn
		{
			moving_ = false;
			movementComplete_ = false;
			approxRangeToTarget_ = 20.;
			centerX_ = -1;
			firstMoveToFirstTarget_ = true;
			cout << "finshed initial move and turn in phase two" << endl;
			if (numTargets_ > 3 || secondsRemaining_ < 1800) return HeadForHomeState_;
			pauseCommanded_ = true;
			return PauseState_;
		}
	}
	turning_ = false;
	
		
	outdoor_bot::movement_msg msg;
	msg.command = "autoMove";
	msg.angle = 0.;
	msg.speed = 1000.;
	msg.distance = 10000.;
	cout << "starting blind move, distance = " << msg.distance << " mm" << endl;
	moving_ = true;
	movement_pub_.publish(msg);
	movementComplete_ = false;

   return NewTargetState_;  // check to see if the image got analyzed 
}


 void on_enter_CheckTargetState()
 {
   currentSection_ = TARGETS;
   // start by capturing an image using fsm
   centerX_ = -1;
   cout << "entering CheckTargetState, approximate range to target = " << approxRangeToTarget_ << endl;
   if (!triedZoomDigcamAlready_ && !verifying_)
   {
		tAF_.set_acquireCamName(ZOOM_DIGCAM);
		lastCamName_ = ZOOM_DIGCAM;		
   }
   else if (!triedRegularDigcamAlready_ && !verifying_) //approxRangeToTarget_ > WEBCAM_DISTANCE_LIMIT)  // too far for webcam *****************might add for real ops to avoid false positives
   {
   	tAF_.set_acquireCamName(REGULAR_DIGCAM);
   	lastCamName_ = REGULAR_DIGCAM;
   }
   else
   {
   	tAF_.set_acquireCamName(WEBCAM);
   	lastCamName_ = WEBCAM;  
   } 	
   tAF_.set_camCommand("capture");
   tAF_.set_homeTarget(false);
   tAF_.set_firstTarget(false);
   tAF_.set_state(tAF_.getCaptureImageState()); 
}

int on_update_CheckTargetState()
{    
   tAF_.update();
   if (tAF_.current_state() != tAF_.getAcquireDoneState()) return CheckTargetState_;  // check to see if the image got analyzed 
          
   if (centerX_ > 0)
   {
      cout << "possible target found: x, y, range = " << centerX_ << ", " << centerY_ << ", " << targetRange_ << endl;
      if (lastCamName_ == WEBCAM)	// once we got it in the webcam, no need to search with other cameras
      {
      	triedZoomDigcamAlready_ = true;
      	triedRegularDigcamAlready_ = true;
      }
      else if (lastCamName_ == REGULAR_DIGCAM) triedZoomDigcamAlready_ = true;  // one we got it in the regular digcam, no need to check the zoom digcam
      else // if it was found with the zoom digcam, the we still want to try them all
      {
         triedZoomDigcamAlready_ = false;
      	triedRegularDigcamAlready_ = false;
      }
      	
      searchCounter_ = 0;
      if (fabs(currentServoDegrees_) > 0) // return the servo to 0 if it was panned
      {
		   tAF_.set_servoDegrees(0);
			tAF_.set_state(tAF_.getMoveCameraState());
			while (tAF_.current_state() != tAF_.getAcquireDoneState()) 
			{
				tAF_.update();
				ros::spinOnce();
			}
		}
      return MoveToTargetState_;
   }
   else if (finalMoveToTarget_ && lastCamName_ == WEBCAM) // the webcam does not use the pan servo, so if we didn't see it here, just move on
   {
   	cout << "did final move to target, so it is OK that we did not see a target here" << endl;
   	triedZoomDigcamAlready_ = false;	// these are in case we need to redo
      triedRegularDigcamAlready_ = false;
   	if (verifying_)	// we were checking to see if we left the target on the ground, but we did not see it, so hopefully we got it
   	{
   		verifying_ = false;
   		alreadyTriedVerifying_ = false;
   		finalMoveToTarget_ = false;
   		return PhaseOneHomeState_;
   	}
   	return MoveToTargetState_;
   }
   else
   {
      cout << "no target found yet" << endl;     
      return FindTargetState_;
   }  
}

 void on_enter_FindTargetState()
 {
   	currentSection_ = TARGETS;
      deltaServoDegrees_ = -deltaServoDegrees_; // switch directions each time
		currentServoDegrees_ += deltaServoDegrees_ * searchCounter_;
		searchCounter_++;
		
		if (abs(currentServoDegrees_) > PAN_CAMERA_SEARCH_MAX)
		{
		   tAF_.set_servoDegrees(0);
		   tAF_.set_state(tAF_.getMoveCameraState());         		   
		   while (tAF_.current_state() != tAF_.getAcquireDoneState()) 
		   {
			   tAF_.update();
   			ros::spinOnce();
		   }
		   
		   if (triedZoomDigcamAlready_ && triedRegularDigcamAlready_) // then the camera pan that exceeded max must be the webcam, so we have tried them all
			{
				cout << "no camera sees a target in FindTargetState" << endl;	
				// we will reset currentServoDegrees_ later, when we use it to know that we exceeded PAN_CAMERA_SEARCH_MAX	   
				return; // no camera sees the target, so move a bit
			
			}
		   
		   // move on to the next camera
		   if (triedZoomDigcamAlready_)
		   {
		   	cout << "no target with regular digcam, we'll try the webcam" << endl;
		   	triedRegularDigcamAlready_ = true;
				tAF_.set_acquireCamName(WEBCAM); // this just signals to the updater to return to checktargetstate_
				searchCounter_ = 0;
		   	currentServoDegrees_ = 0;
		   	// first set tilt
				/*
				if (approxRangeToTarget_ > TARGET_DISTANCE_FOR_WEBCAM_TILT_DOWN)
				{
					if (webcamTilt_ == WEBCAM_TILT_DOWN)
					{
						cout << "tilting webcam up to be level" << endl;
						tAF_.set_servoNumber(FRONT_WEBCAM_TILT);
						webcamTilt_ = WEBCAM_TILT_LEVEL;
						tAF_.set_servoDegrees(WEBCAM_TILT_LEVEL);
						tAF_.set_state(tAF_.getMoveCameraState());
						while (tAF_.current_state() != tAF_.getAcquireDoneState())
						{
		   				tAF_.update();
		   				ros::spinOnce();
						}
					}
				}
				else if (webcamTilt_ == WEBCAM_TILT_LEVEL)
				{
					cout << "tilting webcam down" << endl;
					tAF_.set_servoNumber(FRONT_WEBCAM_TILT);
					webcamTilt_ = WEBCAM_TILT_DOWN;
					tAF_.set_servoDegrees(WEBCAM_TILT_DOWN);
					tAF_.set_state(tAF_.getMoveCameraState());
					while (tAF_.current_state() != tAF_.getAcquireDoneState())
					{
	   				tAF_.update();
	   				ros::spinOnce();
					}
				}
				*/	
				return;	// since the webcam does not use the pan servo, we can just return now
			}		   
		   cout << "no target with the zoom digcam, we'll try the regular digcam" << endl;
		   triedZoomDigcamAlready_ = true;
		   searchCounter_ = 0;
		   currentServoDegrees_ = 0;
		}
		
		
		if (!triedZoomDigcamAlready_)
		{		
			tAF_.set_servoNumber(ZOOM_DIGCAM_PAN);
			tAF_.set_acquireCamName(ZOOM_DIGCAM);
			cout << "setting zoom digcam pan = " << currentServoDegrees_ << endl;
		}
		
		else if (!triedRegularDigcamAlready_)
		{
			tAF_.set_servoNumber(DIGCAM_PAN);
			tAF_.set_acquireCamName(REGULAR_DIGCAM);
			cout << "setting regular digcam pan = " << currentServoDegrees_ << endl;	
		}	
		
		//proceed with pan
		if (fabs(currentServoDegrees_) > 0)
		{
			tAF_.set_servoDegrees(currentServoDegrees_);
			tAF_.set_state(tAF_.getMoveCameraState());
		}		
}

int on_update_FindTargetState()
{
   if (tAF_.get_acquireCamName() == WEBCAM && searchCounter_ == 0) return CheckTargetState_;
   if (tAF_.get_acquireCamName() == WEBCAM && searchCounter_ > 0)
   {
   	currentServoDegrees_ = 0;
   	return MoveToTargetState_;
   }
   
   if (fabs(currentServoDegrees_) > 0)
	{
   	tAF_.update();
   	if (tAF_.current_state() != tAF_.getAcquireDoneState()) return FindTargetState_; // waiting for servo delay
   }
   return CheckTargetState_; // go see if there is a target in this camera direction
}

void on_enter_MoveToTargetState()
{
   if (lastCamName_ == WEBCAM && centerX_ > 0) // if we have seen it in the webcam stay with the webcam
   {
   	triedZoomDigcamAlready_ = true; 
   	triedRegularDigcamAlready_ = true;
   }
   if (lastCamName_ == REGULAR_DIGCAM && centerX_ > 0) // if we have seen it in the regular digcam, then dont check the zoom digcam
   {
   	triedZoomDigcamAlready_ = true; 
   }   
   movementComplete_ = false;
   moving_ = false;
   turning_ = false;
   obstacle_detector_->activate(STOP_IF_OBSTACLE_WITHIN_DISTANCE, ROBOT_RADIUS);
}

int on_update_MoveToTargetState()
{
   if (pauseCommanded_)	
   {
      previousState_ = MoveToTargetState_;
      return PauseState_;
   }

   if (obstacle_detector_->update())
   {
     // We've seen an obstacle!
     cout << "obstacle detected in MoveToTargetState" << endl;
//     return AvoidObstacleState_; // uncomment for real ops ************************
   }

   // check to see if we have arrived at the new pose   
   if ( (moving_ || turning_ || binShading_) && (!movementComplete_ ))
   {
   	ros::spinOnce();
   	return MoveToTargetState_;  // move has not completed yet
   } 
   
   if (moving_ || turning_ || binShading_)	//  move is complete
   {	      	      
		moving_ = false;
		turning_ = false;
		movementComplete_ = false;		
	   if (movementResult_.find("done") != string::npos)
	   {
	   	if (firstMoveToTarget_ || binShading_)
	   	{
	   		if (!binShading_)
	   		{
	   			// we moved around enough to get the target to drop into the bin, so now we need to move the shade
	   			outdoor_bot::movement_msg msg;
	   			movementComplete_ = false;
	   			binShading_ = true;
					cout << "moving bin shade" << endl;
					msg.command = "PDmotorAuto";
					msg.PDmotorNumber = MOTOR_BIN_SHADE;
					msg.speed = BIN_SHADE_DOWN;
					movement_pub_.publish(msg);
					return MoveToTargetState_;
				}
	   		binShading_ = false;	
	   		firstMoveToTarget_ = false;
	   		ROS_INFO("move or turn completed in MoveToTargetState");
	   		cout << "approxRangeToTarget_ = " << approxRangeToTarget_ << endl;
	   		return CheckTargetState_;
	   	}
	   	else if (centerX_ < 0)	// we did not have a target and are not in the first moves, just moved forward with hope
	   	{
	   		cout << "blind move completed in MoveToTargetState, approxRangeToTarget =  " << approxRangeToTarget_ << endl;
	   	}
	   	else cout << "move or turn toward the target image completed, now let's check if we can see it from here" << endl;
	   	
	   	return CheckTargetState_; // move succeeded, see if we can image the target
	   }
	   else
	   {
	      firstMoveToTarget_ = false;
	      previousState_ = MoveToTargetState_; // come back to here when we have recovered movement
	      ROS_INFO("move failed in MoveToTargetState, going to recover state");
	      return MoveToRecoverState_;   // the move failed for some reason, so we have to see what's up
	   }
	}
   
   
   // time to get moving
   
   outdoor_bot::movement_msg msg;
     
   // if this is the very first move, we will just move another 10m to get away from the previous target location
   if (firstMoveToTarget_)
   {
   	msg.distance = 10000.;
		approxRangeToTarget_ -= 10.;
   	msg.command = "autoMove";
   	msg.speed = 1000.;
		cout << "starting moves to first target, distance = " << msg.distance << " mm" << endl;
		moving_ = true;
		movement_pub_.publish(msg);
		movementComplete_ = false;
		return MoveToTargetState_;
   }
   
    if (finalMoveToTarget_ && centerX_ < 0)
	{
		cout << "we made a final move to target and hoped to get in one more centering turn, but must be too close, going to try picking up" << endl;
		return PickupTargetState_; // we hoped to get one more centering turn in, but must have overrun
	}
	     
   if (centerX_ < 0 && (!finalMoveToTarget_)) // we don't have a target in view yet, so we will just turn or move forward a bit
	{    
		if (approxRangeToTarget_ > 10.)
		{
			msg.command = "autoMove";
			msg.angle = 0.;
			msg.speed = 1000.;
			msg.distance = INCREMENTAL_MOVE_TO_TARGET * 1000.;
			cout << "starting blind move, distance = " << msg.distance << " mm" << endl;
			moving_ = true;
			movement_pub_.publish(msg);
			movementComplete_ = false;
			return MoveToTargetState_;
		}
		msg.command = "autoMove";
		msg.angle = INCREMENTAL_TURN_TO_FIND_TARGET;
		msg.speed = -20.;
		msg.distance = 0.;
		cout << "starting blind turn, distance = " << msg.angle * sgn(msg.speed) << " degrees" << endl;
		turning_ = true;
		movement_pub_.publish(msg);
		movementComplete_ = false;
		return MoveToTargetState_;		
   }
   
   // we found a target, if we have not already centered it, do that now
   	
   if (!turning_ && alreadyTurned_ < 2 && centerX_ > 0)
   {
		offsetX_ = ((double) ((totalX_ / 2) - centerX_)) / ((double) totalX_);  // fraction that the image is off-center
		cout << "totalX_, centerX_, center offset ratio = " << totalX_ << ", " << centerX_ << ", " << offsetX_ << endl;
		centerX_ = totalX_/ 2; // reset centerX, assuming we turn correctly.
		if (lastCamName_ == WEBCAM) offsetX_ *= WEBCAM_FOV;
		else if (lastCamName_ == REGULAR_DIGCAM)
		{
			//if (approxRangeToTarget_ <= 2)  offsetX_ *= (regularDigcamFOV_ * 0.8) + 4.;  // camera is panned a bit to the left and located a bit to the right, so when we get close, we have to correct for the offset
			//else
			offsetX_ *= regularDigcamFOV_;
		}
		else if (lastCamName_ == ZOOM_DIGCAM) offsetX_ *= zoomDigcamFOV_;
		else offsetX_ = 0.;
		cout << "center offset degrees = " << offsetX_ << endl;
		double servoOffset = currentServoDegrees_ * 0.9; //*******************************
		currentServoDegrees_ = 0.;	// the servos were centered earlier, but we waited to reset currentServoDegrees_ so we could use it here
		cout << "servo offset degrees = " << servoOffset << endl;
		//double offsetY = ((double) (totalY_ - centerY_)) / ((double) totalY_);
		offsetX_ += servoOffset; //*********************************************check this*****************************************************************************
		cout << "total offset degrees = " << offsetX_ << endl;
		
		if (fabs(offsetX_) > 5 || (approxRangeToTarget_  < 3 && fabs(offsetX_) > 3.))
		{
			// send turn command to center the target
			msg.command = "autoMove";
			msg.angle = offsetX_;	// sign here does not matter, direction is set by the speed
			// positive offset means turn to the left (ccw), negative to the right.
			
			
			msg.speed = 15. * sgn(offsetX_);	// degrees per second******check this
			
			
			
			msg.distance = 0.;
			movement_pub_.publish(msg);
		   turning_ = true;
		   movementComplete_ = false;
			ROS_INFO("centering target");
			alreadyTurned_++;
			return MoveToTargetState_;
		}
		else cout << "centered well enough, now will move" << endl;
		
	}
	
	if (finalMoveToTarget_)
	{
		cout << "we made a final move to target and got in one more centering turn, now let's go pick it up" << endl;
		return PickupTargetState_; 
	}
	
	// we already centered the target
	alreadyTurned_ = 0;
	currentServoDegrees_ = 0.;
	msg.angle = 0.;
	turning_ = false;
		
	if (targetRange_ > 0.1)	
	{
		range_ = targetRange_;
		approxRangeToTarget_ = targetRange_;
		cout << "setting both range and approximate range to target to the range determined by the camera image = " << approxRangeToTarget_ << endl;
   }		
	else if (approxRangeToTarget_ > INCREMENTAL_MOVE_TO_TARGET)
	{
		cout << " setting range_ to = approxRangeToTarget_ = " << approxRangeToTarget_ << endl;
		range_ = approxRangeToTarget_;
	}
	else
	{
		range_ = (2. * INCREMENTAL_MOVE_TO_TARGET) + 1.;	// if we just cannot figure out a range, then try this
		cout << "As a last option, we are just setting the range = (2* incremental move to target) + 1 = " << range_ << endl;
	}
   
   cout << "Moving to target, range = " << range_ << endl; 
    
   
	if (range_ > 20.)
	{
	   msg.command = "autoMove";
	   msg.distance = 10000.;
	   msg.speed = 1000.; 
	   approxRangeToTarget_ -= 10.;
	   ROS_INFO("moving forward 10m");
	}
		
	else if (range_ > 10.)
	{
	   msg.command = "autoMove";
	   msg.distance = 5000.;
	   msg.speed = 1000.; 
	   approxRangeToTarget_ -= 5.;
	   ROS_INFO("moving forward 5m");
	}
	
	else if (range_ > 5.)
	{
	   msg.command = "autoMove";
		msg.distance = 1000;	// mm
		approxRangeToTarget_ -= 3.;
		msg.speed = 1000.;   	// mm/sec or deg/sec 
	   ROS_INFO("auto moving forward 1m");
	}     
	
	else if (range_ > 2.)
	{	   
	   msg.command = "autoMove";
		msg.distance = 500;	// mm
		approxRangeToTarget_ = 2.;
		finalMoveToTarget_ = true;
		msg.speed = 800.;   	// mm/sec or deg/sec 
		ROS_INFO("auto moving forward 1m"); 
	}
	
	else if (range_ > 1.)
	{
	   msg.command = "autoMove";
		msg.distance = (range_ - 0.9) * 1000.;	// mm
		approxRangeToTarget_ = 0.9;
		finalMoveToTarget_ = true;
		msg.speed = 800.;   	// mm/sec or deg/sec 
		ROS_INFO("auto moving forward less than 1m, final move to target");    
	}
	  
	else if (range_ > 0.01)
	{
	   // final approach
	   cout << "range is very short, time to pick up the target, range_ = " << range_ << endl;
	   
	   moving_ = false;
	   turning_ = false;
	   alreadyTurned_ = false;	   
	   return PickupTargetState_;
	}
	
	else return CheckTargetState_;
	 
   msg.angle = 0.;
	movement_pub_.publish(msg);
   moving_ = true;
   movementComplete_ = false;   
   return MoveToTargetState_;
}

void on_enter_PickupTargetState()
{
	//callEncodersService();  // get initial locations
	finalMoveToTarget_ = false;
	outdoor_bot::movement_msg msg;   
   msg.command = "PDmotorAuto";
   msg.PDmotorNumber = MOTOR_PICKER_UPPER;
   cout << "putting down the scooper" << endl;
   msg.speed = PICKER_UPPER_DOWN; //PICKER_UPPER_DOWN_PREP;************************real ops
   movement_pub_.publish(msg);    
   movementComplete_ = false;   
   prepping_ = true;
   placing_ = false;
   driving_ = false;
   dropping_ = false;
   pushing_ = false;
   scooping_ = false;
   retrieving_ = false;	
   verifying_ = false;
}

int on_update_PickupTargetState()
{
   // here we do final staging, which means line up with the target and drive straight to it and 
   // proceed with picking it up
   
   static ros::Time timeIn = ros::Time::now() + ros::Duration(15); 
   static ros::Time timeOut = ros::Time::now() + ros::Duration(30); // initialize with the scooper down times, since it started moving during on_enter

	outdoor_bot::movement_msg msg;   
   msg.command = "PDmotorAuto";
   
   if ( (!movementComplete_ || ros::Time::now() < timeIn ) // need min times because the arduino serial data
   	&& ros::Time::now() < timeOut) //Waiting for the move to finish
   {
   	ros::spinOnce();			   
   	return PickupTargetState_;
   }
   else
   {   	
   	if (prepping_) // finished putting scooper in place, now move forward
   	{
   		prepping_ = false;
   		driving_ = true;
   		movementComplete_ = false;
	      cout << "driving forward 2m to place target on the front of the scooper" << endl;
		   msg.command = "autoMove";
		   msg.distance = 2000;	// mm
		   msg.angle = 0;			// degrees
		   msg.speed = 800;   	// mm/sec or deg/sec 
		   timeIn = ros::Time::now() + ros::Duration(5);
		   timeOut = ros::Time::now() + ros::Duration(30); 
   		movement_pub_.publish(msg);
   		return PickupTargetState_;
   	}
   	
   	else if (driving_) // finished driving to target, now drop bar
   	{
   		driving_ = false;
   		dropping_ = true;
   		movementComplete_ = false;
   		cout << "dropping bar" << endl;
   	   msg.PDmotorNumber = MOTOR_DROP_BAR;
   		msg.speed = DROP_BAR_DOWN;
   		timeIn = ros::Time::now() + ros::Duration(30);
		   timeOut = ros::Time::now() + ros::Duration(60); 
   		movement_pub_.publish(msg); 
   		return PickupTargetState_;
   	}
   	   	
   	else if (dropping_) // finished dropping the bar, now push target onto scooper
   	{
   		dropping_ = false;
   		pushing_ = true;
   		movementComplete_ = false;
	      cout << "driving forward 0.5m to push target into scoop" << endl;
		   msg.command = "autoMove";
		   msg.distance = 1000;
		   msg.angle = 0; 
		   msg.speed = 800; 
		   timeIn = ros::Time::now() + ros::Duration(5);
		   timeOut = ros::Time::now() + ros::Duration(30); 	  
   		movement_pub_.publish(msg);
   		return PickupTargetState_;
   	}
   	  	
   	else if (pushing_) // finished placing scoop, now pick up target
   	{
   		pushing_ = false;
   		scooping_ = true;
   		movementComplete_ = false;
   		cout << "scooping up target" << endl;
   	   msg.PDmotorNumber = MOTOR_PICKER_UPPER;
   		msg.speed = PICKER_UPPER_UP;
   		timeIn = ros::Time::now() +ros::Duration(15);
		   timeOut = ros::Time::now() + ros::Duration(30);
   		movement_pub_.publish(msg);
   		return PickupTargetState_;
   	}
   	
   	else if (scooping_)	// finished scooping, now retrieve the drop bar
   	{
   		scooping_ = false;
   		retrieving_ = true;
   		movementComplete_ = false;
   		cout << "retrieving drop bar" << endl;
   	   msg.PDmotorNumber = MOTOR_DROP_BAR;
   		msg.speed = DROP_BAR_UP;
   		timeIn = ros::Time::now() + ros::Duration(35);
		   timeOut = ros::Time::now() + ros::Duration(60);
   		movement_pub_.publish(msg); 
   		return PickupTargetState_;  
   	} 
   	else if (retrieving_ && !alreadyTriedVerifying_) // we hope we got the target in the bin, we'll back up a bit and check to see if it is still on the ground
   	{
   		retrieving_ = false;
   		verifying_ = true;
   		movementComplete_ = false;
	      cout << "driving backwards 1m to check that we got the target" << endl;
		   msg.command = "autoMove";
		   msg.distance = 1000;	// mm
		   msg.angle = 0;			// degrees
		   msg.speed = -800;   	// mm/sec or deg/sec 
		   timeIn = ros::Time::now() + ros::Duration(5);
		   timeOut = ros::Time::now() + ros::Duration(30); 
   		movement_pub_.publish(msg);
   		return PickupTargetState_;  
   	}
   	else if (verifying_ && !alreadyTriedVerifying_)
   	{
   		 finalMoveToTarget_ = true;
   		 alreadyTriedVerifying_ = true;
   		 return CheckFirstTargetState_;
   	}	
   	else if (phase1_)
   	{
   		verifying_ = false;
   		alreadyTriedVerifying_ = false;
   		finalMoveToTarget_ = false;
   		currentSection_ = HOME; 
   		return PhaseOneHomeState_;
   	}
   	// we just picked up a target in phase two competition
   	// we know that the targets are at least 20m apart, so we will take that as our approx range
   	// we will go downhill for our direction
   	else
   	{
			currentSection_ = TARGETS;
			approxRangeToTarget_ = 20.;
			return NewTargetState_;
		}
		
   }  
}		

bool updateDirectionalAntenna()
{      	
	outdoor_bot::dirAnt_msg dirAntMsg;
	dirAntMsg.antennaCommand = 1;		// sweep antenna to find max direction.  Note that this takes the arduino about a second
	dirAntMsg.antennaPan = 0;
	dirAnt_pub_.publish(dirAntMsg);
	
	// we have to wait because the sweeping occupies the arduino and we don't want to send servo or move commands while that is going on
	outdoor_bot::dirAnt_service::Request req;
   outdoor_bot::dirAnt_service::Response resp;
   int timeOut = 0;
   int previousDirAntSweepNumber = dirAntSweepNumber_;
   while (dirAntSweepNumber_ == previousDirAntSweepNumber && timeOut < 500 ) // wait until sweepNumber updates or we timeout at 5 seconds
   {
		bool success = dirAnt_client_.call(req,resp);
		if (success)
		{
			dirAntMaxAngle_ = resp.dirAntMaxAngle;
			dirAntSweepNumber_ = resp.dirAntSweepNumber;
			dirAntLevel_ = resp.dirAntLevel; 
		}
		else
		{
			cout << "directional antenna service failed" << endl;
			return false;
		}
		// no reason to check more frequently than the arduino sends data, so we can wait a bit
		ros::Time last_time = ros::Time::now(), current_time = ros::Time::now();
		while ( current_time.toSec() - last_time.toSec() < 0.02) 
		{
			ros::spinOnce();
			current_time = ros::Time::now(); // delay a bit
		}
		timeOut++;
	}	
	if (timeOut >= 500) return false;
	cout << "directional antenna: maxDir, level, sweep number = " << dirAntMaxAngle_ << ", " << dirAntLevel_ << ", " << dirAntSweepNumber_ << endl;
	return true;
}	
   
void on_enter_PhaseOneHomeState()
{

	// first move webcam up to level position
	cout << "sending command to move the webcam servo to level " << endl;
	tAF_.set_servoNumber(FRONT_WEBCAM_TILT);
	tAF_.set_servoDegrees( WEBCAM_TILT_LEVEL);
	tAF_.set_state(tAF_.getMoveCameraState());         		   
	while (tAF_.current_state() != tAF_.getAcquireDoneState()) 
	{
		tAF_.update();
		ros::spinOnce();
	}

	// second one
	cout << "sending the same command again" << endl;
	tAF_.set_servoNumber(FRONT_WEBCAM_TILT);
	tAF_.set_servoDegrees( WEBCAM_TILT_LEVEL);
	tAF_.set_state(tAF_.getMoveCameraState());         		   
	while (tAF_.current_state() != tAF_.getAcquireDoneState()) 
	{
		tAF_.update();
		ros::spinOnce();
	}
	
	outdoor_bot::movement_msg msg; 
	
	if (callAccelerometersService())
	{
		// make sure yaw is in the range -180 to 180
		while (yaw_ > 360.) yaw_ -= 360.;
		if (yaw_ > 180.) yaw_ -= 180.;
		while (yaw_ < - 360.) yaw_ += 360.;
		if (yaw_ < -180.) yaw_ += 180.;
		
		if (yaw_ > 0)		
		{
			msg.angle = 180 - yaw_;
			msg.speed = 20.;
		}
		else
		{
			msg.angle = -180 - yaw_;
			msg.speed = -20.;
		}
	}
	else
	{
		msg.angle = 180.;
		msg.speed = 20.;
	}
	movementComplete_ = false;	
	  
   cout << "turning toward home after retrieving target, angle = " << msg.angle << endl;
   msg.command = "autoMove";
   msg.distance = 0;  
   movement_pub_.publish(msg);
}

int on_update_PhaseOneHomeState()
{
	if (!movementComplete_) return PhaseOneHomeState_;
	// now look at radar and dirAnt data
	//
	//
	//usingDirAnt_ = updateDirectionalAntenna();
	// call jenny's routine here 
	// and turn us that amount
	
	// then we need to move forward, in a loop with the dirAnt and the radar distances
	// until we are within about 10 meters, when we can use the cameras
	
	return CheckHomeState_;
}

/*
void on_enter_PhaseTwoHomeState()
{
	outdoor_bot::movement_msg msg;
	usingDirAnt_ = updateDirectionalAntenna();
	if (usingDirAnt_)
	{
		msg.angle = dirAntMaxAngle_;
		msg.speed = 20 * sgn(maxAngle);
		msg.command = "autoMove";
   	msg.distance = 0;  
   	movementComplete_ = false;	
   	movement_pub_.publish(msg);
   }
   else movementComplete_ = true;
}
		
}

int on_update_PhaseTwoHomeState()
{
	if (!movementComplete_) return PhaseTwoHomeState_;	
	return CheckHomeState_;
}
*/

void on_enter_CheckHomeState()
{
   // start by seeing if we can get a radar range and angle
   //if (radarGoodData_ && radarGoodAngle_) return;
   
   // start by capturing an image using fsm
   currentSection_ = HOME;
   centerX_ = -1;
  // if (!triedWebcamAlready_)
   {
   	tAF_.set_acquireCamName(WEBCAM);
   	triedWebcamAlready_ = true;
   	lastCamName_ = WEBCAM;
   }
  /* else
   {
   	tAF_.set_acquireCamName(HOME_DIGCAM);
   	lastCamName_ = HOME_DIGCAM;
   }
   */
   tAF_.set_camCommand("cap_home");
   tAF_.set_homeTarget(true);
   tAF_.set_firstTarget(false);
   tAF_.set_state(tAF_.getCaptureImageState());
}

int on_update_CheckHomeState()
{  
   /*if (radarGoodData_) // && radarGoodAngle_)
   {
   	usingRadar_ = true;
   	return HeadForHomeState_;
   }
   */
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
      return SearchForHomeState_;
   }  
}

// we did not see the target in the camera image, so we need to move the camera and look again
void on_enter_SearchForHomeState()
{
   	deltaServoDegrees_ = -deltaServoDegrees_; // switch directions each time
		currentServoDegrees_ += deltaServoDegrees_ * searchCounter_;
		if (abs(currentServoDegrees_) > PAN_CAMERA_SEARCH_MAX) return; // can't find the target, so center the camera and then move ahead a bit
		searchCounter_++;
		tAF_.set_servoDegrees(currentServoDegrees_);
		
		tAF_.set_servoNumber(FRONT_WEBCAM_PAN);
		tAF_.set_acquireCamName(WEBCAM);
		tAF_.set_state(tAF_.getMoveCameraState());
}

int on_update_SearchForHomeState()
{
   if (abs(currentServoDegrees_) > PAN_CAMERA_SEARCH_MAX)
   {
      return HeadForHomeState_; // can't find the target, so move ahead a bit 
   }
   tAF_.update();
   if (tAF_.current_state() != tAF_.getAcquireDoneState()) return SearchForHomeState_; // waiting for servo delay
   return CheckHomeState_; // go see if there is a target in this camera direction
}

void on_enter_HeadForHomeState()
{
   /* if (usingRadar_)
    {
		moving_ = false;
		turning_ = false;
		alreadyTurned_ = false;
		pastStagingPoint_ = false;
		platforming_ = false;
		atPlatform_ = false; 
		orienting_ = false;
		alreadyOriented_ = false;
		parking_ = false;
		movementComplete_ = false;
		return;
	 }
	 */
   // if ((lastCamName_ == REGULAR_DIGCAM || lastCamName_ == ZOOM_DIGCAM ) && 
   if (abs(currentServoDegrees_) > 0.1) // need to put servo back to the center
   {
	   searchCounter_ = 0;
	   tAF_.set_servoDegrees(0);
	   tAF_.set_state(tAF_.getMoveCameraState());	
	   if (abs(currentServoDegrees_) > PAN_CAMERA_SEARCH_MAX) currentServoDegrees_ = 0;	// we don't want to center on this servo pan setting   
	} 
	triedWebcamAlready_ = false;
	moving_ = false;
	turning_ = false;
	alreadyTurned_ = false;
	movementComplete_ = false;
}

int on_update_HeadForHomeState()
{
   if (movementComplete_)
   {
   	movementComplete_ = false;
   	
   	usingRadar_ = false;
   	//if (usingRadar_)
   	{
   		// we need to let the radars gather data with the robot not moving
   		// so we wait a while between moves.  it takes 2 seconds to get brand new radar data
   		ros::Time last_time = ros::Time::now();
			ros::Time current_time = ros::Time::now();
			while ( current_time.toSec() - last_time.toSec() < 2.0) 
			{
				ros::spinOnce();
				current_time = ros::Time::now(); // delay a bit
			}
			radarNewData_ = false;
			last_time = ros::Time::now();
			current_time = ros::Time::now();
			while ( current_time.toSec() - last_time.toSec() < 4.0 && (!radarNewData_)) 
			{
				ros::spinOnce(); // check messages, wait for new data
				current_time = ros::Time::now(); // delay a bit
			}		
		
		}
		
   	if (moving_)
   	{
   		moving_ = false;
			if (usingRadar_) cout << "radar move finished, range to staging = " << distanceToRadarStagingPoint_ << endl;
		}
		else if (turning_)
		{
			turning_ = false;
    		cout << "turn finished, angle to staging = " << angleToRadarStagingPoint_ << endl;
    	}
    	
    	else if (orienting_)
    	{
    		orienting_ = false;
    		cout << "orientation turn finished, ";
    		if (radarGoodAngle_) cout << "angle to home = " << angleToHomeRadar_ << endl;
    		else cout << "but not getting radar localizations right now " << endl;
    	}
    	
    	else if (platforming_)
    	{
    		platforming_ = false;
    		cout << "move to the front of the platform finished, ";
    		if (radarGoodData_) cout << "range to home = " << distanceToHomeRadar_ << endl;
    		else cout << "but not getting radar range right now " << endl;
    	}
    	
    	else if (parking_)
    	{
    		parking_ = false;
    		cout << "parking move finished, ";
    		if (radarGoodData_) cout << "range to home = " << distanceToHomeRadar_ << endl;
    		else cout << "but not getting radar range right now " << endl;
    	}
    	if (radarGoodData_ && distanceToHomeRadar_ < PARKING_DISTANCE) 
		{
			cout << " we are in parking position, shut it down at a distance  = " << distanceToHomeRadar_ << " meters" << endl;
			return AllDoneState_;
		}
   }
   if (moving_ || turning_ || orienting_ || platforming_ || parking_)
   {
   	ros::spinOnce();
   	return HeadForHomeState_;
	}  
   // now move or turn
   outdoor_bot::movement_msg msg;
   
   /*if (usingRadar_)
   {
   	if ((!turning_) && (!alreadyTurned_) && (!pastStagingPoint_))
   	{
   		// send turn command to center the target
   		
   		usingDirAnt_ = true;
   		
   		if (usingDirAnt_) usingDirAnt_ = updateDirectionalAntenna();
   		if (usingDirAnt_)
   		{
   			cout << "compare directional antenna angle = " <<  dirAntMaxAngle_ << " to radar angle = " <<  angleToHomeRadar_ << endl;
   		}
			alreadyTurned_ = true;
			
			// untested change **************************************
			
   		// if we are too close to the staging point, we get large angles and don't want to go there.
   		if (distanceToRadarStagingPoint_ > 2. 
   			&& fabs(fabs(angleToRadarStagingPoint_) - fabs(angleToHomeRadar_) ) < 20. ) msg.angle = angleToRadarStagingPoint_;
   		else 
   		{
   			msg.angle = angleToHomeRadar_ * 1.2;	// go a little past, to maintain an approach intercept
   			cout << "we are too close to the staging point to turn there, we will instead turn for home with an angle of angleToHomeRadar * 1.2 = " << msg.angle << endl;
   		}
   		
   		// untested change **********************************
   		
   		
   		
   		if (fabs(msg.angle) < 5.  || (!radarGoodAngle_) || distanceToRadarStagingPoint_ < 1.) // no need to turn	// untested change ***********
   		{
		      if (fabs(msg.angle) > 90. && fabs(distanceToRadarStagingPoint_) < 8) pastStagingPoint_ = true;
		      turning_ = true;
		      movementComplete_ = true;
		      if (radarGoodAngle_) cout << " turn toward home was small enough to skip, it was = " << msg.angle << endl;
		      else cout << "missed a radar data point when about to turn, wait for another one" << endl;
		      return HeadForHomeState_;
         }            			 
      	msg.command = "autoMove";
      	msg.distance = 0.;
      	msg.speed = 20. * sgn(msg.angle);
   			// positive offset means turn to the left (ccw), negative to the right.
   		movement_pub_.publish(msg);
         turning_ = true;
         movementComplete_ = false;
   		cout << "using radar to turn toward staging point, turning " << angleToRadarStagingPoint_ << " degrees" << endl;
   		return HeadForHomeState_;
   	}	
	   // send move command to go to staging  point
   	else if ((!moving_) && (!pastStagingPoint_))
   	{
			alreadyTurned_ = false;
   		if (!radarGoodData_)
   		{
		      moving_ = true;
		      movementComplete_ = true;
		      cout << "missed a radar data point when about to move to stage point, wait for another one" << endl;
		      return HeadForHomeState_;
		   }
		   	
  			msg.command = "autoMove";
			msg.angle = 0;
			if (distanceToHomeRadar_ > RADAR_STAGING_POINT_DISTANCE + 4)
			{
				if (radarGoodAngle_) msg.distance = (distanceToRadarStagingPoint_ * 1000.)/ 2.;
				else msg.distance = ((distanceToHomeRadar_ - RADAR_STAGING_POINT_DISTANCE) * 1000.)/ 2.;
				if (msg.distance > 15000.) msg.distance = 15000.;	// we dont want to go too far in one move
				else if (msg.distance < -1000.) msg.distance = -1000.; // and no more than a meter in reverse
				msg.speed = 1000. * sgn(msg.distance);
			}
			else
			{
				if (distanceToRadarStagingPoint_ < 1.5)
				{
					pastStagingPoint_ = true;
					msg.distance = distanceToRadarStagingPoint_ * 1000.;
				}	
				else msg.distance = 1000.;			
				if (msg.distance < -1000.) msg.distance = -1000.; // don't move more than a meter in reverse
				msg.speed = 800.  * sgn(msg.distance);
			}		
			movement_pub_.publish(msg);
			moving_ = true;
			movementComplete_ = false;
			cout << "using radar to move toward staging point, moving " << msg.distance << " mm" << endl;
			if (pastStagingPoint_) cout << "this is the last move to the staging point" << endl;
			return HeadForHomeState_;
		}
		
		else if ((!orienting_) && (!alreadyOriented_) && radarGoodAngle_ && !parking_)
		// we are now close and have good radar data, so time to orient to the platform
		{
			// send turn command to orient the bot to the platform
   		msg.angle = (int) angleToHomeRadar_; 
   		if (fabs(msg.angle) < 3. || distanceToHomeRadar_ < PARKING_DISTANCE + 1.) // don't turn
   		{
		      orienting_ = true;
		      alreadyOriented_ = true;
		      movementComplete_ = true;
		      if (distanceToHomeRadar_ < PLATFORM_DISTANCE + 1.) cout << "too close to parking to turn, but we would have turned an angle = ";
		      else cout << " orienting to home was small enough to skip, it was = ";
		      cout << msg.angle << endl;
		      return HeadForHomeState_;
         }  
         else
         {
		   	msg.command = "autoMove";
		   	msg.distance = 0.;
		   	msg.speed = 20. * sgn(msg.angle);
				movement_pub_.publish(msg);
		      orienting_ = true;
		      alreadyOriented_ = true;
				pastStagingPoint_ = true;
		      movementComplete_ = false;
				cout << "using radar to orient to home platform, turning " << angleToHomeRadar_;
				cout << " degrees" << endl;
				return HeadForHomeState_;
			}
		}
		
		// move forward a bit until we are there
			alreadyOriented_ = false;
			msg.command = "autoMove";
			msg.angle = 0;
			if (distanceToHomeRadar_ > PLATFORM_DISTANCE && (!atPlatform_))
			{
				cout << " using radar to go to the front of the platform  = " << (distanceToHomeRadar_ - PLATFORM_DISTANCE) * 1000. << " mm" << endl;
				if ( distanceToHomeRadar_ - PLATFORM_DISTANCE < 1.0)
				{
					atPlatform_ = true;
					msg.distance = (distanceToHomeRadar_ - PLATFORM_DISTANCE) * 1000.;
				}
				else msg.distance = 1000.;
				platforming_ = true;
				msg.speed = 500. * sgn(msg.distance);
				movement_pub_.publish(msg);
				movementComplete_ = false;
				return HeadForHomeState_;
			}
			else if (distanceToHomeRadar_ > PARKING_DISTANCE)
			{
				cout << " we are moving onto the platform = " << (distanceToHomeRadar_ + 1.0 - PARKING_DISTANCE) * 1000. << " mm" << endl;
				msg.distance = (distanceToHomeRadar_ + 0.5 - PARKING_DISTANCE) * 1000.;
				msg.speed = 650. * sgn(msg.distance);
				parking_ = true;
				
			}
			else
			{
				if (radarGoodData_) 
				{
					cout << " we are in parking position, shut it down at a distance  = " << distanceToHomeRadar_ << " meters" << endl;
					return AllDoneState_;
				}
				else
				{
					cout << "missed a radar data point, wait for the next one" << endl;
					parking_ = true;
					movementComplete_ = true;
					return HeadForHomeState_;
				}					
			}				
			movement_pub_.publish(msg);
			movementComplete_ = false;
			return HeadForHomeState_;
			
		usingRadar_ = false;
		pastStagingPoint_ = false;
		return SearchForHomeState_; // something went wrong, look again for home		
	}
	*/  	


	if (centerX_ > 0 && (!turning_) && (!alreadyTurned_))	
	{
   	// center the target and move forward
   	offsetX_ = ((double) ((totalX_ / 2) - centerX_)) / ((double) totalX_);  // fraction that the image is off-center
    	centerX_ = -1;   	// only want to turn once for each pass through HeadForHomeState
   	cout << "center offset ratio = " << offsetX_ << endl;
   	if (lastCamName_ == WEBCAM) offsetX_ *= WEBCAM_FOV;
   	//************************************************************real ops put in HOME_DIGCAM_FOV
   	else if (lastCamName_ == REGULAR_DIGCAM) offsetX_ *= REGULAR_DIGCAM_ZOOM5_FOV;
   	else if (lastCamName_ == ZOOM_DIGCAM) offsetX_ *= ZOOM_DIGCAM_ZOOM6_FOV;
   	else offsetX_ = 0.;
   	cout << "center offset degrees = " << offsetX_ << endl;
   	double servoOffset = currentServoDegrees_;
   	currentServoDegrees_ = 0.;	// the servos were centered earlier, but we waited to reset currentServoDegrees_ so we could use it here
   	cout << "servo offset degrees = " << servoOffset << endl;
   	//double offsetY = ((double) (totalY_ - centerY_)) / ((double) totalY_);
   	offsetX_ += servoOffset;
   	cout << "total offset degrees = " << offsetX_;
   	if (fabs(offsetX_) > 3.)
   	{
   		// send turn command to center the target
      	msg.command = "autoMove";
      	msg.angle = offsetX_;
      	msg.speed = 20 * sgn(offsetX_);
      	msg.distance = 0.;
   			// positive offset means turn to the left, negative to the right.
   		movement_pub_.publish(msg);
         turning_ = true;
         movementComplete_ = false;
   		ROS_INFO("centering target");
   		return HeadForHomeState_;
   	}
	}
	alreadyTurned_ = true;
	turning_ = false;

	if (distanceToHomeRadar_ > 0.01)	range_ = distanceToHomeRadar_;
	else 
	{
		cout << "call to radar failed" << endl;    
		if (homeCameraRange_ > 1.0)  range_ = homeCameraRange_; // use optical range if radar fails
   	else 
   	{
   		cout << "unable to determine range in HeadForHome.  going back to CheckHomeState" << endl;
   		return CheckHomeState_;	// without any idea what the range is, we need to go back and check where we are
   	}
   }
   
   cout << "Heading for Home, range = " << range_ << endl;  
   moving_ = true;
   msg.command = "autoMove"; 
   movementComplete_ = false;
   
   if (range_ > 30.)
   {
      //move forward 10m 
   	msg.angle = 0;
   	msg.speed = 1000;
   	msg.distance = 10000.;
		
		movement_pub_.publish(msg);      
      ROS_INFO("moving forward 15m");
   }
   else if (range_ > 15.)
   {
      //move forward 5m
   	msg.angle = 0;
   	msg.speed = 1000;
   	msg.distance = 5000.;
		
		movement_pub_.publish(msg); 
      ROS_INFO("moving forward 5m");
   }
   else if (range_ > 10.)
   {
      //move forward 2 m
   	msg.angle = 0;
   	msg.speed = 1000;
   	msg.distance = 2000.;		
		movement_pub_.publish(msg); 
      ROS_INFO("moving forward 2m");
   }
   else if (range_ > 5)
   {
      //move forward 1 m
   	msg.angle = 0;
   	msg.speed = 1000;
   	msg.distance = 2000.;		
		movement_pub_.publish(msg); 
      ROS_INFO("moving forward 2m");
      
   }
   else if (range_ > 0.01)
   {
      // final approach
      cout << "moving to home spot, range_ = " << range_ << endl;
      currentSection_ = PLATFORM;
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
   return PauseState_;
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

void on_enter_UserCommandState()
{
   moving_ = false;
   turning_ = false;
   picking_ = false;
   movementComplete_ = false;	
}

int on_update_UserCommandState()
{
   if (pauseCommanded_)
   {
      previousState_ = UserCommandState_;	// recovery states should come back to here
      // note we do not update curentSection_, as we want to keep that value.
      // pause will return here as long as userCommandReceived_ = true;
      return PauseState_;
   }

   // check to see if we have arrived at the new pose   
   if ( (moving_ || turning_ || picking_) && (!movementComplete_ ))
   {
   	ros::spinOnce();
   	return UserCommandState_;  // move has not completed yet
   } 
   
   if (moving_ || turning_ || picking_)	//  move is complete
   {
   	moving_ = false;
		turning_ = false;
		picking_ = false;
		currentUserCommandNumber_++;
		userCmdNumDataValues_--;
   	if (userCmdNumDataValues_ == 0)
   	{
   		cout << "completed user commanded actions, total steps = " << currentUserCommandNumber_ << endl;
   		return PauseState_;
   	} 
   }
   
   outdoor_bot::movement_msg msg;
   
   cout << "doing user commanded action step " << currentUserCommandNumber_ << endl;
   cout << "distance, angle, speed, pickup: " << endl;
   cout << userCmdDistance_[currentUserCommandNumber_] << ", " << userCmdTurn_[currentUserCommandNumber_]; 
   cout << ", " << userCmdSpeed_[currentUserCommandNumber_] << ", " << userCmdPickup_[currentUserCommandNumber_] << endl;
   
   if (fabs(userCmdDistance_[currentUserCommandNumber_]) > 0.1)  // got a move command
   {
   	msg.command = "autoMove";
		msg.distance = userCmdDistance_[currentUserCommandNumber_];	// mm
		msg.angle = 0.;
		msg.speed = userCmdSpeed_[currentUserCommandNumber_];   	// mm/sec or deg/sec
		moving_ = true;
		movement_pub_.publish(msg);
	}
	else if (fabs(userCmdTurn_[currentUserCommandNumber_]) > 0.1)  // got a turn command
   {
   	msg.command = "autoMove";
		msg.distance = 0.;	// mm
		msg.angle = userCmdTurn_[currentUserCommandNumber_];
		msg.speed = userCmdSpeed_[currentUserCommandNumber_];   	// mm/sec or deg/sec
		turning_ = true;
		movement_pub_.publish(msg);		
	}  
	
	// if we are in here, we got a pickup command, which we have not coded yet, so just skip
	picking_ = true;
	movementComplete_ = true;
	
	return UserCommandState_; 		
}

void on_exit_UserCommandState()
{
	if (!pauseCommanded_)	// we enter these instructions while pause is on.  so the message gets sent with pause commanded
	{								// we want to keep returning here while all that is going on and then, when pause is released, 
		currentSection_ = userCmdReturnSection_;		// this section runs and then ends with on_exit, to pause, to the userCmdReturnSection
		userCommandReceived_ = false;
	}
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
   previousState_ = AllDoneState_;
   currentSection_ = ALL_DONE;	// so if we enter pause state via the telemetry switch, we come back to AllDoneState.
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
	msg.angle = 0.;
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
	msg.speed = 20. * sgn(msg.angle); 	// mm/sec or deg/sec 
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
   cout << "entering pause state, currentSection = " << currentSection_ << endl;

}
  
int on_update_PauseState()
{
   ros::spinOnce();
   if (pauseCommanded_) return PauseState_;  
   if (userCommandReceived_) return UserCommandState_;
   if (currentSection_ == BOOTUP) return BootupState_;
   if (currentSection_ == FIRST_TARGET_CHECK) return CheckFirstTargetState_;
   if (currentSection_ == FIRST_TARGET_MOVE) return MoveToFirstTargetState_;
   if (currentSection_ == TARGETS) return CheckTargetState_;
   if (currentSection_ == HOME) return CheckHomeState_;
   if (currentSection_ == PLATFORM) return MoveOntoPlatformState_;
   if (currentSection_ == ALL_DONE) return AllDoneState_;
   cout << "Pause did not recognize the current section, returning to CheckFirstTargetState" << endl;
   return CheckFirstTargetState_;  // something is seriously wrong if we get to here   
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
   PhaseTwoFirstState_ = fsm_.add_state("PhaseTwoFirstState");
   CheckFirstTargetState_ = fsm_.add_state("CheckFirstTargetState");  // 1
   SearchForFirstTargetState_ = fsm_.add_state("SearchForFirstTargetState");
   MoveToFirstTargetState_ = fsm_.add_state("MoveToFirstTargetState");
   AvoidObstacleState_ = fsm_.add_state("AvoidObstacleState");
   NewTargetState_ = fsm_.add_state("NewTargetState");
   CheckTargetState_ = fsm_.add_state("CheckTargetState");
   FindTargetState_ = fsm_.add_state("FindTargetState");
   PickupTargetState_ = fsm_.add_state("PickupTargetState");
   MoveToTargetState_ = fsm_.add_state("MoveToTargetState");
   PhaseOneHomeState_ = fsm_.add_state("PhaseOneHomeState");
   CheckHomeState_ = fsm_.add_state("CheckHomeState");
   SearchForHomeState_ = fsm_.add_state("SearchForHomeState");
   HeadForHomeState_ = fsm_.add_state("HeadForHomeState");
   MoveOntoPlatformState_ = fsm_.add_state("MoveOntoPlatformState");
   MoveToRecoverState_ = fsm_.add_state("MoveOntoRecoverState");
   MoveToRecoverFailedState_ = fsm_.add_state("MoveOntoRecoverFailedState");
   UserCommandState_ = fsm_.add_state("UserCommandState");
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

   fsm_.set_entry_function(PhaseTwoFirstState_, boost::bind(&on_enter_PhaseTwoFirstState));
   fsm_.set_update_function(PhaseTwoFirstState_, boost::bind(&on_update_PhaseTwoFirstState));
   //fsm_.set_exit_function(PhaseTwoFirstState_, boost::bind(&on_exit_PhaseTwoFirstState));
   
   fsm_.set_entry_function(CheckFirstTargetState_, boost::bind(&on_enter_CheckFirstTargetState));
   fsm_.set_update_function(CheckFirstTargetState_, boost::bind(&on_update_CheckFirstTargetState));
   //fsm_.set_exit_function(CheckFirstTargetState_, boost::bind(&on_exit_CheckFirstTargetState));

   fsm_.set_entry_function(SearchForFirstTargetState_, boost::bind(&on_enter_SearchForFirstTargetState));
   fsm_.set_update_function(SearchForFirstTargetState_, boost::bind(&on_update_SearchForFirstTargetState));
   //fsm_.set_exit_function(SearchForFirstTargetState_, boost::bind(&on_exit_SearchForFirstTargetState));

   fsm_.set_entry_function(MoveToFirstTargetState_, boost::bind(&on_enter_MoveToFirstTargetState));
   fsm_.set_update_function(MoveToFirstTargetState_, boost::bind(&on_update_MoveToFirstTargetState));
   //fsm_.set_exit_function(MoveToFirstTargetState_, boost::bind(&on_exit_MoveToFirstTargetState))

   fsm_.set_entry_function(AvoidObstacleState_, boost::bind(&on_enter_AvoidObstacleState));
   fsm_.set_update_function(AvoidObstacleState_, boost::bind(&on_update_AvoidObstacleState)); 	

   fsm_.set_entry_function(NewTargetState_, boost::bind(&on_enter_NewTargetState));
   fsm_.set_update_function(NewTargetState_, boost::bind(&on_update_NewTargetState));
   //fsm_.set_exit_function(NewTargetState_, boost::bind(&on_exit_NewTargetState));
   
   fsm_.set_entry_function(CheckTargetState_, boost::bind(&on_enter_CheckTargetState));
   fsm_.set_update_function(CheckTargetState_, boost::bind(&on_update_CheckTargetState));
   //fsm_.set_exit_function(CheckTargetState_, boost::bind(&on_exit_CheckTargetState));
   
   fsm_.set_entry_function(FindTargetState_, boost::bind(&on_enter_FindTargetState));
   fsm_.set_update_function(FindTargetState_, boost::bind(&on_update_FindTargetState));
   //fsm_.set_exit_function(FindTargetState_, boost::bind(&on_exit_FindTargetState));

   fsm_.set_entry_function(PickupTargetState_, boost::bind(&on_enter_PickupTargetState));
   fsm_.set_update_function(PickupTargetState_, boost::bind(&on_update_PickupTargetState));
   //fsm_.set_exit_function(PickupTargetState_, boost::bind(&on_exit_PickupTargetState));

   fsm_.set_entry_function(PhaseOneHomeState_, boost::bind(&on_enter_PhaseOneHomeState));
   fsm_.set_update_function(PhaseOneHomeState_, boost::bind(&on_update_PhaseOneHomeState));
   //fsm_.set_exit_function(PhaseOneHomeState_, boost::bind(&on_exit_PhaseOneHomeState));
   
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

   fsm_.set_entry_function(UserCommandState_, boost::bind(&on_enter_UserCommandState));
   fsm_.set_update_function(UserCommandState_, boost::bind(&on_update_UserCommandState));
   fsm_.set_exit_function(UserCommandState_, boost::bind(&on_exit_UserCommandState));
   
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
   
   obstacle_detector_ = new OutdoorBot::Navigation::ObstacleDetector;
	obstacle_avoider_ = new OutdoorBot::Navigation::WallFollower;
	

   //digcams_client_ = nh.serviceClient<outdoor_bot::digcams_service>("digcams_service");
   accelerometers_client_ = nh.serviceClient<outdoor_bot::accelerometers_service>("accelerometers_service");
   //encoders_client_ = nh.serviceClient<outdoor_bot::encoders_service>("encoders_service");
   setPose_client_ = nh.serviceClient<outdoor_bot::setPose_service>("setPose_service");
   NavTargets_client_ = nh.serviceClient<outdoor_bot::NavTargets_service>("NavTargetsService");
   //mainTargets_client_ = nh.serviceClient<outdoor_bot::mainTargets_service>("mainTargetsService");
   webcam_pub_ = nh.advertise<outdoor_bot::webcams_custom>("webcam_cmd", 25);
   digcam_pub_ = nh.advertise<outdoor_bot::digcams_custom>("digcam_cmd", 25);
   servo_pub_ = nh.advertise<outdoor_bot::servo_msg>("servo_cmd", 5);
   dirAnt_pub_ = nh.advertise<outdoor_bot::dirAnt_msg>("dirAnt_cmd", 5);  
   dirAnt_client_ = nh.serviceClient<outdoor_bot::dirAnt_service>("dirAnt_service"); 
   //pmotor_pub_ = nh.advertise<outdoor_bot::pmotor_msg>("pmotor_cmd", 5);
   movement_pub_ = nh.advertise<outdoor_bot::movement_msg>("movement_command", 2);
   mainTargetsCommand_pub_ = nh.advertise<outdoor_bot::mainTargetsCommand_msg>("mainTargets_cmd", 25);
   NavTargetsCommand_pub_ = nh.advertise<std_msgs::String>("NavTargets_cmd", 5);
   target_center_sub_ = nh.subscribe("target_center", 10, targetCenterCallback);
   home_center_sub_ = nh.subscribe("Home_target_center", 10, homeCenterCallback); 
   move_complete_sub_ = nh.subscribe("movement_complete", 4, moveCompleteCallback);
   pause_sub_ = nh.subscribe("pause_state", 4, pauseCallback);
   //image_sent_ = nh.subscribe("digcam_file", 50, imageSentCallback);
   mainTargetImageReceived_sub_ = nh.subscribe("main_target_image_received", 5, mainTargetImageReceivedCallback);
   navTargetImageReceived_sub_ = nh.subscribe("nav_target_image_received", 5, navTargetImageReceivedCallback);
   radar_sub_ = nh.subscribe("radar", 5, radarCallback);
   userDesignatedTargets_sub_ = nh.subscribe("user_commands", 2, userDesignatedTargetsCallback);

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
   
   currentSection_ = BOOTUP;
   fsm_.set_state(BootupState_);
   
      // now we keep updating and nicely move through the states
   
   ros::Rate updateRate(100); // update at 100 Hz
   while (nh.ok())
   {
      ros::spinOnce();
      fsm_.update();
      updateRate.sleep();
      //struct timespec ts;
      //ts.tv_sec = 0;
      //ts.tv_nsec = 10000000;
      //nanosleep(&ts, NULL); // update every 10 ms
      //cout << "state before update = " << fsm_.current_state() << endl;
      //cout << "current state = " << fsm_.current_state() << endl;
      //ros::Time last_time = ros::Time::now();
		//while ( ros::Time::now().toSec() - last_time.toSec() < 0.01) ros::spinOnce(); // delay a bit
   }

   cout << "all done" << endl;
   delete obstacle_avoider_;
   delete obstacle_detector_;
   return EXIT_SUCCESS;
}

