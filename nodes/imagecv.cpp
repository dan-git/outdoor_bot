#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Header.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "outdoor_bot/mainTargetsCommand_msg.h"
#include "outdoor_bot/NavTargets_msg.h"
#include "outdoor_bot_defines.h"

//#include <iostream>
//#include "outdoor_bot/digital_cams_custom.h"

using namespace cv;
using namespace std;

class image_cvproc
{
private:
   ros::NodeHandle nh_;
   ros::Publisher filename_pub_, navTargetsCommand_pub_, mainTargetsCommand_pub_;
   std::string camFilename_;  
   image_transport::ImageTransport it_;
	image_transport::Publisher home_image_pub_, digcam_image_pub_, webcam_image_pub_;
   image_transport::Subscriber subWebcam_;
   image_transport::Subscriber subDigcam_;
   image_transport::Subscriber subHomecam_;
   ros::Subscriber digcam_file_sub_;
   ros::Subscriber webcam_file_sub_;
   Mat fileImage_;
   int digcamImageNumber_, webcamImageNumber_;

public:
   image_cvproc(ros::NodeHandle &nh)
   :  nh_(nh), it_(nh), digcamImageNumber_(0), webcamImageNumber_(0)
   {
      filename_pub_ = nh_.advertise<std_msgs::String>("camera_file", 50); // advertise camera files
      home_image_pub_ = it_.advertise("home_target_image", 5); 
      digcam_image_pub_ = it_.advertise("digcam_image", 5); 
      webcam_image_pub_ = it_.advertise("webcam_image", 5); 
      mainTargetsCommand_pub_ = nh.advertise<outdoor_bot::mainTargetsCommand_msg>("mainTargets_cmd", 25);
      navTargetsCommand_pub_ = nh.advertise<std_msgs::String>("NavTargets_cmd", 5);
      subDigcam_ = it_.subscribe("digcam_image", 5, &image_cvproc::digcamImageCallback, this);
      subWebcam_ = it_.subscribe("webcam_image", 5, &image_cvproc::webcamImageCallback, this);
      subHomecam_ = it_.subscribe("home_target_image", 5, &image_cvproc::homecamImageCallback, this);
      webcam_file_sub_ = nh.subscribe("webcam_file", 50, &image_cvproc::cameraFileCallback, this); 
      digcam_file_sub_ = nh.subscribe("digcam_file", 50, &image_cvproc::cameraFileCallback, this);
   }

   void digcamImageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
     ROS_INFO("digcam image received by image_cvproc");
     Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
     Mat resizedImg;
     Size dsize(0,0); //round(fx*src.cols), round(fy*src.rows))}
     resize(img, resizedImg, dsize, 0.25, 0.25, CV_INTER_AREA);
     string windowName = "digcam";
     /*
     if (digcamImageNumber_ == 1)
     {
		   cv::destroyWindow("digcam");
   		windowName = "digcam_left";
     }
     if (digcamImageNumber_ > 1)
     {
     	  if (digcamImageNumber_ == 2) cv::destroyWindow("digcam_left");
     	  windowName = "digcam";
     }
     */
     try
     {
        cv::imshow(windowName, resizedImg);
        //cv::waitKey(0);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("In digcamImageCallback, could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
     digcamImageNumber_++;
   }
   
   void webcamImageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
     ROS_INFO("webcam image received by image_cvproc");
     string windowName = "webcam";
     std_msgs::Header imgHeader = msg->header;
     /*
     if (webcamImageNumber_ == 1)
     {
     		cv::destroyWindow("webcam_front");
     		windowName = "webcam_rear";
     }
     if (webcamImageNumber_ > 1)
     {
     		 //if (webcamImageNumber_ == 2) cv::destroyWindow("webcam_rear");
     		 windowName = "webcam";
     }
     */
     try
     {
        cv::imshow(windowName, cv_bridge::toCvShare(msg, "bgr8")->image);
        //cv::waitKey(0);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("In webcamImageCallback, could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
     webcamImageNumber_++;
   }
   
   void homecamImageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
     ROS_INFO("homecam image received by image_cvproc");
     Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
     //Mat resizedImg;
     //Size dsize(0,0); //round(fx*src.cols), round(fy*src.rows))}
     //resize(img, resizedImg, dsize, 0.25, 0.25, CV_INTER_AREA);
     string windowName = "homecam";
     try
     {
        cv::imshow(windowName, img);
        //cv::waitKey(0);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("In homecamImageCallback, could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
   }
 

   void cameraFileCallback(const std_msgs::String::ConstPtr& msg)
   {
      ROS_INFO("image filename received by image_cvproc");
      camFilename_ = msg->data.c_str();  
   }

   bool readImageFile(std::string filename)
   {
      fileImage_ = imread(filename, CV_LOAD_IMAGE_UNCHANGED); 
      if (fileImage_.empty()) //check whether the image is loaded or not
      {
          ROS_ERROR("Error : readImageFile failed to read an image");
          return false;
      }
      return true;

   }

   bool writeToFile(std::string filename, Mat image)
   {
      if (image.empty()) return false;
      vector<int> compression_params; //vector that stores the compression parameters of the image
      compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique
      compression_params.push_back(98); //specify the compression quality
      bool bSuccess = imwrite(filename, image, compression_params); //write the image to file
      if (bSuccess) 
      {
         std_msgs::String msg;
         msg.data = filename;
         filename_pub_.publish(msg);   // publish the filename
         ROS_INFO("image written to file");
         return true;
      }
      else ROS_ERROR("error in writing image to file");
      return false;
   }

   void publishHomeImage(Mat image)
   {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      home_image_pub_.publish(msg);
   }
   
   void publishWebcamImage(Mat image)
   {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      webcam_image_pub_.publish(msg);
   }
   
   void publishDigcamImage(Mat image)
   {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      digcam_image_pub_.publish(msg);
   }
		
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
	
   Mat getImage() { return fileImage_; }
   
   void analyzeImage(int camName, double approxRange, bool firstTarget, bool homeTarget)
   {
   
      if (homeTarget)
      {
      	cout << "sending command to analyze image for home target" << endl;
      	//newNavTargetImageReceived_ = false;
      	std_msgs::String msg;
      	msg.data = "home";
      	navTargetsCommand_pub_.publish(msg); 
      }
      else
      {
	      cout << "sending command to mainTargets to analyze image" << endl;
	      //if (newMainTargetDigcamImageReceived_ && (camName_ == REGULAR_DIGCAM || camName == ZOOM_DIGCAM) newMainTargetDigcamImageReceived_ = false;
	      //if (newMainTargetWebcamImageReceived_ && camName_ == WEBCAM) newMainTargetWebcamImageReceived_ = false;
	      outdoor_bot::mainTargetsCommand_msg msg; 
	      msg.cameraName = camName;
	      msg.firstTarget = firstTarget; 
	      msg.approxRange = approxRange; 
	      mainTargetsCommand_pub_.publish(msg);
	   }
	}
	
};

class userImageCommands
{
private:
  ros::NodeHandle nh_;
  ros::Publisher userImageCmd_pub_;

  string userCommand_;
  bool userInputReceived_;
  std_msgs::String userString_;
  
public:
	userImageCommands(ros::NodeHandle &nh)
	: nh_(nh)
	{
		//set the callback function for any mouse event
      //setMouseCallback("user_image", userImageCommands::userCallBack, NULL);
	}
	
	void getUserInput()
	{
		string input = "";
		userCommand_ = "";
		int comComplete = 0;

		cout << "Hit any key to display latest image" << endl;
		getline(cin, input);
		cout << "Hit any key to start drawing an ROI" << endl;
		userCommand_.append(input);
	
		cout << "Another command (0) or Finish up (1)?" << endl;
		getline(cin, input);
		// This code converts from string to number safely.
		stringstream myStream(input);
		if ( !(myStream >> comComplete) )
		{
			cout << "Failed to understand comComplete, finishing up" << endl;
			comComplete = 1;
			return;
		}
		 
		 
		 cout << "Return section choices are as follows: " << endl;
		 cout << "0 = BOOT, 1 = FIRST_TARGET_CHECK, 2 = FIRST_TARGET_MOVE, 3 = TARGETS, 4 = HOME, 5 = PLATFORM, 6 = ALL_DONE" << endl;
		 cout << "Enter return section: " << endl;
		 int sectionSelect;
		 getline(cin, input);
		 // check for valid entry
		 stringstream mySectionStream(input);
		 if (mySectionStream >> sectionSelect)
		 {
		 	if (sectionSelect < 0 || sectionSelect > 6)
			{
				cout << "invalid return section, defaulting to TARGETS" << endl;
			 	userCommand_.append("3");
			}
			else userCommand_.append(input);
		 }
		 else
		 {
			 cout << "Failed to understand return section, defaulting to TARGETS" << endl;
			 userCommand_.append("3");
		 }

		 userCommand_.append(";");
		 
		 cout << "command string = " << userCommand_ << endl;
	}

	static void userCallBack(int event, int x, int y, int flags, void* userdata)
	{
		 if  ( event == EVENT_LBUTTONDOWN )
		 {
		     cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		 }
		 else if  ( event == EVENT_RBUTTONDOWN )
		 {
		     cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		 }
		 else if  ( event == EVENT_MBUTTONDOWN )
		 {
		     cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		 }
		 else if ( event == EVENT_MOUSEMOVE )
		 {
		     cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
		 }    
	}
};



int main(int argc, char* argv[])
{
   ros::init(argc, argv, "image_processor");
   ros::NodeHandle nh;

   image_cvproc ic(nh);
   userImageCommands uIC(nh);
   
      
   //cv::namedWindow("zoom_digcam"); 
   //cv::namedWindow("digcam_left");
   //cv::namedWindow("webcam_front");
   //cv::namedWindow("webcam_rear");
   cv::namedWindow("digcam");
   cv::namedWindow("webcam");
   cv::namedWindow("homecam");
  // cv::namedWindow("user_image");
   cv::startWindowThread();
   
   
   //setMouseCallback("user_image", uIC.userCallBack, NULL);  // this is now setup inside the class

   
       // Read image from file 
   //string filenm = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam/zoom_digcam_zoom7_5m_area_31054.jpg";   
  // if (ic.readImageFile(filenm))
 
      
 
    //show the image
    //imshow("user_image", ic.getImage());
 
    // Wait until user press some key
   // waitKey(0);
/*
   // test sending images
   bool homeTarget = false;
   bool firstTarget = true;
   
   string filenm = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam/zoom_digcam_zoom7_5m_area_31054.jpg";   
   ic.readImageFile(filenm);
   int camName = ZOOM_DIGCAM;
   double approxRange = 5.0;
   cout << "ready to publish?" << endl;
   ic.askUser();		// have to have a delay in here or the message does not publish     
   ic.publishDigcamImage(ic.getImage());
   cout << "ready to analyze?" << endl;
   ic.askUser();	
   ic.analyzeImage(camName, approxRange, firstTarget, homeTarget);
   ros::spinOnce();
   
   filenm = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/webcam/webcam_white_1_1m_area_4000.jpg";
   ic.readImageFile(filenm);
   camName = WEBCAM;
   approxRange = 1.0;
   cout << "ready to publish?" << endl;
   ic.askUser();		// have to have a delay in here or the message does not publish     
   ic.publishWebcamImage(ic.getImage());
   cout << "ready to analyze?" << endl;
   ic.askUser();	
   ic.analyzeImage(camName, approxRange, firstTarget, homeTarget);
   ros::spinOnce();
         
   filenm = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam/digcam__white_zoom7_5m_area_11747.jpg";
   ic.readImageFile(filenm);
   camName = REGULAR_DIGCAM;
   approxRange = 5.0;
   cout << "ready to publish?" << endl;
   ic.askUser();		// have to have a delay in here or the message does not publish     
   ic.publishDigcamImage(ic.getImage());
   cout << "ready to analyze?" << endl;
   ic.askUser();	
   ic.analyzeImage(camName, approxRange, firstTarget, homeTarget);
   ros::spinOnce();
   
*/

/*
   cv_bridge::CvImagePtr cv_ptr;   // CvImagePtr is an opencv format 
   // might need the headers, don't know, otherwise where do these parameters come from?
   //msg->encoding = "bgr8";
   //msg->height = 200;
   //msg->width = 200;

   try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); // converts from ROs to opencv format, makes a copy so we can alter it
   } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
   }

   cv::Mat &mat = cv_ptr->image;  // we can get the opencv Mat format from the CvImage format
    
   // we can get the ROS image publishing formatfrom it too
   sensor_msgs::ImagePtr imageForPublishingROS = cv_ptr->toImageMsg();

   pub.publish(imageForPublishingROS);
*/


/*
  while(nh.ok())
   {
   	uIC.getUserInput();	// we block here if there are no user inputs
   	//cmds.printCommands();
   	//cmds.publishCommands();
   	ros::spinOnce();  // check for incoming messages
   	//struct timespec ts;
      //ts.tv_sec = 0;
      //ts.tv_nsec = 10000000;
      //nanosleep(&ts, NULL); // update every 10 ms
   }
   */
   ros::spin();
 
	cv::destroyWindow("webcam");
	cv::destroyWindow("digcam");
	cv::destroyWindow("homecam");
//	cv::destroyWindow("user_image");
   return EXIT_SUCCESS;
}
