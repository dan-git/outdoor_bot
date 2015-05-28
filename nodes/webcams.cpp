#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "outdoor_bot/webcams_custom.h"
#include "outdoor_bot_defines.h"

#define MAX_NUM_WEBCAMS 2

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class webcam
{
   private:
      VideoCapture camera_;
      Mat frame_;
      double dWidth_; // the width of frames of the video
      double dHeight_;
      int camNum_;
      char windowTitle_[16];

	public:
		webcam() {}

		~webcam() {}

      bool initialize()
      {
         if (camera_.isOpened()) return true;
         camera_.open(camNum_);   
         if (camera_.isOpened())  printf("video cam %d opened \n", camNum_);
         else
         {
            printf("Cannot open video cam %d \n", camNum_);
            return false;
         }
         // restricted to these formats or the logitech C310 uses up so much USB bandwidth that
         // you can only use one webcam at a time
         camera_.set(CV_CAP_PROP_FRAME_WIDTH, 640);
         camera_.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

        dWidth_ = camera_.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
        dHeight_ = camera_.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
         printf("Frame size : %f x %f \n", dWidth_, dHeight_);
         return true;
      }

      void release()
      {
         if (camera_.isOpened()) camera_.release();
      }

      bool capture(Mat *image)
      {
        if (!initialize()) return false;
        bool bSuccess = camera_.read(frame_); // read a new frame from video
        if (!bSuccess) //if not success, break loop
        {
             printf("Cannot read a frame from video stream in camera %d \n", camNum_);
             return false;
        }
        *image = frame_;
        printf("Read a frame from video stream in camera %d \n", camNum_);
        return true;
      }

      bool grab()
      {
         bool bSuccess = camera_.grab(); // grab a new frame from video
        if (!bSuccess) //if not success, break loop
        {
             printf("Cannot grab a frame from video stream in camera %d \n", camNum_);
             return false;
        }
        printf("Grabbed a frame from video stream in camera %d \n", camNum_);
        return true;
      }

      bool retrieve(Mat *image)
      {
         bool bSuccess = camera_.retrieve(frame_); // grab a new frame from video
         if (!bSuccess) //if not success, break loop
         {
            printf("Cannot grab a frame from video stream in camera %d \n", camNum_);
            return false;
         }

         *image = frame_;
         printf("Grabbed a frame from video stream in camera %d \n", camNum_);
         return true;
      }         

      bool writeToFile(std::string filename)
      {
         if (frame_.empty()) return false;
         vector<int> compression_params; //vector that stores the compression parameters of the image
         compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique
         compression_params.push_back(98); //specify the compression quality
         bool bSuccess = imwrite(filename, frame_, compression_params); //write the image to file
         if (bSuccess) 
         {
            ROS_INFO("image written to file");
            return true;
         }
         else ROS_ERROR("error in writing image to file");
         return false;
      }

      void createWindow()
      {        
        sprintf(windowTitle_, "webcam %d", camNum_);
        namedWindow(windowTitle_,CV_WINDOW_AUTOSIZE); //create a window
      }

      void show()
      {
        imshow(windowTitle_, frame_); //show the frame
        waitKey(30); //wait to give highGUI time to process
      } 

      void closeWindow()
      {
        destroyWindow(windowTitle_); //destroy the window
      }

      int getCamNum() { return camNum_; }
      void setCamNum(int value) { camNum_ = value; }    
    };

class webcamControl
{
   private:
      ros::NodeHandle nh_;
      ros::Publisher filename_pub_;
      image_transport::ImageTransport it_;
      image_transport::Publisher image_pub_, home_image_pub_;
      unsigned int webcamTilt_;
      int numWebcams_;
      int numImages_[MAX_NUM_WEBCAMS];
      webcam camera_[MAX_NUM_WEBCAMS];
      bool cap_home_;

      void publishFilename(const char *filename)
      {
         std_msgs::String msg;
         msg.data = filename;
         filename_pub_.publish(msg);
      }
     
   public:
      webcamControl(ros::NodeHandle &nh)
      : nh_(nh), it_(nh)
      {
         ros::Subscriber webcam_cmd = nh.subscribe("webcam_cmd", 50, &webcamControl::cameraCommandCallback, this); 
         filename_pub_ = nh_.advertise<std_msgs::String>("webcam_file", 50); // advertise camera messages
         home_image_pub_ = it_.advertise("home_target_image", 1); // advertise home target images
         image_pub_ = it_.advertise("webcam_image", 1);
         numWebcams_ = 0;
         cap_home_ = false;
         for (int i=0; i < MAX_NUM_WEBCAMS; i++) numImages_[i] = 0;
         openAllWebcams();
         //releaseAllWebcams();
      }
    
      int openAllWebcams()
      {
         for (numWebcams_ = 0; numWebcams_ < MAX_NUM_WEBCAMS; numWebcams_++)
         {
            camera_[numWebcams_].setCamNum(numWebcams_);
            if (!camera_[numWebcams_].initialize()) break;
         }
         if (numWebcams_ > 0) ROS_INFO("%d webcams detected", numWebcams_);
         else ROS_ERROR("no webcams detected");         
         return numWebcams_;
      }

      void releaseAllWebcams()
      {
         for (int i=0; i < numWebcams_; i++) camera_[i].release();
      }

      void openWebcam(int camNum)
      {
         if (camNum <= numWebcams_ && camNum >= 0)
         {
            camera_[camNum].initialize();
         }
      }

      void releaseWebcam(int camNum)
      {
          if (camNum <= numWebcams_ && camNum >= 0)
         {
            camera_[camNum].release();
         }
      }        


      bool capture(int camNumber)
      {
         if (camNumber > numWebcams_ || camNumber < 0)
         {
         	cout << "invalid camera number requested for capture in webcams, camera number = " << camNumber << endl;
         	return false;
         }
         char filename[80];
         sprintf(filename, "webcam%d", camNumber);
         Mat image;
         if (camera_[camNumber].capture(&image))
         {
            std::string camType = "webcam";
            std_msgs::Header imgHeader;
            imgHeader.seq = webcamTilt_;
            imgHeader.frame_id = camType;
            imgHeader.stamp = ros::Time::now();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(imgHeader, "bgr8", image).toImageMsg();
            //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            if (cap_home_)
		   	{
		   		home_image_pub_.publish(msg);  // home target image
		   		ROS_INFO("webcams published a home image");
		   		return true;
		   	}
		   	else 
		   	{
		   		image_pub_.publish(msg);   // publish the image
		   		ROS_INFO("webcams published a target image");
      			return true;
      		}
      	}
      	return false;
      }

      void createWindow(int camNumber)
      {
         if (camNumber > numWebcams_ || camNumber < 0) return;
         camera_[camNumber].createWindow();
      }

      void show(int camNumber)
      {
         if (camNumber > numWebcams_ || camNumber < 0) return;
         camera_[camNumber].show();
      }

      void closeWindow(int camNumber)
      {
         if (camNumber > numWebcams_ || camNumber < 0) return;
         camera_[camNumber].closeWindow();
      }

      void writeToFile(int camNumber, std::string filename)
      {
         if (camNumber > numWebcams_ || camNumber < 0) return;
         if (camera_[camNumber].writeToFile(filename))
         {
            std_msgs::String msg_name;
            msg_name.data = filename;       
            filename_pub_.publish(msg_name);  // publish the filename
         }
      }

      void cameraCommandCallback(const outdoor_bot::webcams_custom::ConstPtr &msg)
      {
         std::string cameraCommand = msg->command; 
         int camNum = msg->camera_number + LAPTOP_HAS_BUILTIN_WEBCAM;
         cout << "webcam command received for camera number " << camNum << endl;
         if ( (!cameraCommand.compare("cap_and_rel")) || (!cameraCommand.compare("capture")) || (!cameraCommand.compare("cap_home")) )
         {
            if (!cameraCommand.compare("cap_home")) cap_home_ = true;
            else cap_home_ = false;
            webcamTilt_ = msg->tilt;
            capture(camNum);
            if (msg->write_file)
            {
               std::stringstream buffer;
               buffer << "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/webcam/webcam" << camNum
                  << "_" << numImages_[camNum] << ".jpg";
               numImages_[camNum]++;
               std::string filenm = buffer.str();
               writeToFile(camNum, filenm);
            }              
            //if (!cameraCommand.compare("cap_and_rel")) releaseWebcam(camNum); *** note that commenting this out means there is no difference between capture vs cap_and_rel
            releaseWebcam(camNum);
         }
         else if (!cameraCommand.compare("release")) releaseWebcam(camNum);
         else if (!cameraCommand.compare("open")) openWebcam(camNum);
      }


      /*
      void cameraCommandCallback(const std_msgs::String::ConstPtr& msg)
      {
         std::string cameraCommand = msg->data.c_str();
         char filename[256];
         strcpy(filename, msg->data.c_str());
         for (int i=0; i < numWebcams_; i++) capture(i);    
      }
      */
};


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter(ros::NodeHandle &nh)
    : nh_(nh), it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

      
int main(int argc, char* argv[])
{
   //Initialize ROS
   ros::init(argc, argv, "webcams");
   ros::NodeHandle nh;
   ROS_INFO("webcams starting");
   webcamControl wcControl(nh);
   //ImageConverter ic(nh);   // cannot run both the video stream and webcam control on the same camera
   //if (wcControl.openAllWebcams() == 0) return 1;
  
   // subscribe to webcam commands
   ros::Subscriber webcam_cmd;
   webcam_cmd = nh.subscribe("webcam_cmd", 50, &webcamControl::cameraCommandCallback, &wcControl);

   /*
   if (wcControl.capture(0))
   {
      ROS_INFO("webcam0 captured a frame");
      //wcControl.createWindow(0);
      //wcControl.show(0);
      //std::string filenm = "/home/dbarry/TestImage0.jpg";
      //wcControl.writeToFile(0, filenm);
   }
   else ROS_ERROR("webcam0 capture failed"); 

   if (wcControl.capture(1))
   {
      ROS_INFO("webcam1 captured a frame");
      //wcControl.createWindow(1);
      //wcControl.show(1);
      //std::string filenm = "/home/dbarry/TestImage1.jpg";
      //wcControl.writeToFile(1, filenm);
   }
   else ROS_ERROR("webcam1 capture failed");
   */
   /*
   while(nh.ok())
   {
   	ros::spinOnce();  // check for incoming messages
   	struct timespec ts;
      ts.tv_sec = 0;
      ts.tv_nsec = 10000000;
      nanosleep(&ts, NULL); // update every 10 ms
   }
   */
   ros::spin();
   //wcControl.closeWindow(0);
   return EXIT_SUCCESS;
}
