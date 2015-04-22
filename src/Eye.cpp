#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, const char** argv )
{
     Mat img = imread("shot-0000.jpg", CV_LOAD_IMAGE_UNCHANGED); //read the image data in the file "MyPic.JPG" and store it in 'img'

     if (img.empty()) //check whether the image is loaded or not
     {
          cout << "Error : Image cannot be loaded..!!" << endl;
          //system("pause"); //wait for a key press
          return -1;
     }

     namedWindow("MyWindow", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"
     imshow("MyWindow", img); //display the image which is stored in the 'img' in the "MyWindow" window

     waitKey(0); //wait infinite time for a keypress

     destroyWindow("MyWindow"); //destroy the window with the name, "MyWindow"

     return 0;
}

/*
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "cv.h"
//#include "highgui.h"
//#include "cvcam.h"


class Eye
{
	public:
		Eye(int EyeNum, int EyeIndex);
		virtual ~Eye();
		//wxString GetWindowName() { return m_WindowName; }
		int GetEyeNum() { return m_EyeNum; }
		void Display();
		void Format();
		void GetProperties(double *FrameWidth, double *FrameHeight);
		//void SetProperties(double FrameWidth, double FrameHeight);
		//wxString Description();
		void SetResolution(int Width, int Height);
		IplImage *GetLastImage() { return m_Image; }
		IplImage *QueryFrame();
		void ShowLastImage();

	private:
		const char* m_WindowName;
		CvCapture* m_pCapture;
		IplImage *m_Image;
		int m_EyeNum;
};


Eye::Eye(int EyeNum, int EyeIndex)
{
	m_EyeNum = EyeNum;

	//if (EyeIndex == 0) m_WindowName = _T("Upward ");
	//else if (EyeIndex == 1) m_WindowName = _T("Forward ");
	//else { m_WindowName = _T(""); m_WindowName << EyeIndex; }
	m_WindowName = " Eye";
	cvNamedWindow(m_WindowName, CV_WINDOW_AUTOSIZE);
   //cv::namedWindow(OPENCV_WINDOW);
   m_pCapture = cvCaptureFromCAM(EyeNum);
   m_Image = QueryFrame();   // get a first image
   ShowLastImage();  // and display it
}

Eye::~Eye()
{
	//cvDestroyWindow(m_WindowName.mb_str());
	cvReleaseCapture(&m_pCapture);
}

void Eye::Format()
{
	//cvcamGetProperty(m_EyeNum, CVCAM_VIDEOFORMAT, NULL);
}

void Eye::GetProperties(double *FrameWidth, double *FrameHeight)
{
	*FrameWidth = cvGetCaptureProperty(m_pCapture,CV_CAP_PROP_FRAME_WIDTH);
	*FrameHeight = cvGetCaptureProperty(m_pCapture,CV_CAP_PROP_FRAME_HEIGHT);
	//cvcamGetProperty(m_EyeNum, CVCAM_CAMERAPROPS, NULL);
}
*/
/*
void Eye::SetProperties(double FrameWidth, double FrameHeight)
{
   cvSetCaptureProperty(m_pCapture, CV_CAP_PROP_FRAME_WIDTH, FrameWidth);
   cvSetCaptureProperty(m_pCapture, CV_CAP_PROP_FRAME_WIDTH, FrameHeight);
}
*/

/*
wxString Eye::Description()
{
	//char Data[256];
	//wxString TempString = _T("");
	//cvcamGetProperty(m_EyeNum, CVCAM_DESCRIPTION, Data);
	//for (int i=0; i < 256; i++) TempString << Data[i];
	//return TempString;
}
*/
/*
void Eye::SetResolution(int Width, int Height)
{
	//cvcamSetProperty(m_EyeNum, CVCAM_RNDWIDTH, &Width);
	//cvcamSetProperty(m_EyeNum, CVCAM_RNDHEIGHT, &Height);
}
*/
// note that to get images from multiple cameras simultaneously, we should write two other methods
// one that uses cvGrabFrame() and the other that uses cvRetrieveFrame()
// then we first grab the frames from all the cameras and then go
// retrieve them.  if we do both grab and retrieve on each camera in turn, then the images
// will not be very simultaneous because cvRetrieveFrame takes a long time
// note that cvQueryFrame combines cvGrabFrame and cvRetrieveFrame into one command
/*
IplImage *Eye::QueryFrame()
{
   for (int i=0; i < 8; i++) cvGrabFrame(m_pCapture); // it takes a few images to get to the newest one
   m_Image = cvRetrieveFrame(m_pCapture);
   ShowLastImage();
   return m_Image;
}

void Eye::ShowLastImage()
{
	cvShowImage(m_WindowName, m_Image);
}

int main (int argc, char *argv[])
{
   ros::init(argc, argv, "image_converter");
   Eye test(0,0);
   ros::spin();
   return 0;
}

*/
