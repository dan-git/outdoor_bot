/**
 * OpenCV SimpleBlobDetector Example
 *
 * Copyright 2015 by Satya Mallick <spmallick@gmail.com>
 *
 */

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{

	// Read image
	//Mat im = imread( "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/blob.jpg", CV_LOAD_IMAGE_GRAYSCALE );
   //Mat im = imread( "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/firstTargetCropped.jpg", CV_LOAD_IMAGE_GRAYSCALE );
   //Mat im = imread( "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/whiteCircle.jpg", CV_LOAD_IMAGE_GRAYSCALE );
   //Mat im = imread( "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/blackCircles.jpg", CV_LOAD_IMAGE_GRAYSCALE );
   //Mat im = imread("/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0.jpg", CV_LOAD_IMAGE_GRAYSCALE );
   //Mat im = imread("/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0_scaled.jpg", CV_LOAD_IMAGE_GRAYSCALE );
   //Mat im = imread("/home/dbarry/Dropbox/outdoor_bot/media/image_processing/targetWithCircles.jpg", CV_LOAD_IMAGE_GRAYSCALE );

   Mat im_original = imread("/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0_scaled.jpg", CV_LOAD_IMAGE_UNCHANGED);
   //Mat im = imread("/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0.jpg", CV_LOAD_IMAGE_UNCHANGED);
   //Mat im = imread("/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/200mm_crouching.JPG", CV_LOAD_IMAGE_UNCHANGED);
   //Mat im = imread("/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/300_mm_crouching.JPG", CV_LOAD_IMAGE_UNCHANGED); 
   //Mat im = imread("/home/dbarry/Dropbox/outdoor_bot/media/photos/300mm_at_WPI/300mm_crouching _with_tree_near_target.JPG", CV_LOAD_IMAGE_UNCHANGED);

	imshow("keypoints", im_original);
   waitKey(0);

   // create a working copy
   Mat im = im_original.clone();
   CvSize sz_ = cvSize(im_original.cols, im_original.rows);
   
   // down-scale and upscale the image to filter out some noise
   Mat pyr(cvSize(sz_.width/2, sz_.height/2), CV_8UC3); // depth 8, 3 channels for downscaling
   pyrDown( im, pyr, cvSize(sz_.width/2, sz_.height/2)  );
   pyrUp( pyr, im, sz_ );

   vector<Mat> planes_BGR;
   split(im, planes_BGR); // split into separate color planes (B G R)
   
   Mat timg_B = planes_BGR[0];
   Mat timg_G = planes_BGR[1];
   Mat timg_R = planes_BGR[2];

   // leave out the green, so we don't see grass
   Mat im_BR;
   addWeighted( timg_B, 0.5, timg_R, 0.5, 0.0, im_BR);
	imshow("keypoints", im_BR );
   waitKey(0);

   // threshold a bit
   threshold(im_BR, im_BR, 200, 255, CV_THRESH_BINARY);
	imshow("keypoints", im_BR );
   waitKey(0);

   // can reduce noise a bit here with an erosion
   //erode(im_BR, im_BR, Mat(), Point(-1,-1), 1);

   // invert the image to get it ready for blob detection
   bitwise_not(im_BR, im_BR);
	imshow("keypoints", im_BR );
   waitKey(0);

   // reduce noise with dilation
   dilate(im_BR, im_BR, Mat(), Point(-1,-1), 1);
	imshow("keypoints", im_BR );
   waitKey(0);

   // Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 100;  // note that this is inverted, it only detects black blobs on white backgrounds s
	params.maxThreshold = 255;  // and I think the thresholds are wrt to local background.

	// Filter by Area.
	params.filterByArea = true;
	//params.minArea = 7327; // 7327 corresponds to a size value (~radius?) of 47.8009 for the keypoint
                           // if that was a radius, it would correspond to an area of 7178.3
                           // even a radius of 48 corresponds to an area of only 7238.2
   long totalArea = im.cols * im.rows;
   cout << "total area = " << totalArea << endl;
   params.minArea = 100; //totalArea / 500;
   params.maxArea = totalArea / 4;

	// Filter by Circularity
	params.filterByCircularity = false;
	params.minCircularity = 0.1;

	// Filter by Convexity
	params.filterByConvexity = false;
	params.minConvexity = 0.87;

	// Filter by Inertia
	params.filterByInertia = false;
	params.minInertiaRatio = 0.01;


	// Storage for blobs
	vector<KeyPoint> keypoints;


	#if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2

		// Set up detector with params
		SimpleBlobDetector detector(params);

		// Detect blobs
		detector.detect( im_BR, keypoints);
	#else 

		// Set up detector with params
		Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);   

		// Detect blobs
		detector->detect( im_BR, keypoints);
	#endif 

   cout << "Found " << keypoints.size() << " keypoints" << endl;
	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
	// the size of the circle corresponds to the size of blob

	//Mat im_with_keypoints;
	//drawKeypoints( im_BR, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

	// Show blobs
   double maxKeypointArea = -1.;
   int maxIndex;
   for (int i=0; i < keypoints.size(); i++)
   {
      double keypointArea = keypoints[i].size * keypoints[i].size * 3.14;
      cout << "keypoint center: " << keypoints[i].pt << endl;
      cout << "keypoint radius: " << keypoints[i].size << endl;
      cout << "keypoint area: " <<  keypointArea << endl;
      cout << "keypoint response: " << keypoints[i].response << endl;
      cout << "keypoint angle: " << keypoints[i].angle << endl;
      cout << "keypoint class_id: " << keypoints[i].class_id << endl;
      cout << "keypoint octave: " << keypoints[i].octave << endl;

      if (keypointArea > maxKeypointArea)
      {
         maxKeypointArea = keypointArea;
         maxIndex = i;
      }
    }

    if (maxKeypointArea > 0 )
    {
      cout << endl << endl << "Possible target found: " << endl;
      cout << "Target center: " << keypoints[maxIndex].pt << endl;
      cout << "Target radius: " << keypoints[maxIndex].size << endl;
      cout << "Target area: " <<  maxKeypointArea << endl;
      cout << "Target response: " << keypoints[maxIndex].response << endl;
      cout << "Target angle: " << keypoints[maxIndex].angle << endl;
      cout << "Target class_id: " << keypoints[maxIndex].class_id << endl;
      cout << "Target octave: " << keypoints[maxIndex].octave << endl;
      Point keyCenter((int)keypoints[maxIndex].pt.x, (int) keypoints[maxIndex].pt.y);
      circle(im_original, keyCenter, 50, Scalar(0,0,255), 4, 8, 0);
	   imshow("keypoints", im_original );
	   waitKey(0);
    }

}

