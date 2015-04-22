#ifndef EYE_H
#define EYE_H

//****************************************************************************#include "wx.h"
//#include "enums.h"
//#include <wx/wx.h>
//#include <wx/string.h>

#ifndef OPENCV_INCLUDES
#define OPENCV_INCLUDES
#include "cv.h"
#include "highgui.h"
//#include "cvcam.h"
#endif // OPENCV_INCLUDES

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
		//wxString m_WindowName;
		CvCapture* m_pCapture;
		IplImage *m_Image;
		int m_EyeNum;
};

#endif // EYE_H
