#ifndef _STEREOMATCHING_H
#define _STEREOMATCHING_H

#include <opencv2/opencv.hpp>
#include <iostream>

#if CV_MAJOR_VERSION==3
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#endif

#include "../include/DefStruct.h"

using namespace std;
using namespace cv;


#if CV_MAJOR_VERSION==3
using namespace cv::ximgproc;
#endif

typedef enum error { NO_PROB, IMGSIZE_ERR, IMGSCALE_ERR } MATCHING_ERROR;
/**
	@class CStereoMatching
	@brief disparity map
*/
class CStereoMatching{
private:
	//----------------input-------------------
	Mat m_imgLeftInput;		// rectified image
	Mat m_imgRightInput;	// rectified image

#if CV_MAJOR_VERSION==3
	//BM based disparity
	Ptr<StereoBM> bm = StereoBM::create(0, 0); // default construct
	//SGBM based disparity
	//Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 0, 0);

	Ptr<DisparityWLSFilter> wls_filter;
	Rect m_rectFilterROI;
#endif
#if CV_MAJOR_VERSION == 2
	StereoBM bm;
#endif

	StereoCamParam_t m_objStereoParam;
	MATCHING_ERROR MakeDisparity();

public:

	//----------------output-------------------
	Mat m_matDisp16;
	Mat m_imgDisp8;

	//---------------function------------------
	CStereoMatching(StereoCamParam_t& objStereoParam);
	CStereoMatching(Mat& imgLeftInput, Mat& imgRightInput, StereoCamParam_t& objStereoParam);
	
	//Set param
	void SetParamOCVStereo(StereoCamParam_t& objStereoParam);

	//make disparity
	MATCHING_ERROR SetImage(Mat& imgLeft, Mat& imgRight);
	MATCHING_ERROR MakeDisparity(Mat& imgLeft, Mat& imgRight, bool flgUseWLSFilter=true);
	MATCHING_ERROR MakeDisparity(Mat& imgLeft, Mat& imgRight, Mat& matDisp16);
	MATCHING_ERROR ImproveDisparity_Naive(Mat& imgDisp8);
	MATCHING_ERROR ImproveDisparity_WLSFilter(Mat& imgDisp8); ///< OCV310 new disparity postprocess

};

#endif 