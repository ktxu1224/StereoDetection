#include "../detection/StereoMatching.h"

CStereoMatching::CStereoMatching(StereoCamParam_t& objStereoParam)
{
	SetParamOCVStereo(objStereoParam);
}

CStereoMatching::CStereoMatching(Mat& imgLeftInput, Mat& imgRightInput, StereoCamParam_t& objStereoParam)
{
	SetParamOCVStereo(objStereoParam);
	m_imgLeftInput = imgLeftInput;
	m_imgRightInput = imgRightInput;
}

void CStereoMatching::SetParamOCVStereo(StereoCamParam_t& objStereoParam)
{
	m_objStereoParam = objStereoParam;

	/*stereoPtr->create(m_objStereoParam.m_nNumberOfDisp, m_objStereoParam.m_nWindowSize);
	stereoPtr->setPreFilterCap(31);
	stereoPtr->setBlockSize(m_objStereoParam.m_nWindowSize > 0 ? m_objStereoParam.m_nWindowSize : 9);
	stereoPtr->setMinDisparity(0);
	stereoPtr->setNumDisparities(m_objStereoParam.m_nNumberOfDisp);
	stereoPtr->setTextureThreshold(10);
	stereoPtr->setUniquenessRatio(10);
	stereoPtr->setSpeckleWindowSize(300);
	stereoPtr->setSpeckleRange(32);
	stereoPtr->setDisp12MaxDiff(1);*/

	m_objStereoParam.m_nWindowSize = 9;
	int p1 = 8 * m_objStereoParam.m_nWindowSize*m_objStereoParam.m_nWindowSize;
	int p2 = 32 * m_objStereoParam.m_nWindowSize*m_objStereoParam.m_nWindowSize;

	stereoPtr->create(0, m_objStereoParam.m_nNumberOfDisp, m_objStereoParam.m_nWindowSize);
	stereoPtr->setP1(p1);
	stereoPtr->setP2(p2);
	stereoPtr->setPreFilterCap(25);
	stereoPtr->setUniquenessRatio(10);
	stereoPtr->setSpeckleRange(2);
	stereoPtr->setSpeckleWindowSize(400);
	stereoPtr->setDisp12MaxDiff(1);
	stereoPtr->setMode(cv::StereoSGBM::MODE_SGBM);

}

MATCHING_ERROR CStereoMatching::SetImage(Mat& imgLeft, Mat& imgRight) {
	if (imgLeft.size() != imgRight.size())
		return IMGSCALE_ERR;
	if (imgLeft.size() != m_objStereoParam.objCamParam.m_sizeSrc)
		return IMGSCALE_ERR;
	if (imgLeft.channels() == 3) {
		cvtColor(imgLeft, m_imgLeftInput, CV_BGR2GRAY);
		cvtColor(imgRight, m_imgRightInput, CV_BGR2GRAY);
		return NO_PROB;
	}
	m_imgLeftInput = imgLeft;
	m_imgRightInput = imgRight;

	return NO_PROB;
}

MATCHING_ERROR CStereoMatching::MakeDisparity()
{
	MakeDisparity(m_imgLeftInput, m_imgRightInput, m_matDisp16);
	m_matDisp16.convertTo(m_imgDisp8, CV_8U, 255 / (m_objStereoParam.m_nNumberOfDisp*16.));

	return NO_PROB;
}

MATCHING_ERROR CStereoMatching::MakeDisparity(Mat& imgLeft, Mat& imgRight, bool flagUseWLSFilter)
{

	MATCHING_ERROR Error = SetImage(imgLeft, imgRight);

	if (flagUseWLSFilter == false) {
		MakeDisparity();
	}
	else
		ImproveDisparity_WLSFilter(m_imgDisp8);

	return Error;
}

MATCHING_ERROR CStereoMatching::MakeDisparity(Mat& imgLeft, Mat& imgRight, Mat& matDisp16) {
	stereoPtr->compute(imgLeft, imgRight, matDisp16);
	m_matDisp16 = matDisp16;

	return NO_PROB;
}

MATCHING_ERROR CStereoMatching::ImproveDisparity_Naive(Mat& imgDisp8)
{
	uchar chTempCur = 0;
	uchar chTempPrev = 0;

	int cnt = 1;
	for (int v = 0; v < imgDisp8.rows; v++) {
		for (int u = m_objStereoParam.m_nNumberOfDisp; u < imgDisp8.cols; u++) {
			chTempCur = imgDisp8.at<uchar>(v, u);
			if (chTempCur == 0) {
				imgDisp8.at<uchar>(v, u) = chTempPrev;
			}
			else {
				chTempPrev = chTempCur;
			}
		}
	}
	Size size(7, 7);
	Mat matKernel = getStructuringElement(MORPH_RECT, size);

	morphologyEx(imgDisp8, imgDisp8, MORPH_OPEN, matKernel, Point(-1, -1), 2);

	return NO_PROB;

}

MATCHING_ERROR CStereoMatching::ImproveDisparity_WLSFilter(Mat& imgDisp8)
{
	Mat matDispLeft16;
	Mat matDispRight16;

	wls_filter = createDisparityWLSFilter(stereoPtr);
	Ptr<StereoMatcher> right_matcher = createRightMatcher(stereoPtr);
	stereoPtr->compute(m_imgLeftInput, m_imgRightInput, matDispLeft16);
	right_matcher->compute(m_imgRightInput, m_imgLeftInput, matDispRight16);
	wls_filter->setLambda(8000.);
	wls_filter->setSigmaColor(1.5);
	wls_filter->filter(matDispLeft16, m_imgLeftInput, m_matDisp16, matDispRight16);

	m_matDisp16.convertTo(imgDisp8, CV_8U, 255 / (m_objStereoParam.m_nNumberOfDisp*16.));

	return NO_PROB;
}