/**
@file StereoMatcing.cpp
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com).
@brief create disparity map
*/

#include "../include/StereoMatching.h"
// Ptr<StereoBM> bm = StereoBM::create(48, 9); ///< default construct

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

#if CV_MAJOR_VERSION==3
	bm->create(m_objStereoParam.m_nNumberOfDisp, m_objStereoParam.m_nWindowSize);

	bm->setPreFilterCap(31);
	bm->setBlockSize(m_objStereoParam.m_nWindowSize > 0 ? m_objStereoParam.m_nWindowSize : 9);
	bm->setMinDisparity(0);
	bm->setNumDisparities(m_objStereoParam.m_nNumberOfDisp);
	bm->setTextureThreshold(10);		/// SAD window response threshold : default=12
	bm->setUniquenessRatio(5);			/// >(match_val - min_match)/min_match
	bm->setSpeckleWindowSize(100);		//25;//9;
	bm->setSpeckleRange(32);			//4;
	//bm->setSmallerBlockSize(9);
	//bm->setDisp12MaxDiff(1);
#else
	bm.state->preFilterCap = 31;
	bm.state->SADWindowSize = m_objStereoParam.m_nWindowSize > 0 ? m_objStereoParam.m_nWindowSize : 9;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities = m_objStereoParam.m_nNumberOfDisp;
	bm.state->textureThreshold = 10;		//SAD window response threshold : default=12
	bm.state->uniquenessRatio = 15;		// > (match_val - min_match)/min_match
	bm.state->speckleWindowSize = 100;//25;//9;
	bm.state->speckleRange = 32;//4;
	//bm.state->disp12MaxDiff = 1;
#endif

}
MATCHING_ERROR CStereoMatching::SetImage(Mat& imgLeft, Mat& imgRight){
	if (imgLeft.size() != imgRight.size()) return IMGSCALE_ERR;
	if (imgLeft.size() != m_objStereoParam.objCamParam.m_sizeSrc) return IMGSCALE_ERR;
	if (imgLeft.channels() == 3){
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
    // Code to create disparity maps out of
    std::cout<<"Disparity 3: Call Function "<<std::endl;
	MakeDisparity(m_imgLeftInput, m_imgRightInput, m_matDisp16);
	std::cout<<"Disparity 5: Convert from 16 to 8"<<std::endl;
	m_matDisp16.convertTo(m_imgDisp8, CV_8U, 255 / (m_objStereoParam.m_nNumberOfDisp*16.));

	//Mat m_imgDisp8_ori;
	//m_imgDisp8_ori = m_imgDisp8.clone();
	//imshow("disp", m_imgDisp8);
    //std::cout<<"Reached Make Disparity Map"<<std::endl;
    //waitKey();
	return NO_PROB;
}
MATCHING_ERROR CStereoMatching::MakeDisparity(Mat& imgLeft, Mat& imgRight, bool flgUseWLSFilter)
{
#if CV_MAJOR_VERSION==2
	flgUseWLSFilter = false;
#endif
	MATCHING_ERROR Error;
	Error = SetImage(imgLeft, imgRight);
	//cout << Error << endl;
	if (flgUseWLSFilter == false){
		std::cout<<"Disparity 2: Function call Inside Stereo Matching"<<std::endl;
		MakeDisparity();
        //Code Used to create disparity
        std::cout<<"Disparity 6: Improve Disparity"<<std::endl;
		ImproveDisparity_Naive(m_imgDisp8);

	}
#if CV_MAJOR_VERSION==3
	else if (flgUseWLSFilter == false){
	}
	// There is matching code inside the WLSFilter(), It is
	else ImproveDisparity_WLSFilter(m_imgDisp8);
#endif
	return Error;
}
MATCHING_ERROR CStereoMatching::MakeDisparity(Mat& imgLeft, Mat& imgRight, Mat& matDisp16){
#if CV_MAJOR_VERSION==3
	// cout << "SADWindowSize : " << bm->getBlockSize() << endl;
	// cout << "NumOfDisparity : " << bm->getNumDisparities() << endl;
	std::cout<<"Disparity 4: Apply BM algorithm"<<std::endl;
	bm->compute(imgLeft, imgRight, matDisp16);
	// cout << "complete" << endl;
#else
	bm(imgLeft, imgRight, matDisp16, CV_16S);
#endif
	m_matDisp16 = matDisp16;
	return NO_PROB;
}
MATCHING_ERROR CStereoMatching::ImproveDisparity_Naive(Mat& imgDisp8)
{
	uchar chTempCur = 0;
	uchar chTempPrev = 0;

	int cnt = 1;
	for (int v = 0; v < imgDisp8.rows; v++){
		for (int u = m_objStereoParam.m_nNumberOfDisp; u < imgDisp8.cols; u++){
			chTempCur = imgDisp8.at<uchar>(v, u);
			//shTempCur = m_matDisp16.at<short>(v, u);
			if (chTempCur == 0) {
				imgDisp8.at<uchar>(v, u) = chTempPrev;
				//m_matDisp16.at<short>(v, u) = shTempPrev;
			}
			else {
				chTempPrev = chTempCur;
				//shTempPrev = shTempCur;
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
#if CV_MAJOR_VERSION==3
	Mat matDispLeft16;
	Mat matDispRight16;
	/*Mat conf_map = Mat(m_imgLeftInput.rows, m_imgLeftInput.cols, CV_8U);
	conf_map = Scalar(255);*/

	wls_filter = createDisparityWLSFilter(bm);
	Ptr<StereoMatcher> right_bm = createRightMatcher(bm);

	wls_filter->setLambda(8000.);
	wls_filter->setSigmaColor(1.5);

	bm->compute(m_imgLeftInput, m_imgRightInput, matDispLeft16);
	right_bm->compute(m_imgRightInput, m_imgLeftInput, matDispRight16);
	wls_filter->filter(matDispLeft16, m_imgLeftInput, m_matDisp16, matDispRight16);
	/*conf_map = wls_filter->getConfidenceMap();

	m_rectFilterROI = wls_filter->getROI();*/

	m_matDisp16.convertTo(imgDisp8, CV_8U, 255 / (m_objStereoParam.m_nNumberOfDisp*16.));
#endif
	return NO_PROB;
}
