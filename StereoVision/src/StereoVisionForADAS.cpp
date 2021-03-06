#include "../detection/StereoVisionForADAS.h"


StereoCamParam_t CStereoVisionForADAS::InitStereoParam(int nDatasetName)
{
	StereoCamParam_t objStereoCamParam;

	//左边为准
	if (nDatasetName == MYDATA)
	{
		objStereoCamParam.m_dBaseLine = 0.56;
		objStereoCamParam.m_dMaxDist = 60;
		objStereoCamParam.m_nNumberOfDisp = 80;
		objStereoCamParam.m_nWindowSize = 15;
		objStereoCamParam.objCamParam.m_dCameraHeight = 1.39;
		objStereoCamParam.objCamParam.m_dFocalLength_X = 1383.4;
		objStereoCamParam.objCamParam.m_dFocalLength_Y = 1374.2;
		objStereoCamParam.objCamParam.m_dOx = 798.16;
		objStereoCamParam.objCamParam.m_dOy = 628.67;
		objStereoCamParam.objCamParam.m_dPitchDeg = 0.011;//0.5;
		objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(1571, 900);
	}
	else if (nDatasetName == KITTI)
	{
		objStereoCamParam.m_dBaseLine = 0.54;
		objStereoCamParam.m_dMaxDist = 60.;
		objStereoCamParam.m_nNumberOfDisp = 80;
		objStereoCamParam.m_nWindowSize = 11;
		objStereoCamParam.objCamParam.m_dCameraHeight = 1.65;
		objStereoCamParam.objCamParam.m_dFocalLength_X = 721.54;
		objStereoCamParam.objCamParam.m_dFocalLength_Y = 721.54;
		objStereoCamParam.objCamParam.m_dPitchDeg = -1.;
		objStereoCamParam.objCamParam.m_dYawDeg = -1.2;
		objStereoCamParam.objCamParam.m_dOx = 609.56;
		objStereoCamParam.objCamParam.m_dOy = 172.85;
		objStereoCamParam.objCamParam.m_sizeSrc = cv::Size(1242, 375);
	}
	else
		printf("This DB is not availabe. sorry");

	PitchDegToVanishingLine(objStereoCamParam);

	return objStereoCamParam;
}

//calculate VanishingY
int CStereoVisionForADAS::PitchDegToVanishingLine(StereoCamParam_t& objStereoParam)
{
	objStereoParam.objCamParam.m_nVanishingY = (int)(objStereoParam.objCamParam.m_dFocalLength_Y*tan(objStereoParam.objCamParam.m_dPitchDeg*PI / 180)) + objStereoParam.objCamParam.m_dOy;

	return 0;
}

void CStereoVisionForADAS::MakePseudoColorLUT()
{
	int b = 125;
	int g = 0;
	int r = 0;

	int idx = 0;

	int mode = 0;

	while (1)
	{
		m_pseudoColorLUT[idx][0] = b;
		m_pseudoColorLUT[idx][1] = g;
		m_pseudoColorLUT[idx][2] = r;

		if (b == 255 && g == 0 && r == 0)
			mode = 1;
		else if (b == 255 && g == 255 && r == 0)
			mode = 2;
		else if (b == 0 && g == 255 && r == 0)
			mode = 3;
		else if (b == 0 && g == 255 && r == 255)
			mode = 4;
		else if (b == 0 && g == 0 && r == 255)
			mode = 5;

		switch (mode)
		{
		case 0: b += 5; break;
		case 1: g += 5; break;
		case 2: b -= 5; break;
		case 3: r += 5; break;
		case 4: g -= 5; break;
		case 5: r -= 5; break;
		default: break;
		}

		if (idx == 255)
			break;

		idx++;
	}
}
void CStereoVisionForADAS::cvtPseudoColorImage(Mat& srcGray, Mat& dstColor)
{
	for (int i = 0; i < srcGray.rows; i++)
	{
		for (int j = 0; j < srcGray.cols; j++)
		{
			unsigned char val = srcGray.data[i*srcGray.cols + j];
			if (val == 0) continue;
			dstColor.data[(i*srcGray.cols + j) * 3 + 0] = m_pseudoColorLUT[val][0];
			dstColor.data[(i*srcGray.cols + j) * 3 + 1] = m_pseudoColorLUT[val][1];
			dstColor.data[(i*srcGray.cols + j) * 3 + 2] = m_pseudoColorLUT[val][2];
		}
	}

}

CStereoVisionForADAS::CStereoVisionForADAS(StereoCamParam_t& objStereoParam)
	:m_objStereoMatching(objStereoParam), m_objStixelEstimation(objStereoParam), m_objStixelSegmentation(objStereoParam)
{
	m_objStereoParam = objStereoParam;
	MakePseudoColorLUT();
	m_imgColorDisp = Mat(objStereoParam.objCamParam.m_sizeSrc, CV_8UC3);
	m_imgColorDisp = Scalar(0);
	m_imgGround = Mat(objStereoParam.objCamParam.m_sizeSrc, CV_8UC1);
	m_imgStixelGray = Mat(objStereoParam.objCamParam.m_sizeSrc, CV_8UC1);
	m_imgStixelGray = Scalar(0);
}

int CStereoVisionForADAS::Objectness(Mat& imgLeft, Mat& imgRight)
{
	m_vecobjBB.clear();
	m_vecobjStixelInROI.clear();
	m_vecobjStixels.clear();

	if (imgLeft.channels() == 3) {
		cvtColor(imgLeft, m_imgLeftInput, CV_BGR2GRAY);
		cvtColor(imgRight, m_imgRightInput, CV_BGR2GRAY);
	}
	else {
		m_imgLeftInput = imgLeft;
		m_imgRightInput = imgRight;
	}

	m_objStereoMatching.MakeDisparity(m_imgLeftInput, m_imgRightInput, false);
	m_matDisp16 = m_objStereoMatching.m_matDisp16;
	m_imgDisp8 = m_objStereoMatching.m_imgDisp8;
	imshow("disparity.bmp", m_imgDisp8);
	//waitKey();

	m_imgGround = Scalar(0);
	m_objStixelEstimation.EstimateStixels(m_matDisp16, m_imgDisp8);
	m_imgGround = m_objStixelEstimation.m_imgGround;
	m_vecobjStixelInROI = m_objStixelEstimation.m_vecobjStixelInROI;
	m_vecobjStixels = m_objStixelEstimation.m_vecobjStixels;

	m_objStixelSegmentation.SegmentStixel(m_vecobjStixels);
	m_vecobjBB = m_objStixelSegmentation.m_vecobjBB;

	return 0;

}

int CStereoVisionForADAS::Objectness(Mat& imgLeft, Mat& imgRight, Mat& imgDisp8)
{
	m_vecobjBB.clear();
	m_vecobjStixelInROI.clear();
	m_vecobjStixels.clear();

	if (imgLeft.channels() == 3) {
		cvtColor(imgLeft, m_imgLeftInput, CV_BGR2GRAY);
		cvtColor(imgRight, m_imgRightInput, CV_BGR2GRAY);
	}
	else
	{
		m_imgLeftInput = imgLeft;
		m_imgRightInput = imgRight;
	}
	m_imgDisp8 = imgDisp8;

	m_imgGround = Scalar(0);
	m_objStixelEstimation.EstimateStixels_only8bitDisp(m_imgDisp8);
	m_imgGround = m_objStixelEstimation.m_imgGround;
	m_vecobjStixelInROI = m_objStixelEstimation.m_vecobjStixelInROI;
	m_vecobjStixels = m_objStixelEstimation.m_vecobjStixels;

	m_objStixelSegmentation.SegmentStixel(m_vecobjStixels);
	m_vecobjBB = m_objStixelSegmentation.m_vecobjBB;

	return 0;
}

int CStereoVisionForADAS::RectToDisp(Rect& rectBox, Mat& matRect)
{
	if (m_matDisp16.empty()) { cout << "There is no disparity image" << endl; return -1; }
	if (rectBox.x < 0 || rectBox.x + rectBox.width > m_matDisp16.cols || rectBox.y < 0 || rectBox.y + rectBox.height > m_matDisp16.rows)
	{
		cout << "out of range" << endl;
		return -1;
	}
	Mat imgDisp8;
	m_matDisp16.convertTo(imgDisp8, CV_8U, 1 / 16.);
	matRect = imgDisp8(rectBox).clone();
	return 0;
}
int CStereoVisionForADAS::Disp16ToDepth(const uchar nDisparity, float& fDistanceMeter)
{
	fDistanceMeter = (float)m_objStereoParam.m_dBaseLine*(float)m_objStereoParam.objCamParam.m_dFocalLength_X / (float)nDisparity;
	return 0;
}

void CStereoVisionForADAS::Display()
{
	TopViewStixel(m_vecobjStixelInROI);
}
void CStereoVisionForADAS::Display(Mat& imgDisplay)
{
#ifndef _DEBUG
	Display();
#endif
	for (unsigned int i = 0; i < m_vecobjBB.size(); i++) {
		rectangle(imgDisplay, m_vecobjBB[i].rectBB, Scalar::all(255), 2, 8);
	}
}
void CStereoVisionForADAS::Display(Mat& imgDisplay, Mat& imgStixelResult)
{
	//if (imgDisplay.channels() == 3) cvtColor(imgDisplay, imgDisplay, CV_BGR2GRAY);
	if (imgStixelResult.channels() == 3) cvtColor(imgStixelResult, imgStixelResult, CV_BGR2GRAY);

	float fBrightness = 70;

	//Display(imgDisplay);
	for (unsigned int i = 0; i < m_vecobjBB.size(); i++) {
		rectangle(imgDisplay, m_vecobjBB[i].rectBB, Scalar(255, 0, 0), 2, 8);
	}

	m_imgStixelGray = Scalar(0);

	DrawGround(m_imgColorDisp, m_imgGround);
	line(m_imgColorDisp, Point(0, m_objStereoParam.objCamParam.m_nVanishingY), Point(m_imgColorDisp.cols, m_objStereoParam.objCamParam.m_nVanishingY), Scalar(255, 255, 255), 3);
	DrawStixel(m_imgColorDisp, m_vecobjStixels);
	DrawLane(m_imgColorDisp, m_objStereoParam);
	DrawStixel(m_imgColorDisp, m_vecobjStixelInROI);

	cvtColor(m_imgLeftInput, imgStixelResult, CV_GRAY2BGR);
	addWeighted(imgStixelResult, 0.4, m_imgColorDisp, 0.6, 0., imgStixelResult);
	for (unsigned int i = 0; i < m_vecobjBB.size(); i++) {
		rectangle(imgStixelResult, m_vecobjBB[i].rectBB, Scalar::all(255 - m_vecobjBB[i].dZ / fBrightness * 250), 2, 8);
		char temp[20];
		sprintf_s(temp, sizeof(temp), "%.2fm", m_vecobjBB[i].dZ);
		putText(imgStixelResult, temp, m_vecobjBB[i].rectBB.br() - Point(m_vecobjBB[i].rectBB.width, 0), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255 - m_vecobjBB[i].dZ / fBrightness * 250), 2);
	}


}
void CStereoVisionForADAS::DrawLane(Mat& imgResult, StereoCamParam_t& objStereoParam)
{
	if (objStereoParam.objCamParam.m_dCameraHeight == 1.17) {
		///////////////////////////////temp///////////////// daimler
		line(imgResult,
			Point(340, 216),//(int)(31 / 40 * 340) - 47),
			Point(640, 449),//(int)(31 / 40 * 640) - 47),
			Scalar(0, 255, 255), 5
		);
		line(imgResult,
			Point(300, 216),
			Point(0, 449),
			Scalar(0, 255, 255), 5
		);
	}
}

void CStereoVisionForADAS::DrawStixel(Mat& imgResult, vector<stixel_t>& vecobjStixels)
{
	for (unsigned int u = 0; u < vecobjStixels.size(); u++) {
		line(m_imgStixelGray,
			Point(vecobjStixels[u].nCol, vecobjStixels[u].nGround),
			Point(vecobjStixels[u].nCol, vecobjStixels[u].nHeight),
			Scalar(vecobjStixels[u].chDisparity));
	}
	cvtPseudoColorImage(m_imgStixelGray, imgResult);
	m_imgStixelGray.setTo(0);
}

void CStereoVisionForADAS::DrawGround(Mat& imgResult, Mat& imgGround)
{
	threshold(imgGround, imgGround, 0, 255, CV_THRESH_BINARY);
	cvtColor(imgGround, imgResult, CV_GRAY2BGR);
	Mat imgTemp = imgResult.clone();
	imgTemp = Scalar(0, 75, 150);
	imgResult = imgResult & imgTemp;
}

void CStereoVisionForADAS::TopViewStixel(vector<stixel_t>& objStixelInROI)
{
	Mat imgTopView(500, m_imgLeftInput.cols, CV_8UC3, Scalar::all(0));

	int nScale = 10; // nScale^-1 meter

	TopViewLane(imgTopView, 3., imgTopView.cols / 2);

	for (int i = 0; i < 500; i += 5 * nScale)
	{
		line(imgTopView,
			Point(0, i),
			Point(imgTopView.cols, i),
			Scalar(100, 100, 100));
	}

	//Top view drawing
	//cout << m_vecobjStixels.size() << endl;
	for (unsigned int u = 0; u < m_vecobjStixels.size(); u++) {
		if (m_vecobjStixels[u].chDisparity == 0) continue;
		//int nDistance = (int)(m_nFocalLength*m_dBaseLine*nScale / ((double)m_vecobjStixels[u].chDisparity*m_nNumberOfDisp/255));
		int nDistance = (int)m_vecobjStixels[u].dZ * nScale;
		if (nDistance < 500) imgTopView.at<Vec3b>(500 - nDistance, m_vecobjStixels[u].nCol) = Vec3b(0, 0, 255);//Vec3b(0, 0, 10*(double)nDistance*m_vecobjStixels[u].nHeight/(double)m_nFocalLength);
	}
	for (unsigned int u = 0; u < objStixelInROI.size(); u++) {
		int nDistance = (int)objStixelInROI[u].dZ * nScale;
		if (nDistance < 500) imgTopView.at<Vec3b>(500 - nDistance, objStixelInROI[u].nCol) = Vec3b(255, 255, 0);//Vec3b(0, 0, 10*(double)nDistance*m_vecobjStixels[u].nHeight/(double)m_nFocalLength);
	}

	imgTopView.copyTo(m_imgTopView);

}
void CStereoVisionForADAS::TopViewLane(Mat& imgTopView, float fLaneInterval, int nCenterPointX)
{
	float fLaneIntervalPixel = 0;
	for (int i = 1; i < imgTopView.rows; i++)
	{
		fLaneIntervalPixel = (float)(m_objStereoParam.objCamParam.m_dFocalLength_X)*(float)fLaneInterval / ((float)(500 - i) / 10);
		if (fLaneIntervalPixel < m_imgLeftInput.cols)
		{
			imgTopView.at<Vec3b>(i, nCenterPointX - (int)fLaneIntervalPixel / 2) = Vec3b(0, 255, 255);
			imgTopView.at<Vec3b>(i, nCenterPointX + (int)fLaneIntervalPixel / 2) = Vec3b(0, 255, 255);
		}
	}

}
