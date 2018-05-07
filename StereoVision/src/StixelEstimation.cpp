#include "../detection/StixelEstimation.h"

CStixelEstimation::CStixelEstimation(StereoCamParam_t& objStereroParam)
{
	m_objStereoParam = objStereroParam;
	m_nStixelWidth = 1;
	m_dGroundVdispSlope = 0;
	m_dGroundVdispOrig = 0;
	m_nVdispGroundThreshold = 50;
	m_nStixelGroundMargin = 15;
	m_imgGround = Mat(m_objStereoParam.objCamParam.m_sizeSrc, CV_8U);
	m_imgGround = Scalar(0);
	m_road_estimation_format = ROAD_ESTIMATION_CAMERAMODEL;;
}

STIXEL_ERROR CStixelEstimation::SetDispImage(Mat& matDisp16)
{
	m_matDisp16 = matDisp16;
	matDisp16.convertTo(m_imgDisp8, CV_8U, 255 / (m_objStereoParam.m_nNumberOfDisp*16.));
	if (m_imgDisp8.size() != m_objStereoParam.objCamParam.m_sizeSrc)
		return SIZE_ERR;
	return OK;
}

STIXEL_ERROR CStixelEstimation::SetDispImage(Mat& matDisp16, Mat& imgDisp8)
{
	m_matDisp16 = matDisp16;
	m_imgDisp8 = imgDisp8;
	if (m_imgDisp8.size() != m_objStereoParam.objCamParam.m_sizeSrc) return SIZE_ERR;
	return OK;
}

STIXEL_ERROR CStixelEstimation::SetDisp8Image(Mat& imgDisp8)
{
	m_imgDisp8 = imgDisp8;
	if (m_imgDisp8.size() != m_objStereoParam.objCamParam.m_sizeSrc)
		return SIZE_ERR;
	return OK;
}

STIXEL_ERROR CStixelEstimation::EstimateStixels(Mat& matDisp16)
{
	Size size(5, 5);
	Mat matKernel = getStructuringElement(MORPH_RECT, size);

	SetDispImage(matDisp16);
	GroundEstimation(m_imgDisp8);
	RmSky(m_imgDisp8);
	morphologyEx(m_imgDisp8, m_imgDisp8, MORPH_OPEN, matKernel, Point(-1, -1), 1);
	StixelDistanceEstimation(m_imgDisp8, m_vecobjStixels);
	StixelROIConstraint_Lane(m_vecobjStixels, m_vecobjStixelInROI, 3., m_imgDisp8.cols / 2);

	return OK;
}

STIXEL_ERROR CStixelEstimation::EstimateStixels(Mat& matDisp16, Mat& imgDisp8, bool flgUseMultiLayer)
{
	Size size(5, 5);
	Mat matKernel = getStructuringElement(MORPH_RECT, size);
	SetDispImage(matDisp16, imgDisp8);
	GroundEstimation(m_imgDisp8);
	RmSky(m_imgDisp8);

	morphologyEx(m_imgDisp8, m_imgDisp8, MORPH_OPEN, matKernel, Point(-1, -1), 1);


	StixelDistanceEstimation(m_imgDisp8, m_vecobjStixels, flgUseMultiLayer);

	return OK;
}

STIXEL_ERROR CStixelEstimation::EstimateStixels_only8bitDisp(Mat& imgDisp8, bool flgUseMultiLayer)
{
	Size size(5, 5);
	Mat matKernel = getStructuringElement(MORPH_RECT, size);

	SetDisp8Image(imgDisp8);

	GroundEstimation(m_imgDisp8);
	RmSky(m_imgDisp8);
	morphologyEx(m_imgDisp8, m_imgDisp8, MORPH_OPEN, matKernel, Point(-1, -1), 1);

	StixelDistanceEstimation(m_imgDisp8, m_vecobjStixels, flgUseMultiLayer);

	StixelROIConstraint_Lane(m_vecobjStixels, m_vecobjStixelInROI, 3., m_imgDisp8.cols / 2);

	return OK;
}

STIXEL_ERROR CStixelEstimation::GroundEstimation(Mat& imgDisp8)
{
	ComputeVDisparity(imgDisp8);

	RmVDisparityNoise(m_imgVdisp);

	if (m_road_estimation_format == ROAD_ESTIMATION_AUTO)
	{
		ExtractGroundPoint(m_imgVdisp, m_vecLinePoint);
		FitLineRansac(m_vecLinePoint, m_vec2fLine);
	}
	else
	{
		RoadEstimationCameraModel(m_vec2fLine);
	}
	RmGround(m_vec2fLine, imgDisp8);

	if (m_dGroundVdispOrig <= 0 || m_dGroundVdispSlope > 2 || m_dGroundVdispSlope < 0.5) {
		printf("RANSAC : cannot find ground line. out of ranges\n");
		return GND_ERR;
	}
	return OK;
}

STIXEL_ERROR CStixelEstimation::ComputeVDisparity(Mat& imgDisp8)
{
	int maxDisp = 255;
	m_imgVdisp = Mat(imgDisp8.rows, 255, CV_8U, Scalar(0));
	for (int u = 0; u < imgDisp8.rows; u++) {
		if (u < m_objStereoParam.objCamParam.m_nVanishingY)
			continue;        // we are finding ground. therefore we check pixels below vanishing point 
		for (int v = 0; v < imgDisp8.cols; v++) {
			int disp = (imgDisp8.at<uchar>(u, v));
			//if(disp>0 && disp < maxDisp){
			if (disp > 6 && disp < maxDisp - 2) {
				m_imgVdisp.at<uchar>(u, disp) += 1;
			}
		}
	}
	return OK;
}

STIXEL_ERROR CStixelEstimation::RmVDisparityNoise(Mat& imgVdisp)
{
	int nThresh = m_nVdispGroundThreshold;
	threshold(imgVdisp, imgVdisp, 10, 255, 3);
	return OK;
}

STIXEL_ERROR CStixelEstimation::ExtractGroundPoint(Mat& imgVdisp, vector<Point2f>& vecLinePoint)
{
	vecLinePoint.clear();
	for (int u = m_objStereoParam.objCamParam.m_nVanishingY; u < imgVdisp.rows; u++) {
		for (int v = 0; v < imgVdisp.cols; v++) {
			int value = imgVdisp.at<uchar>(u, v);
			if (value > 0) {
				vecLinePoint.push_back(Point2f((float)u, (float)v));
			}
		}
	}
	return OK;
}

STIXEL_ERROR CStixelEstimation::FitLineRansac(vector<Point2f>& vecLinePoint, Vec2f& vec2fLine)
{
	Vec4f vec4fLine;
	int iterations = 100;
	double sigma = 1.;
	double a_max = 7.;

	int n = vecLinePoint.size();
	if (n < 2)
	{
		printf("Points must be more than 2 EA\n");
		return GNDRANSAC_ERR;
	}

	RNG rng;
	double bestScore = -1.;
	for (int k = 0; k < iterations; k++)
	{
		int i1 = 0, i2 = 0;
		double dx = 0;
		while (i1 == i2)
		{
			i1 = rng(n);
			i2 = rng(n);
		}
		Point2f p1 = vecLinePoint[i1];
		Point2f p2 = vecLinePoint[i2];

		Point2f dp = p2 - p1;
		dp *= 1. / norm(dp);
		double score = 0;

		if (fabs(dp.x / 1.e-5f) && fabs(dp.y / dp.x) <= a_max)
		{
			for (int i = 0; i < n; i++)
			{
				Point2f v = vecLinePoint[i] - p1;
				double d = v.y*dp.x - v.x*dp.y;
				score += exp(-0.5*d*d / (sigma*sigma));
			}
		}
		if (score > bestScore)
		{
			vec4fLine = Vec4f(dp.x, dp.y, p1.x, p1.y);
			bestScore = score;
		}
	}

	vec2fLine[0] = vec4fLine[0] / vec4fLine[1];
	vec2fLine[1] = vec4fLine[2] - vec2fLine[0] * vec4fLine[3];

	return OK;
}

STIXEL_ERROR CStixelEstimation::RoadEstimationCameraModel(Vec2f& vec2fLine)
{
	const float sinTilt = sinf(m_objStereoParam.objCamParam.m_dPitchDeg);
	const float cosTilt = cosf(m_objStereoParam.objCamParam.m_dPitchDeg);
	const float a = (m_objStereoParam.m_dBaseLine / m_objStereoParam.objCamParam.m_dCameraHeight) * cosTilt;
	const float b = (m_objStereoParam.m_dBaseLine / m_objStereoParam.objCamParam.m_dCameraHeight) * (m_objStereoParam.objCamParam.m_dFocalLength_X * sinTilt - m_objStereoParam.objCamParam.m_dOy * cosTilt);

	vec2fLine[0] = m_objStereoParam.m_nNumberOfDisp / (a * 255.0);
	vec2fLine[1] = -b / a;

	return OK;
}

STIXEL_ERROR CStixelEstimation::RmGround(Vec2f vec2fLine, Mat& imgDisp8)
{
	m_dGroundVdispSlope = vec2fLine[0];
	m_dGroundVdispOrig = vec2fLine[1];

	const double deltaHeight = 0.2;  //unit: meter
	double cosPitch = cos(m_objStereoParam.objCamParam.m_dPitchDeg);
	double middleValue = (double)m_objStereoParam.m_nNumberOfDisp / (m_objStereoParam.m_dBaseLine*255.);

	for (int u = (int)m_dGroundVdispOrig; u < imgDisp8.rows; u++)
		for (int v = 0; v < imgDisp8.cols; v++)
		{
			int disparityValue = imgDisp8.at<uchar>(u, v);
			double test = m_dGroundVdispOrig + m_dGroundVdispSlope * disparityValue - u;
			//remove the points below a certain height
			//double deltaV = deltaHeight * disparityValue *(double)m_objStereoParam.m_nNumberOfDisp / (255 * m_objStereoParam.m_dBaseLine);
			double deltaV = disparityValue * middleValue *deltaHeight / cosPitch;
			if (test > deltaV)
			{
				imgDisp8.at<uchar>(u, v) = disparityValue;
			}
			else
			{
				imgDisp8.at<uchar>(u, v) = 0;
				m_imgGround.at<uchar>(u, v) = disparityValue;
			}
		}

	return OK;
}

//remove points above specific height, Yw向下，因而V越大对应的三维空间中的Y越大，因而天空的Y最小，地面最大
STIXEL_ERROR CStixelEstimation::RmSky(Mat& imgDisp8)
{
	double orig = m_dGroundVdispOrig - 3;
	double slope = -0.5;  //slope一定小于0，且越小，去掉的高度越小，如-1.9时，10米以上； -2.5时，20m以上

	for (int u = 0; u < imgDisp8.rows; u++) {
		for (int v = 0; v < imgDisp8.cols; v++) {
			int value = imgDisp8.at<uchar>(u, v);
			if (u < (orig + slope * value)) {
				imgDisp8.at<uchar>(u, v) = 0;
			}

		}
	}
	return OK;
}

STIXEL_ERROR CStixelEstimation::StixelDistanceEstimation(Mat& imgDisp8, vector<stixel_t>& vecStixels, bool flgUseMultiLayer)
{
#ifdef _DEBUG
	int cntFrame = 1036;
	char* chLeftImageName = new char[50];
	sprintf_s(chLeftImageName, 50, "./data/my_data_left/%06d.bmp", cntFrame);
	Mat imgLeft = imread(chLeftImageName, 1);
#endif

	STIXEL_ERROR Err;
	vecStixels.clear();
	for (int u = 0; u < imgDisp8.cols; u++)
	{
		stixel_t objStixelTemp;
		if (u < 30) {
			objStixelTemp.chDisparity = 0;
			objStixelTemp.nGround = 0;
			objStixelTemp.nHeight = 0;
			objStixelTemp.nCol = u;
		}
		//原本flgUseMultiLayer位于EstimateStixels函数原型中，默认为true，被修改为false
		else if (flgUseMultiLayer == true) {
			Err = StixelDisparityEstimation_col_ML(imgDisp8, u, vecStixels);
		}
		else {
			Err = StixelDisparityEstimation_col_SL(imgDisp8, u, objStixelTemp);
			if (Err == OK && objStixelTemp.nHeight > 0 && objStixelTemp.nGround > 0) {
#ifdef _DEBUG
				/*rectangle(imgDisp8, Rect(u, objStixelTemp.nHeight, 1, objStixelTemp.nGround - objStixelTemp.nHeight), Scalar(255, 255, 255), 1);
				rectangle(imgLeft, Rect(u, objStixelTemp.nHeight, 1, objStixelTemp.nGround - objStixelTemp.nHeight), Scalar(255, 255, 255), 1);
				imshow("stixel-sourceImage", imgLeft);
				imshow("stixel-disparity", imgDisp8);
				waitKey();*/
#endif
				m_vecobjStixels.push_back(objStixelTemp);
			}
		}
	}

	StixelDisparityToDistance(vecStixels);

	return OK;
}

//has been modified
STIXEL_ERROR CStixelEstimation::StixelDisparityEstimation_col_SL(Mat& imgDisp8, int col, stixel_t& objStixel)
{
	int nIter = imgDisp8.rows / 2;
	uchar bottomDisp, topDisp;
	const double deltaHeight = 0.1; //physical height，unit: meter
	const double depthDiffThre = 1.5;    //try to find stixel whose depth of upbound and downbound within 1m, unit: meter
	double zTopBound = 0.;
	double zBottomBound = 0.;
	bool flag = true;
	bool breakFlag = false;
	double cosPitch = cos(m_objStereoParam.objCamParam.m_dPitchDeg);
	double sinPitch = sin(m_objStereoParam.objCamParam.m_dPitchDeg);
	double middleValue = m_objStereoParam.m_dBaseLine*255. / (double)m_objStereoParam.m_nNumberOfDisp;

	for (int v = 1; v < nIter && !breakFlag; v++) {
		topDisp = imgDisp8.at<uchar>(v, col);
		bottomDisp = imgDisp8.at<uchar>(imgDisp8.rows - v, col);
		//get the v coordinate of up bound
		if (topDisp > 0 && flag) {
			//zTopBound = m_objStereoParam.objCamParam.m_dFocalLength_X*m_objStereoParam.m_dBaseLine / ((double)topDisp*(double)m_objStereoParam.m_nNumberOfDisp / 255);
			zTopBound = middleValue * (m_objStereoParam.objCamParam.m_dFocalLength_X*cosPitch + (m_objStereoParam.objCamParam.m_dOy - v)*sinPitch) / static_cast<double>(topDisp);
			if (zBottomBound == 0 || abs(zTopBound - zBottomBound) < depthDiffThre)
			{
				flag = false;
				//nHeight为stixel上边界在图像中的行坐标，并非真实高度
				objStixel.nHeight = v;
				nIter = imgDisp8.rows - v;
			}
		}
		//get the v coordinate of down bound
		if (bottomDisp > 0 && objStixel.nGround == -1) {
			flag = true;
			//zBottomBound = m_objStereoParam.objCamParam.m_dFocalLength_X*m_objStereoParam.m_dBaseLine / ((double)bottomDisp*(double)m_objStereoParam.m_nNumberOfDisp / 255);
			zBottomBound = middleValue * (m_objStereoParam.objCamParam.m_dFocalLength_X*cosPitch + (m_objStereoParam.objCamParam.m_dOy - v)*sinPitch) / static_cast<double>(bottomDisp);
			//加上地面移除中的多去掉的高度（图像中的像素单位），使得stixel的下边界位于地面		
			double deltaV = bottomDisp * deltaHeight / (middleValue*cosPitch);
			objStixel.nGround = imgDisp8.rows - v + deltaV;  //nGround为stixel下边界在图像中的行坐标
			objStixel.nCol = col;   //图像中的列坐标
			nIter = imgDisp8.rows - v;
		}
		//break the loop when the up and down bound of stixel has been got and set the disparity of this stixel to the one in the middle
		if (objStixel.nHeight != -1 && objStixel.nGround != -1)
		{
			breakFlag = true;
			objStixel.chDisparity = imgDisp8.at<uchar>(static_cast<int>(0.5*(objStixel.nHeight + objStixel.nGround)), col);
		}
	}

	return OK;
}

//under development
STIXEL_ERROR CStixelEstimation::StixelDisparityEstimation_col_ML(Mat& imgDisp8, int col, vector<stixel_t>& vecStixels)
{
	int nSkybound = 0;
	uchar chDisp;
	stixel_t objStixelTemp;
	const double deltaHeight = 0.3;   //height above ground
	const double depthDiffThre = 1.0; //depth threshold for separating one stixel according depth difference 


	for (int v = 0; v < imgDisp8.rows / 2; v++) {
		if (imgDisp8.at<uchar>(v, col) > 1) {
			nSkybound = v;
			break;
		}
	}
	for (int v = imgDisp8.rows - 1; v >= nSkybound; v--) {
		chDisp = imgDisp8.at<uchar>(v, col);
		if (chDisp > 0 && objStixelTemp.nGround == -1 && v > m_objStereoParam.objCamParam.m_nVanishingY - 10) {
			objStixelTemp.nGround = v + 10;
			objStixelTemp.chDisparity = imgDisp8.at<uchar>(v, col);
			objStixelTemp.nCol = col;
		}
		else if (chDisp <= 1 && objStixelTemp.nGround != -1) {
			objStixelTemp.nHeight = v;
			vecStixels.push_back(objStixelTemp);
			objStixelTemp.chDisparity = 0; objStixelTemp.nGround = -1; objStixelTemp.nHeight = -1;
		}
		else if (v == nSkybound + 1 && objStixelTemp.nGround != -1)
		{
			objStixelTemp.nHeight = v;
			vecStixels.push_back(objStixelTemp);
			objStixelTemp.chDisparity = 0; objStixelTemp.nGround = -1; objStixelTemp.nHeight = -1;
		}
	}

	/*for (int v = 0; v < imgDisp8.rows / 2; v++) {
		if (imgDisp8.at<uchar>(v, col) > 1) {
			nSkybound = v;
			break;
		}
	}
	for (int v = imgDisp8.rows - 1; v >= nSkybound; v--) {
		chDisp = imgDisp8.at<uchar>(v, col);

		if (chDisp > 0 && objStixelTemp.nGround == -1 && v > m_objStereoParam.objCamParam.m_nVanishingY - 10) {
			objStixelTemp.nGround = v;
			objStixelTemp.chDisparity = chDisp;
			objStixelTemp.nCol = col;
		}
		else if (chDisp <= 1 && objStixelTemp.nGround != -1) {
			objStixelTemp.nHeight = v + 1;
			double zTopBound = m_objStereoParam.objCamParam.m_dFocalLength*m_objStereoParam.m_dBaseLine / ((double)imgDisp8.at<uchar>(objStixelTemp.nHeight, col)*(double)m_objStereoParam.m_nNumberOfDisp / 255);
			double zBottomBound = m_objStereoParam.objCamParam.m_dFocalLength*m_objStereoParam.m_dBaseLine / ((double)imgDisp8.at<uchar>(objStixelTemp.nGround, col)*(double)m_objStereoParam.m_nNumberOfDisp / 255);
			int i = 0;
			for (i = objStixelTemp.nHeight; i < objStixelTemp.nGround&&abs(zTopBound - zBottomBound) > depthDiffThre; i++)
			{
				zTopBound = m_objStereoParam.objCamParam.m_dFocalLength*m_objStereoParam.m_dBaseLine / ((double)imgDisp8.at<uchar>(i, col)*(double)m_objStereoParam.m_nNumberOfDisp / 255);
			}
			if (i < objStixelTemp.nGround)
			{
				objStixelTemp.nHeight = i;
				double deltaV = deltaHeight * imgDisp8.at<uchar>(objStixelTemp.nGround, col) *(double)m_objStereoParam.m_nNumberOfDisp / (255 * m_objStereoParam.m_dBaseLine);
				objStixelTemp.nGround = (objStixelTemp.nGround + deltaV) < (imgDisp8.rows - 1) ? (objStixelTemp.nGround + deltaV) : (imgDisp8.rows - 1);
				vecStixels.push_back(objStixelTemp);
				v = i;
			}

			objStixelTemp.chDisparity = 0;
			objStixelTemp.nGround = -1;
			objStixelTemp.nHeight = -1;
		}
		else if (v == nSkybound + 1 && objStixelTemp.nGround != -1)
		{
			objStixelTemp.nHeight = v + 1;
			double zTopBound = m_objStereoParam.objCamParam.m_dFocalLength*m_objStereoParam.m_dBaseLine / ((double)imgDisp8.at<uchar>(objStixelTemp.nHeight, col)*(double)m_objStereoParam.m_nNumberOfDisp / 255);
			double zBottomBound = m_objStereoParam.objCamParam.m_dFocalLength*m_objStereoParam.m_dBaseLine / ((double)imgDisp8.at<uchar>(objStixelTemp.nGround, col)*(double)m_objStereoParam.m_nNumberOfDisp / 255);
			int i = 0;
			for (i = objStixelTemp.nHeight; i < objStixelTemp.nGround&&abs(zTopBound - zBottomBound) > depthDiffThre; i++)
			{
				zTopBound = m_objStereoParam.objCamParam.m_dFocalLength*m_objStereoParam.m_dBaseLine / ((double)imgDisp8.at<uchar>(i, col)*(double)m_objStereoParam.m_nNumberOfDisp / 255);
			}
			if (i < objStixelTemp.nGround)
			{
				objStixelTemp.nHeight = i - 2;
				double deltaV = deltaHeight * imgDisp8.at<uchar>(objStixelTemp.nGround, col) *(double)m_objStereoParam.m_nNumberOfDisp / (255 * m_objStereoParam.m_dBaseLine);
				objStixelTemp.nGround = (objStixelTemp.nGround + deltaV) < (imgDisp8.rows - 1) ? (objStixelTemp.nGround + deltaV) : (imgDisp8.rows - 1);
				vecStixels.push_back(objStixelTemp);
				v = i;
			}

			objStixelTemp.chDisparity = 0;
			objStixelTemp.nGround = -1;
			objStixelTemp.nHeight = -1;
		}
	}*/

	/*uchar previousRawDisp = imgDisp8.at<uchar>(v + 1, col);
	double depthDiff = 100;
	if (chDisp > 0 && previousRawDisp > 0)
	{
		double zPreviousRaw = m_objStereoParam.objCamParam.m_dFocalLength*m_objStereoParam.m_dBaseLine / ((double)previousRawDisp*(double)m_objStereoParam.m_nNumberOfDisp / 255);
		double zCurrentRaw = m_objStereoParam.objCamParam.m_dFocalLength*m_objStereoParam.m_dBaseLine / ((double)chDisp*(double)m_objStereoParam.m_nNumberOfDisp / 255);
		depthDiff = abs(zPreviousRaw - zCurrentRaw);
	}
	const double depthDiffThre = 5.5;
	if (chDisp > 0 && depthDiff < depthDiffThre && objStixelTemp.nGround == -1 && v > m_objStereoParam.objCamParam.m_nVanishingY - 10)
	{
		//仅第一次加地面,需改
		objStixelTemp.nGround = v + 1;
		objStixelTemp.nCol = col;
	}
	else if (chDisp > 0 && depthDiff >= depthDiffThre && objStixelTemp.nGround != -1)
	{
		objStixelTemp.nHeight = v + 1;
		objStixelTemp.chDisparity = imgDisp8.at<uchar>(static_cast<int>(0.5*(objStixelTemp.nHeight + objStixelTemp.nGround)), col);
		vecStixels.push_back(objStixelTemp);
		objStixelTemp.chDisparity = 0;
		objStixelTemp.nGround = -1;
		objStixelTemp.nHeight = -1;
	}
	else if (chDisp <= 1 && objStixelTemp.nGround != -1) {
		objStixelTemp.nHeight = v + 1;
		objStixelTemp.chDisparity = imgDisp8.at<uchar>(objStixelTemp.nHeight, col);
		vecStixels.push_back(objStixelTemp);
		objStixelTemp.chDisparity = 0;
		objStixelTemp.nGround = -1;
		objStixelTemp.nHeight = -1;
	}
	else if (v == nSkybound + 1 && objStixelTemp.nGround != -1)
	{
		objStixelTemp.nHeight = v + 1;
		objStixelTemp.chDisparity = imgDisp8.at<uchar>(objStixelTemp.nHeight, col);
		vecStixels.push_back(objStixelTemp);
		objStixelTemp.chDisparity = 0;
		objStixelTemp.nGround = -1;
		objStixelTemp.nHeight = -1;
	}*/


	return OK;
}

//dont take installation into consideration
STIXEL_ERROR CStixelEstimation::StixelDisparityToDistance(vector<stixel_t>& vecStixels)
{
	double cosPitch = cos(m_objStereoParam.objCamParam.m_dPitchDeg);
	double sinPitch = sin(m_objStereoParam.objCamParam.m_dPitchDeg);
	double middleValue = m_objStereoParam.m_dBaseLine*255. / (double)m_objStereoParam.m_nNumberOfDisp;

	for (unsigned int u = 0; u < vecStixels.size(); u++) {
		if (vecStixels[u].chDisparity == 0)
			vecStixels[u].dZ = 0;
		vecStixels[u].dZ = middleValue * (m_objStereoParam.objCamParam.m_dFocalLength_X*cosPitch + (m_objStereoParam.objCamParam.m_dOy - 0.5*(vecStixels[u].nHeight + vecStixels[u].nGround))*sinPitch) / static_cast<double>(vecStixels[u].chDisparity);
		vecStixels[u].dX = -0.5*m_objStereoParam.m_dBaseLine + middleValue * (vecStixels[u].nCol - m_objStereoParam.objCamParam.m_dOx) / static_cast<double>(vecStixels[u].chDisparity);
		vecStixels[u].dY_bottom = ((vecStixels[u].nGround - m_objStereoParam.objCamParam.m_dOy)*cosPitch + m_objStereoParam.objCamParam.m_dFocalLength_Y*sinPitch)*middleValue / static_cast<double>(vecStixels[u].chDisparity);
		vecStixels[u].dY_top = ((vecStixels[u].nHeight - m_objStereoParam.objCamParam.m_dOy)*cosPitch + m_objStereoParam.objCamParam.m_dFocalLength_Y*sinPitch)*middleValue / static_cast<double>(vecStixels[u].chDisparity);

		//vecStixels[u].dZ = m_objStereoParam.objCamParam.m_dFocalLength_X*m_objStereoParam.m_dBaseLine / ((double)vecStixels[u].chDisparity*(double)m_objStereoParam.m_nNumberOfDisp / 255);
		//vecStixels[u].dX = vecStixels[u].dZ*(double)(vecStixels[u].nCol - m_objStereoParam.objCamParam.m_dOx) / m_objStereoParam.objCamParam.m_dFocalLength_X;
		//vecStixels[u].dY_bottom = vecStixels[u].dZ*(double)(vecStixels[u].nGround - m_objStereoParam.objCamParam.m_dOy) / m_objStereoParam.objCamParam.m_dFocalLength_Y;
		//vecStixels[u].dY_top = vecStixels[u].dZ*(double)(vecStixels[u].nHeight - m_objStereoParam.objCamParam.m_dOy) / m_objStereoParam.objCamParam.m_dFocalLength_Y;
	}
	return OK;
}

STIXEL_ERROR CStixelEstimation::StixelROIConstraint_Lane(vector<stixel_t>& vecStixelsInput,
	vector<stixel_t>& vecStixelsOutput, float fLaneInterval, int nCenterPointX)
{
	vecStixelsOutput.clear();

	for (unsigned int u = 0; u < vecStixelsInput.size(); u++) {
		if (vecStixelsInput[u].chDisparity == 0) continue;
		if (vecStixelsInput[u].dX < (double)fLaneInterval / 2 - vecStixelsInput[u].dZ / m_objStereoParam.objCamParam.m_dFocalLength_X*(double)(m_imgDisp8.cols / 2 - nCenterPointX))
		{
			if (vecStixelsInput[u].dX > -(double)fLaneInterval / 2 + vecStixelsInput[u].dZ / m_objStereoParam.objCamParam.m_dFocalLength_X*(double)(m_imgDisp8.cols / 2 - nCenterPointX))
			{

				vecStixelsOutput.push_back(vecStixelsInput[u]);

			}
		}
	}
	return OK;
}