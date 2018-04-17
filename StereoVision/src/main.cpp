#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "../include/StereoVisionForADAS.h"

using namespace std;
using namespace cv;

int main()
{
	StereoCamParam_t objParam = CStereoVisionForADAS::InitStereoParam(KITTI);
	CStereoVisionForADAS objStereoVision(objParam); //stixel constructor

	int cntFrame = 0;
	char* chLeftImageName = new char[50];
	char* chRightImageName = new char[50];
	int nWaitTime = 0;

	while (true) {
		sprintf_s(chLeftImageName, 50, "./data/left/%010d.png", cntFrame);
		sprintf_s(chRightImageName, 50, "./data/right/%010d.png", cntFrame++);

		Mat imgLeft = imread(chLeftImageName, 1);
		Mat imgRight = imread(chRightImageName, 1);
		if (imgLeft.empty()) {
			cout << "read image fail" << endl;
			break;
		}

		int64 t = getTickCount();
		// procesing
		objStereoVision.Objectness(imgLeft, imgRight);
		printf("Time elapsed: %.3fms\n", (getTickCount() - t) * 1000 / getTickFrequency());

		//display
		Mat imgDisp8;
		objStereoVision.m_matDisp16.convertTo(imgDisp8, CV_8U, 255 / (objParam.m_nNumberOfDisp*16.));
		Mat imgG;
		bitwise_and(objStereoVision.m_imgGround, imgDisp8, imgG);
		Mat imgResult = imgLeft.clone();
		Mat imgStixel = imgLeft.clone();
		Mat imgDispColor;
		applyColorMap(imgDisp8, imgDispColor, COLORMAP_OCEAN);
		objStereoVision.Display(imgResult, imgStixel);

		resize(imgLeft, imgLeft, Size(imgLeft.cols / 2, imgLeft.rows / 2));
		resize(imgDisp8, imgDisp8, Size(imgDisp8.cols / 2, imgDisp8.rows / 2));
		resize(imgDispColor, imgDispColor, Size(imgDispColor.cols / 2, imgDispColor.rows / 2));
		resize(imgResult, imgResult, Size(imgLeft.cols, imgLeft.rows));
		resize(imgStixel, imgStixel, Size(imgLeft.cols, imgLeft.rows));

		imshow("result", imgResult);
		imshow("stixel", imgStixel);
		waitKey(0);
	}

	delete[] chLeftImageName;
	delete[] chRightImageName;
	chLeftImageName = nullptr;
	chRightImageName = nullptr;

	return 0;
}