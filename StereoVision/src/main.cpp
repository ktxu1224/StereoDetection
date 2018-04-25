#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "../include/StereoVisionForADAS.h"

using namespace std;
using namespace cv;

int main()
{
	StereoCamParam_t objParam = CStereoVisionForADAS::InitStereoParam(MYDATA);
	CStereoVisionForADAS objStereoVision(objParam);

	int cntFrame = 1036;
	char* chLeftImageName = new char[50];
	char* chRightImageName = new char[50];
	int nWaitTime = 0;

	while (true) {
		sprintf_s(chLeftImageName, 50, "./data/my_data_left/%06d.bmp", cntFrame);
		sprintf_s(chRightImageName, 50, "./data/my_data_right/%06d.bmp", cntFrame++);

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

		/*imwrite(".\\result.bmp", imgResult);
		imwrite(".\\stixel.bmp", imgStixel);*/

		imshow ("result", imgResult);
		imshow("stixel", imgStixel);
		waitKey(0);
	}

	delete[] chLeftImageName;
	delete[] chRightImageName;
	chLeftImageName = nullptr;
	chRightImageName = nullptr;

	return 0;
}