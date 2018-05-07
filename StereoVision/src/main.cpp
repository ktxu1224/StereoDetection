#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "../detection/StereoVisionForADAS.h"

using namespace std;
using namespace cv;

Mat imgLeft;

int main()
{
	StereoCamParam_t objParam = CStereoVisionForADAS::InitStereoParam(MYDATA);
	CStereoVisionForADAS objStereoVision(objParam);

	int cntFrame = 1068;
	char* chLeftImageName = new char[50];
	char* chRightImageName = new char[50];

	char* save_path = new char[30];
	Mat stitched_image(500 * 2, 1571, CV_8UC3);
	while (true) {
		sprintf_s(chLeftImageName, 50, "./data/my_data_left/%06d.bmp", cntFrame);
		sprintf_s(chRightImageName, 50, "./data/my_data_right/%06d.bmp", cntFrame++);

	    imgLeft = imread(chLeftImageName, 1);
		Mat imgRight = imread(chRightImageName, 1);
		GaussianBlur(imgLeft, imgLeft, Size(3, 3), 0, 0);
		GaussianBlur(imgRight, imgRight, Size(3, 3), 0, 0);

		objStereoVision.Objectness(imgLeft, imgRight);

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

		imgStixel(Rect(0, 399, imgStixel.cols, 500)).copyTo(stitched_image(Rect(0, 0, imgStixel.cols, 500)));
		imgResult(Rect(0, 399, imgStixel.cols, 500)).copyTo(stitched_image(Rect(0, 500, imgStixel.cols, 500)));

		//sprintf_s(save_path, 30, "%s%d%s", ".//result_stitch_", cntFrame, ".bmp");
		//imwrite(save_path, stitched_image);
		imshow("stitched image", stitched_image);
		waitKey(0);
	}

	delete[] chLeftImageName;
	delete[] chRightImageName;
	delete[] save_path;
	chLeftImageName = nullptr;
	chRightImageName = nullptr;
	save_path = nullptr;

	return 0;
}