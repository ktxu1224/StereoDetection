#ifndef _DEFSTRUCT_H
#define _DEFSTRUCT_H

#include <opencv2/opencv.hpp>
#include <iostream>

#define PI 3.1415926

#ifndef WIN32
#define sprintf_s snprintf
#endif

using namespace cv;

enum { MYDATA, KITTI };
//enum { Ped, Car, TrafficSign, TrafficLight, Else};
enum { TP, FN, FP, TP_S, FN_S };
enum { STEREO_BM = 0, STEREO_SGBM = 1 };
enum { GRAY, COLOR };

struct stixel_t
{
	int nGround;		// Ground bound point
	int nHeight;		// Upper bound point in image, unit: pixel
	uchar chDisparity;	// Disparity(normalized 255)
	int nCol;			// column
	double dZ;			// distance(meter)
	double dY_bottom;			// meter, Y coordinate of stixel bottom
	double dY_top;              // meter, Y coordinate of stixel bottom
	double dX;			// meter
	stixel_t() {
		nGround = -1;
		nHeight = -1;
		chDisparity = 0;
		dZ = 0.;
		dY_bottom = 0.;
		dY_top = 0;
		dX = 0.;
	}
};

struct Object_t
{
	Rect rectBB;
	double dZ; //min depth
	double leftDX, rightDX, topDY, bottomDY;  //coordinate in 3D world
	double maxDepthofStixels;
	double width, height; //depth width and height of one object
	std::vector<stixel_t> vecobjStixels;
	Object_t() {
		rectBB = Rect(0, 0, 0, 0);
		dZ = leftDX = rightDX = topDY = bottomDY = 0.;
		width = height = 0.;
		vecobjStixels.clear();
	}
	Object_t(Rect rect, double dz, double minDX = 0., double maxDX = 0., double minDY = 0., double maxDY = 0., double maxDepth = 0.) {
		rectBB = rect;
		dZ = dz;
		leftDX = minDX;
		rightDX = maxDX;
		topDY = minDY;
		bottomDY = maxDY;
		maxDepthofStixels = maxDepth;
		width = abs(leftDX - rightDX);
		height = abs(topDY - bottomDY);
		vecobjStixels.clear();
	}
	Object_t(Rect rect, stixel_t objStixel, double minDX = 0., double maxDX = 0., double minDY = 0., double maxDY = 0.) {
		rectBB = rect;
		dZ = maxDepthofStixels = objStixel.dZ;
		leftDX = minDX;
		rightDX = maxDX;
		topDY = minDY;
		bottomDY = maxDY;
		width = abs(leftDX - rightDX);
		height = abs(topDY - bottomDY);
		vecobjStixels.push_back(objStixel);
	}
};

struct CameraParam_t
{
	double m_dPitchDeg;  	// unit : degree
	double m_dYawDeg;     // unit : degree
	double m_dFocalLength_X;   // unit : pixels
	double m_dFocalLength_Y;
	double m_dOx;    //< unit : pixels
	double m_dOy;    // unit : pixels
	double m_dCameraHeight; // cam height from ground
	double m_dFOVvDeg; // vertical FOV
	double m_dFOVhDeg; // horizontal FOV
	int m_nVanishingY; // vanishing line location
	cv::Size m_sizeSrc;
	CameraParam_t() {
		m_dPitchDeg = 0.;
		m_dYawDeg = 0.;
		m_dFocalLength_X = 0.;
		m_dFocalLength_Y = 0;
		m_dOx = 0.;
		m_dOy = 0.;
		m_dCameraHeight = 0.;
		m_dFOVvDeg = 0.;
		m_dFOVhDeg = 0.;
		m_sizeSrc = cv::Size(0, 0);
	}
};

struct StereoCamParam_t
{
	int m_nNumberOfDisp; // number of disparity.
	int m_nWindowSize;   // window size. It must be odd number.
	double m_dBaseLine;  // baseline, unit : meters1
	double m_dMaxDist;   // Maximum distance value, unit : meters
	CameraParam_t objCamParam;
	StereoCamParam_t() {
		m_nNumberOfDisp = 80;
		m_nWindowSize = 9;
		m_dBaseLine = 0.;
		m_dMaxDist = 50.0;
	}
};

#endif 

