#ifndef _STIXELSEGMENTATION_H
#define _STIXELSEGMENTATION_H

#include <iostream>
#include <map> 
#include "DefStruct.h"
#include "StixelEstimation.h"

using namespace std;
using namespace cv;

typedef enum Seg_error { GOOD, SIZ_ERR } SEG_ERROR;

/**
	@class CStixelSegmentation
	@brief stixel Segmentation
*/
class CStixelSegmentation {
private:
	vector<stixel_t> m_vecobjStixels;
	StereoCamParam_t m_objStereoParam;

	SEG_ERROR StixelZClustering(vector<stixel_t>& objStixels, vector<Object_t>& objBBcandidate);
	SEG_ERROR StixelXClustering(vector<Object_t>& objBBinput, vector<Object_t>& objBBOutput);
	SEG_ERROR StixelBBboxOptimization(vector<Object_t>& objBBinput, vector<Object_t>& objBBOutput);

public:
	vector<Object_t> m_vecobjBB;

	CStixelSegmentation(StereoCamParam_t objStereoParam);
	SEG_ERROR SegmentStixel(vector<stixel_t>& objStixels);
};

#endif 