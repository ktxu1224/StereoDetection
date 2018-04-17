#include "../include/StixelSegmenation.h"

CStixelSegmentation::CStixelSegmentation(StereoCamParam_t objStereoParam)
{
	m_objStereoParam = objStereoParam;
}

SEG_ERROR CStixelSegmentation::SegmentStixel(vector<stixel_t>& objStixels)
{
	if (objStixels.size() < 1)
		return SIZ_ERR;
	m_vecobjBB.clear();

	vector<Object_t> objBBcandidateZ, objBBcandidateX;
	objBBcandidateZ.clear();
	objBBcandidateX.clear();

	StixelZClustering(objStixels, objBBcandidateZ);

	StixelXClustering(objBBcandidateZ, objBBcandidateX);

	StixelBBboxOptimization(objBBcandidateX, m_vecobjBB);

	return GOOD;
}

//cluster based on depth
SEG_ERROR CStixelSegmentation::StixelZClustering(vector<stixel_t>& objStixels, vector<Object_t>& objBBcandidate)
{
	multimap<double, stixel_t> mapSortedStixel;

	for (unsigned int i = 0; i < objStixels.size(); i++)
	{
		mapSortedStixel.insert(pair<double, stixel_t>(objStixels[i].dZ, objStixels[i]));
	}

	multimap<double, stixel_t>::iterator start;
	multimap<double, stixel_t>::iterator end;
	start = mapSortedStixel.begin();
	end = mapSortedStixel.end();

	objBBcandidate.clear();
	Rect rectTemp(start->second.nCol, start->second.nHeight, 0, abs(start->second.nHeight - start->second.nGround));
	Object_t bbTemp(rectTemp, start->second, start->second.dX, start->second.dX, start->second.dY_top, start->second.dY_bottom);
	objBBcandidate.push_back(bbTemp);
	*start++;
	const double dZThreshold = 0.8;//depth threshold, unit: meter
	while (start != end)
	{
		bool flgDetected = false;
		unsigned int i = 0;
		for (i = 0; i < objBBcandidate.size(); i++)
		{
			//每次都和该类中最远的depth比较
			if (abs(objBBcandidate[i].maxDepthofStixels - start->first) < dZThreshold)
			{
				objBBcandidate[i].vecobjStixels.push_back(start->second);
				objBBcandidate[i].maxDepthofStixels = start->second.dZ;
				//bounding box（unit:pixel）更新
				if (objBBcandidate[i].rectBB.width < start->second.nCol - objBBcandidate[i].rectBB.x)
				{
					objBBcandidate[i].rectBB.width = abs(objBBcandidate[i].rectBB.x - start->second.nCol);
				}
				else if (objBBcandidate[i].rectBB.width < objBBcandidate[i].rectBB.x + objBBcandidate[i].rectBB.width - start->second.nCol)
				{
					objBBcandidate[i].rectBB.width = objBBcandidate[i].rectBB.x + objBBcandidate[i].rectBB.width - start->second.nCol;
				}
				objBBcandidate[i].rectBB.x = objBBcandidate[i].rectBB.x < start->second.nCol ? objBBcandidate[i].rectBB.x : start->second.nCol;
				objBBcandidate[i].rectBB.y = objBBcandidate[i].rectBB.y < start->second.nHeight ? objBBcandidate[i].rectBB.y : start->second.nHeight;
				objBBcandidate[i].rectBB.height = objBBcandidate[i].rectBB.height > (start->second.nGround - start->second.nHeight) ? objBBcandidate[i].rectBB.height : (start->second.nGround - start->second.nHeight);
				//3D 坐标，x向右；y向下时
				objBBcandidate[i].leftDX = objBBcandidate[i].leftDX < start->second.dX ? objBBcandidate[i].leftDX : start->second.dX;
				objBBcandidate[i].rightDX = objBBcandidate[i].rightDX > start->second.dX ? objBBcandidate[i].rightDX : start->second.dX;
				objBBcandidate[i].topDY = objBBcandidate[i].topDY < start->second.dY_top ? objBBcandidate[i].topDY : start->second.dY_top;
				objBBcandidate[i].bottomDY = objBBcandidate[i].bottomDY > start->second.dY_bottom ? objBBcandidate[i].bottomDY : start->second.dY_bottom;

				flgDetected = true;
				break;
			}
		}   //<<end of for
		if (flgDetected == false)
		{
			Rect rectTemp(start->second.nCol, start->second.nHeight, 0, abs(start->second.nHeight - start->second.nGround));
			Object_t bbTemp(rectTemp, start->second, start->second.dX, start->second.dX, start->second.dY_top, start->second.dY_bottom);

			objBBcandidate.push_back(bbTemp);
		}

		*start++;

	} //<<end of while

	//calculate width and height of each object, filter based on width and height
	const double heightThre = 0.5; //unit：meter
	const double WidthThre = 0.15;  //unit：meter
	vector<Object_t>::iterator it;
	for (it = objBBcandidate.begin();it != objBBcandidate.end();)
	{
		it->width = abs(it->leftDX - it->rightDX);
		it->height = abs(it->bottomDY - it->topDY);
		if (it->height < heightThre || it->width < WidthThre)
			it = objBBcandidate.erase(it);
		else
			++it;
	}

	return GOOD;
}

//X coordinate in 3D world based clustering
SEG_ERROR CStixelSegmentation::StixelXClustering(vector<Object_t>& objBBinput, vector<Object_t>& objBBOutput)
{
	objBBOutput.clear();
	for (unsigned int i = 0; i < objBBinput.size(); i++)
	{
		multimap<int, stixel_t> mapSortedStixel;		// X-direction distance
		for (unsigned int j = 0; j < objBBinput[i].vecobjStixels.size(); j++)
		{
			mapSortedStixel.insert(pair<int, stixel_t>(objBBinput[i].vecobjStixels[j].nCol, objBBinput[i].vecobjStixels[j]));
		}

		multimap<int, stixel_t>::iterator start;
		multimap<int, stixel_t>::iterator end;
		multimap<int, stixel_t>::iterator iter;
		start = mapSortedStixel.begin();
		iter = mapSortedStixel.begin();
		end = mapSortedStixel.end();

		*start++;
		bool flgSeparator = false;
		const double dXThre = 0.3; //x coordinate threshold in 3D world
		multimap<int, stixel_t>::iterator iterLB = mapSortedStixel.begin();    // Left bound
		multimap<int, stixel_t>::iterator iterRB = mapSortedStixel.end();      // Right bound

		while (start != end)
		{
			if (start->second.dX - iter->second.dX > dXThre)
			{
				flgSeparator = true;

				iterRB = iter;

				stixel_t temp = iterLB->second;
				Rect rectTemp(temp.nCol, temp.nHeight, iterRB->second.nCol - iterLB->second.nCol, 0);
				Object_t bbTemp(rectTemp, temp);

				int nBBminRow = m_objStereoParam.objCamParam.m_sizeSrc.height;
				int nBBmaxRow = 0;
				double minX = temp.dX;
				double maxX = minX;
				double minY = temp.dY_top;
				double maxY = temp.dY_bottom;
				double minZ = temp.dZ;
				while (iterLB != iterRB)
				{
					stixel_t currentStixel = iterLB->second;
					bbTemp.vecobjStixels.push_back(currentStixel);
					//更新boundingbox的尺寸
					nBBmaxRow = currentStixel.nGround > nBBmaxRow ? currentStixel.nGround : nBBmaxRow;
					nBBminRow = currentStixel.nHeight < nBBminRow ? currentStixel.nHeight : nBBminRow;
					//更新世界坐标系下的x和y,用于更新width和height
					minX = currentStixel.dX < minX ? currentStixel.dX : minX;
					maxX = currentStixel.dX > maxX ? currentStixel.dX : maxX;
					minY = currentStixel.dY_top < minY ? currentStixel.dY_top : minY;
					maxY = currentStixel.dY_bottom > maxY ? currentStixel.dY_bottom : maxY;
					//求最小的距离
					minZ = currentStixel.dZ < minZ ? currentStixel.dZ : minZ;

					*iterLB++;
				}
				rectTemp.height = nBBmaxRow - nBBminRow;
				rectTemp.y = nBBminRow;
				bbTemp.rectBB = rectTemp;
				bbTemp.dZ = minZ;
				bbTemp.height = abs(maxY - minY);
				bbTemp.width = abs(maxX - minX);

				objBBOutput.push_back(bbTemp);
				iterLB = start;

			}
			*start++;
			*iter++;
		}

		if (flgSeparator == false)
			objBBOutput.push_back(objBBinput[i]);

		//更新同一个类中待分离的最后一部分的参数
		if (flgSeparator == true)
		{
			Rect rectTemp(iterLB->second.nCol, iterLB->second.nHeight, objBBinput[i].rectBB.x + objBBinput[i].rectBB.width - iterLB->second.nCol, 0);
			stixel_t temp = iterLB->second;
			Object_t bbTemp(rectTemp, temp);
			int nBBminRow = m_objStereoParam.objCamParam.m_sizeSrc.height;
			int nBBmaxRow = 0;
			double minX = temp.dX;
			double maxX = minX;
			double minY = temp.dY_top;
			double maxY = temp.dY_bottom;
			double minZ = temp.dZ;
			while (iterLB != end)
			{
				stixel_t currentStixel = iterLB->second;
				bbTemp.vecobjStixels.push_back(currentStixel);
				//更新boundingbox的尺寸
				nBBmaxRow = currentStixel.nGround > nBBmaxRow ? currentStixel.nGround : nBBmaxRow;
				nBBminRow = currentStixel.nHeight < nBBminRow ? currentStixel.nHeight : nBBminRow;
				//更新世界坐标系下的x和y,用于更新width和height
				minX = currentStixel.dX < minX ? currentStixel.dX : minX;
				maxX = currentStixel.dX > maxX ? currentStixel.dX : maxX;
				minY = currentStixel.dY_top < minY ? currentStixel.dY_top : minY;
				maxY = currentStixel.dY_bottom > maxY ? currentStixel.dY_bottom : maxY;
				//求最小的距离
				minZ = currentStixel.dZ < minZ ? currentStixel.dZ : minZ;

				*iterLB++;
			}
			rectTemp.height = nBBmaxRow - nBBminRow;
			rectTemp.y = nBBminRow;
			bbTemp.rectBB = rectTemp;
			bbTemp.dZ = minZ;
			bbTemp.height = abs(maxY - minY);
			bbTemp.width = abs(maxX - minX);

			objBBOutput.push_back(bbTemp);

		}
	}
	return GOOD;
}

SEG_ERROR CStixelSegmentation::StixelBBboxOptimization(vector<Object_t>& objBBinput, vector<Object_t>& objBBOutput)
{
#if _DEBUG
	int cntFrame = 10;
	char* chLeftImageName = new char[50];
	sprintf_s(chLeftImageName, 50, "./data/left/%010d.png", cntFrame);
	Mat imgLeft = imread(chLeftImageName, 1);
	for (int i = 0; i < objBBinput.size(); i++)
	{
		rectangle(imgLeft, objBBinput[i].rectBB, Scalar(255, 255, 255), 1);
		imshow("debug", imgLeft);
		//waitKey();
	}
#endif

	objBBOutput.clear();

	for (unsigned int i = 0; i < objBBinput.size(); i++)
	{
		if (objBBinput[i].width > 0.2 && objBBinput[i].height > 0.3 && objBBinput[i].dZ < m_objStereoParam.m_dMaxDist)
		{
			objBBOutput.push_back(objBBinput[i]);
		}
	}
	
#if _DEBUG
	for (int i = 0; i < objBBOutput.size(); i++)
	{
		rectangle(imgLeft, objBBOutput[i].rectBB, Scalar(255, 0, 0), 1);
		imshow("debug", imgLeft);
		waitKey();
	}
#endif

	for (unsigned int i = 0; i < objBBOutput.size(); i++)
	{
		int nMaxRow = 0;
		double dMinDist = m_objStereoParam.m_dMaxDist;
		for (unsigned int j = 0; j < objBBOutput[i].vecobjStixels.size(); j++)
		{
			nMaxRow = (nMaxRow < objBBOutput[i].vecobjStixels[j].nGround) ? objBBOutput[i].vecobjStixels[j].nGround : nMaxRow;
			dMinDist = dMinDist < objBBOutput[i].vecobjStixels[j].dZ ? dMinDist : objBBOutput[i].vecobjStixels[j].dZ;
		}
		objBBOutput[i].rectBB.height = nMaxRow - objBBOutput[i].rectBB.y;
		objBBOutput[i].dZ = dMinDist;
	}

	return GOOD;
}