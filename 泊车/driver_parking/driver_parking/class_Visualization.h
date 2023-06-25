#pragma once

#include "Common_OpenCV.h"
//#include <omp.h>
 


class cls_VISUAL
{
private:
	std::vector<float> mbr_vecData[8][3];

public:
	cls_VISUAL();
	~cls_VISUAL();
	void Fig1Y(int ID, float f1Ymin, float f1Ymax, int nStep, const char* str1YName1, float f1YNum1, const char* str1YName2 = NULL, float f1YNum2 = -1, const char* str1YName3 = NULL, float f1YNum3 = -1);
	void Fig2Y(int ID, float f1Ymin, float f1Ymax, float f2Ymin, float f2Ymax, int nStep, const char* str1YName1, float f1YNum1, const char* str2YName1, float f2YNum1,const char* str1YName2=NULL, float f1YNum2=-1);


};