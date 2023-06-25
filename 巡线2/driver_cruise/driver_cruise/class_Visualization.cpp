#include "class_Visualization.h"


cls_VISUAL::cls_VISUAL()
{
	

}
cls_VISUAL::~cls_VISUAL()
{


}
void  cls_VISUAL::Fig1Y(int ID, float f1Ymin, float f1Ymax, int nStep, const char* str1YName1, float f1YNum1, const char* str1YName2, float f1YNum2, const char* str1YName3, float f1YNum3)
{
	int nDataNum;
	if (str1YName2 == NULL) nDataNum = 1;
	else if (str1YName3 == NULL) nDataNum = 2;
	else nDataNum = 3;

	cv::Mat im3Src = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
	cv::line(im3Src, cv::Point(19, 19), cv::Point(19, 460), cv::Scalar(255, 255, 255), 1);	// |
	cv::line(im3Src, cv::Point(19, 19), cv::Point(15, 25), cv::Scalar(255, 255, 255), 1);	// /
	cv::line(im3Src, cv::Point(19, 19), cv::Point(23, 25), cv::Scalar(255, 255, 255), 1);	// \
	cv::line(im3Src, cv::Point(16,19), cv::Point(18,19), cv::Scalar(255,255,255), 1);	// -

	cv::line(im3Src, cv::Point(16, 460), cv::Point(624, 460), cv::Scalar(255, 255, 255), 1);	// __
	cv::line(im3Src, cv::Point(620, 460), cv::Point(614, 456), cv::Scalar(255, 255, 255), 1); // \
	cv::line(im3Src, cv::Point(620,460), cv::Point(614,464), cv::Scalar(255,255,255), 1); // /

	char cName[512];
	sprintf_s(cName, "%.1f", f1Ymin);
	cv::putText(im3Src, cName, cv::Point(20, 460), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
	sprintf_s(cName, "%.1f", f1Ymax);
	cv::putText(im3Src, cName, cv::Point(20, 19), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
	for (int i = 1; i < nStep; i++)
	{
		float fHeight = 20.0 + 440.0 / (float)nStep * i;
		cv::line(im3Src, cv::Point(16, fHeight), cv::Point(18, fHeight), cv::Scalar(255, 255, 255));
		cv::LineIterator it(im3Src, cv::Point(20, fHeight), cv::Point(620, fHeight), 8);            // get a line iterator
		for (int i = 0; i < it.count; i++, it++)
			if (i % 6 == 0 || i % 6 == 1) { (*it)[0] = 100; (*it)[1] = 100; (*it)[2] = 100; }

		sprintf_s(cName, "%.1f", (f1Ymax - f1Ymin) / (float)nStep * (float)(nStep - i) + f1Ymin);
		cv::putText(im3Src, cName, cv::Point(20, fHeight), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(100, 100, 100));
	}


	mbr_vecData[ID][0].push_back(f1YNum1);
	if (nDataNum >= 2)
		mbr_vecData[ID][1].push_back(f1YNum2);
	if (nDataNum == 3)
		mbr_vecData[ID][2].push_back(f1YNum3);

	
	cv::Vec3b myColor[3] = { cv::Vec3b(255, 255, 0), cv::Vec3b(0, 255, 255) ,cv::Vec3b(0, 0, 255) };
	for (int nDataIndex = 0; nDataIndex < nDataNum; nDataIndex++)
	{
		cv::Point2f p2f_data0(20, 460);
		int i = mbr_vecData[ID][nDataIndex].size() > 550 ? mbr_vecData[ID][nDataIndex].size() - 550 : 0;

		for (i; i < mbr_vecData[ID][nDataIndex].size(); i++) {
			p2f_data0.x = p2f_data0.x + 1;
			p2f_data0.y = (f1Ymax - mbr_vecData[ID][nDataIndex][i]) / (f1Ymax - f1Ymin) * 440 + 20;
			if (p2f_data0.y < 480 && p2f_data0.y >= 0)
				im3Src.at<cv::Vec3b>(p2f_data0.y, p2f_data0.x) = myColor[nDataIndex];
		}
	}
	sprintf_s(cName, "%s: %.2f", str1YName1, f1YNum1);
	cv::putText(im3Src, cName, cv::Point(180, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
	if (nDataNum >= 2)
	{
	sprintf_s(cName, "%s: %.2f", str1YName2, f1YNum2);
	cv::putText(im3Src, cName, cv::Point(180, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
	}
	if (nDataNum == 3)
	{
		sprintf_s(cName, "%s: %.2f", str1YName3, f1YNum3);
		cv::putText(im3Src, cName, cv::Point(320, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Vec3b(0, 0, 255));
	}

	sprintf_s(cName, "Figure-%d", ID);
	cv::imshow(cName, im3Src);

}

void cls_VISUAL::Fig2Y(int ID, float f1Ymin, float f1Ymax, float f2Ymin, float f2Ymax, int nStep, const char* str1YName1, float f1YNum1, const char* str2YName1, float f2YNum1,const char* str1YName2, float f1YNum2)
{
	cv::Mat im3Src = cv::Mat::zeros(cv::Size(640,480),CV_8UC3);
	cv::line(im3Src, cv::Point(19,19), cv::Point(19,460), cv::Scalar(255,255,255), 1);	// |
	cv::line(im3Src, cv::Point(19,19), cv::Point(15,25), cv::Scalar(255,255,255), 1);	// /
	cv::line(im3Src, cv::Point(19,19), cv::Point(23,25), cv::Scalar(255,255,255), 1);	// \
	cv::line(im3Src, cv::Point(16,19), cv::Point(18,19), cv::Scalar(255,255,255), 1);	// -

	cv::line(im3Src, cv::Point(620,19), cv::Point(620,460), cv::Scalar(255,255,255), 1); // |
	cv::line(im3Src, cv::Point(620,19), cv::Point(616,25), cv::Scalar(255,255,255), 1);	// /
	cv::line(im3Src, cv::Point(620,19), cv::Point(624,25), cv::Scalar(255,255,255), 1);	// \
	cv::line(im3Src, cv::Point(621,19), cv::Point(623,19), cv::Scalar(255,255,255), 1);	// -

	cv::line(im3Src, cv::Point(16,460),cv::Point(624,460),cv::Scalar(255,255,255), 1);	// __
	cv::line(im3Src, cv::Point(620,460), cv::Point(614,456), cv::Scalar(255,255,255), 1); // \
	cv::line(im3Src, cv::Point(620,460), cv::Point(614,464), cv::Scalar(255,255,255), 1); // /

	char cName[512];
	sprintf_s(cName,"%.1f",f1Ymin);
	cv::putText(im3Src,cName,cv::Point(20,460),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255));
	sprintf_s(cName,"%.1f",f1Ymax);
	cv::putText(im3Src,cName,cv::Point(20,19),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255));
	for (int i=1; i<nStep; i++)
	{
		float fHeight = 20.0+440.0/(float)nStep*i;
		cv::line(im3Src,cv::Point(16,fHeight),cv::Point(18,fHeight),cv::Scalar(255,255,255));
		cv::LineIterator it(im3Src, cv::Point(20,fHeight), cv::Point(620,fHeight), 8);            // get a line iterator
		for(int i = 0; i < it.count; i++,it++)
			if ( i%6==0 || i%6==1) {(*it)[0] = 100;(*it)[1] = 100;(*it)[2] = 100;} 

			sprintf_s(cName,"%.1f",(f1Ymax-f1Ymin)/(float)nStep*(float)(nStep-i)+f1Ymin);
			cv::putText(im3Src,cName,cv::Point(20,fHeight),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(100,100,100));
	}

	//char c2Name[512];
	sprintf_s(cName,"%.1f",f2Ymin);
	cv::putText(im3Src,cName,cv::Point(560,460),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255));
	sprintf_s(cName,"%.1f",f2Ymax);
	cv::putText(im3Src,cName,cv::Point(560,19),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255));
	for (int i=1; i<nStep; i++)
	{
		float fHeight = 20.0+440.0/(float)nStep*i;
		cv::line(im3Src,cv::Point(621,fHeight),cv::Point(623,fHeight),cv::Scalar(255,255,255));

		sprintf_s(cName,"%.1f",(f2Ymax-f2Ymin)/(float)nStep*(float)(nStep-i)+f2Ymin);
		cv::putText(im3Src,cName,cv::Point(560,fHeight),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(100,100,100));
	}

	mbr_vecData[ID][0].push_back(f1YNum1);
	mbr_vecData[ID][1].push_back(f2YNum1);
	mbr_vecData[ID][2].push_back(f1YNum2);
	cv::Point2f p2f_data0(20, 460);
	cv::Point2f p2f_data1(20, 460);
	cv::Point2f p2f_data2(20, 460);

	int i = mbr_vecData[ID][0].size() > 550? mbr_vecData[ID][0].size()- 550 : 0;

	for (i; i<mbr_vecData[ID][0].size(); i++){
		p2f_data0.x = p2f_data0.x + 1;
		p2f_data0.y = (f1Ymax - mbr_vecData[ID][0][i])/(f1Ymax-f1Ymin) * 440 + 20;
		if (p2f_data0.y < 480 && p2f_data0.y >= 0)
			im3Src.at<cv::Vec3b>(p2f_data0.y,p2f_data0.x) = cv::Vec3b(255,255,0);

		p2f_data1.x = p2f_data1.x + 1;
		p2f_data1.y = (f2Ymax - mbr_vecData[ID][1][i])/(f2Ymax-f2Ymin) * 440 + 20;
		if (p2f_data1.y < 480 && p2f_data1.y >= 0)
			im3Src.at<cv::Vec3b>(p2f_data1.y,p2f_data1.x) = cv::Vec3b(0,255,255);
		if (str1YName2!=NULL)
		{
			p2f_data2.x = p2f_data2.x + 1;
			p2f_data2.y = (f1Ymax - mbr_vecData[ID][2][i]) / (f1Ymax - f1Ymin) * 440 + 20;
			if (p2f_data2.y < 480 && p2f_data2.y >= 0)
				im3Src.at<cv::Vec3b>(p2f_data2.y, p2f_data2.x) = cv::Vec3b(0, 0, 255);

		}

	}
	sprintf_s(cName, "%s: %.2f", str1YName1, f1YNum1);
	cv::putText(im3Src, cName, cv::Point(180, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
	sprintf_s(cName, "%s: %.2f", str2YName1, f2YNum1);
	cv::putText(im3Src, cName, cv::Point(180, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
	sprintf_s(cName, "%s: %.6f", str1YName2, f1YNum2);
	if (str1YName2 != NULL)
	{
		cv::putText(im3Src, cName, cv::Point(320, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255));
		sprintf_s(cName, "Figure-%d", ID);
	}
	cv::imshow(cName, im3Src);
	//cv::waitKey(1);
}
