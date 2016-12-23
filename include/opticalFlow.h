#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>

using namespace std;
using namespace cv;

/************************************************************************
@brief��ͨ��������Lucas - Kanade ������������ĳЩ�㼯�Ĺ�����ϡ�������
************************************************************************/
class OpticalFlow
{
public:
	OpticalFlow(int _max_corners, double _quality_level, double _min_dist);
	OpticalFlow();
	~OpticalFlow();
	void tracking(Mat &_frame);
	Mat output;
	vector<Point2f> points[2];	// point0Ϊ�������ԭ��λ�ã������ٵĵ㣩��point1Ϊ��ǰ֡�������㣬�ɹ������������
	vector<Point2f> initial;	// ���ٵ�ĳ�ʼλ��
	vector<Point2f> displacement;
	int getPointsNum(){ return pointsNum; };
	bool isDroneAppear();
	RotatedRect motionObject;
	vector<Point2f> movingPoints;
private:
	bool addNewPoints();
	bool acceptTrackedPoint(int _i);
	Mat gray;					// ��ǰ֡
	Mat gray_prev;				// ǰһ֡
	vector<Point2f> features;	// ����������
	int max_corners;			// �������������
	double quality_level;		// �������ĵȼ�
	double min_dist;			// ��������֮�����С����
	vector<uchar> status;		// ����������״̬��������������Ϊ1������Ϊ0
	vector<float> err;			// �¾�����������λ�õ����
	int pointsNum;				//��׷�������������
};


