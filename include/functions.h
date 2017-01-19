#pragma once
#ifndef MYHEADFILE_H
#define MYHEADFILE_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "getConfig.h"

#endif // !MYHEADFILE_H

using namespace std;
using namespace cv;

//@brief����ʾ��Ϣ
void showText();

//@brief������ֱ��ͼ�ж��������Ҫ��ɫ
int JudgeColor(MatND hist);

//@brief������ѡ�����򷨣���a����Ӵ�С����
//@param��a����Ҫ��������飬����ִ������մӴ�С��˳������
//@param��b��������ĳ�����aһ�����������0��n-1������ִ������ʾa����Ӵ�С��ֵ��ԭa�����еĵ�λ������
//@param��n�����鳤��
void choise(int *a, int *b, int n);

// ���þ�����Ϣ�ж���������ϵ�еĵ㣨��װ�׵������Ƿ���һ�ԣ����Ƿ���װ�ף�
bool isPair(Vec3f v1, Vec3f v2);

// ������֮������ĵ�
Point centerOf2Points(Point p1, Point p2);