#pragma once
#ifndef POINTSET_H
#define POINTSET_H

#include <deque>
#include <vector>
#include "Point.h"
#include <map>

using namespace std;
struct SPoint;
struct SPoint_UV;
struct PointSet//start from 1
{
	vector<SPoint>point;//��̬�洢���е�
	deque<bool>enabled;//��̬�洢���е����Ч��
	//Point* point;
	int count;//��Ч��ĸ�������ʼΪ0
	int maxpos;//���е�ı�ʶ����ʼΪ0
	 //bool* enabled;

	vector<SPoint_UV>point_uv;//��̬�洢���������
	deque<bool>enabled_uv;//��̬�洢������������Ч��
	int count_uv;//��Ч�����ĸ�������ʼΪ0
	int maxpos_uv;//���������ı�ʶ����ʼΪ0

	PointSet();//�洢���е�
	int addPoint(SPoint& _p);
	int addPoint(SPoint& _p, int index);

	int addPoint_uv(SPoint_UV& _p);
	int addPoint_uv(SPoint_UV& _p, int index);

	void calculateErrorMat();//����������������󣨲���������չ��
	void calculateUVErrorMat();//������������������
	void calculateUVErrorMat1();//������������������
	void calculateQEMUVMat();//�������������QSlim��������չ��
	void calculateD_QEMUVMat();//�����������������󣨹˼�ϸ�ڣ�
	void calculateMSADOMat();//����������������������

	void delPoint(int pos);
	void delPoint_uv(int pos);

	int Repeat(SPoint& _p);//�жϵ��Ƿ��ظ���������
	int Repeat_UV(SPoint_UV& _p);//�ж�UV���Ƿ��ظ���������
	map<int, int> Ptr_SPoint;//
	map<int, int> Ptr_SPoint_UV;//

	void updata();//���µ��������
};
#endif
