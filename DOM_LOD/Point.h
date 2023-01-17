#pragma once
#ifndef POINT_H
#define POINT_H

#include <vector>
#include "PointSet.h"
#include <opencv2/opencv.hpp>

using namespace std;

struct PointSet;
struct SPoint
{
	cv::Vec3d Point_Vector;//���������(xyz)��������             *
	cv::Vec3d Point_rgb;//���������(rgb)��������
	cv::Point2d PointUV;//��������������
	cv::Vec2d PointUV_Vector;//����������������������          *
	cv::Vec2d PointUV_Vector2;//����������������������
	set<int> SPointUV_index;//����������ʶ
	//Vector3 cdt;//�������������
	vector<int> neighbor;//һ��������ʶ
	//Mat4 error;//�����������ʱ���£�
	cv::Mat error;//�����������ʱ���£�
	cv::Mat Texture_error;//����������������ʱ���£�
	vector<cv::Mat> Texture_errors;//���ж�������������󼯺ϣ���ʱ���£�

	bool point_changeable;//���Ƿ���۵�(�߽�㲻���۵�)
	//bool pointPair_changeable;//�������߷�ת�ıߵĶ��㲻���۵�

	SPoint();
	//Point(Vector3 _cdt);
	//Point operator= (Point& _p);
	bool hasNeighbor(int neiId);
	void addNeighbor(int neiId);
	void removeNeighbor(int neiId);

	void calculateMat(PointSet* set);//�����������������
	void calculateUVMat(PointSet* set);//������������������
	void calculateUVMat1(PointSet* set);//�����������������
	void calculateQEMMat(PointSet* set);//�����������չ����������
	void calculateD_QEMMat(PointSet* set);//�������ϸ�ڲ����Ĵ��������������

	void error_reset();//��ʼ����������������
	void Texture_error_reset();//��ʼ�����������������
	void Texture_errors_reset();//��ʼ����������������󼯺�
	//void accelerateFinding(PointSet* set);

	double SeamAngleError(PointSet* set, int num_SPoint_UV);//����ӷ�Ƕ����

	bool operator == (const SPoint & obj) const //���� ��==�� ���������������� const �����ˣ�����ᱨ��
	{
		return Point_Vector == obj.Point_Vector;//����ƥ�������������Լ��趨
	}
};


struct SPoint_UV
{
	cv::Vec2d PointUV_Vector;//�������������(uv)��������
	vector<int> neighbor;//һ������������ʶ

	SPoint_UV();
	bool hasNeighbor(int neiId);
	void addNeighbor(int neiId);

	bool operator == (const SPoint_UV & obj) const //���� ��==�� ���������������� const �����ˣ�����ᱨ��
	{
		return PointUV_Vector == obj.PointUV_Vector;//����ƥ�������������Լ��趨
	}
};
#endif

