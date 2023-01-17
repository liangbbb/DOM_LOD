#pragma once
#ifndef PAIR_H
#define PAIR_H

#include "Point.h"
#include "PointSet.h"
#include <osg/Vec3f>
#include <osg/Geometry>

using namespace std;
struct Pair
{
	int v1;//���ʶ
	int v2;//���ʶ
	int strat_Point;//���
	int end_Point;//ָ���
	int Edge_type;//�ߵ����0�������۵���1���ڲ��ߣ�2����ӱ߽�ߣ�3���غϱ߽�ߣ�

	SPoint bestPoint;//����۵���
	double delCost;//�ߵ��۵�����
	bool changeable;//���Ƿ���۵�

	Pair();
	Pair(int _v1, int _v2);
	void sort();
	bool JudgeBoundary(PointSet* set);//�ж��Ƿ�Ϊ�߽���
	bool JudgeUVBoundary(PointSet* set, int &V1, int &V2);//�ж��Ƿ���۵��������
	int JudgeTex(PointSet* set, int &V1, int &V2);//�жϱߵ����ͣ�1��p1,p2����һ�����ǵ�����㣻2��p1,p2��Ϊ������㣻3��������
	int JudgeStrategy(PointSet* set);//�жϱߵ����ͣ�1���ڲ��ߣ�2���߽�ߣ���ӣ���3���߽�ߣ��ص�����4��������
	int JudgeStrategy_MSADO(PointSet* set);//�жϱߵ����ͣ�1����ƽ���㣻2��һ��ƽ���㣻3�����ۺ۵㣻4��������
	double calculate_TexComplexity(PointSet* set);
	void calculateBestPoint(PointSet* set, bool &success);//��������������������ŵ�
	void calculateBestPointUV(PointSet* set, bool &success);//���������������������ŵ�
	void calculateBestPointQEM(PointSet* Set, bool &success);//���������������������ŵ�
	void calculateBestPointQEM1(PointSet* Set, bool &success);//���������������������ŵ�
	void calculateBestPointD_QEM(PointSet* Set, bool &success, int edge_type);//�˼�ϸ�ڵļ򻯼������ŵ�
	void calculateBestPointMSADO(PointSet* Set, bool &success, int edge_type);//��������������ŵ�
	void calculateBest(PointSet* set, bool &success);//��������������������ŵ�

	double calculateMaxNormalDeviation(PointSet* set);//���������ƫת
	bool JudgeAbnormalCollpase(PointSet* set);//�ж��쳣�۵�
	bool ChangeTopology(PointSet* set);//�ж������Ƿ�ı�
	bool calculateVertexSharpness(PointSet* set, int v1, double &Sharpness);//�����ļ����
	bool CalculateVertexSharpness(PointSet* set, int v1, double &Sharpness);//�����ļ����
	double calculateTriangularArea(cv::Vec3d P1, cv::Vec3d P2, cv::Vec3d P3);//�������������

	bool calculateSeamError(PointSet* set, int v1, double &SeamError);//�ӷ�Ƕ����

	void calculateBestUV(PointSet* set);
	cv::Vec3d calculateProjectionPoint(cv::Vec3d plane[3], cv::Vec3d P);
	bool JudgeInclusion(cv::Vec3d plane[3], cv::Vec3d P);
	void calculateDelCost(PointSet* set);//������������������۵�����
	void calculateDelCostUV(PointSet* set);//�������������������۵�����
	void calculateDelCostDQEMUV(PointSet* set);//�������������������۵�����
	void calculateDelCostQEM(PointSet* set);//�������������������۵�����
	void calculateDelCostQEM1(PointSet* set);//�������������������۵�����
	void calculateDelCostD_QEM(PointSet* set, int type, double V_sharpness, double S_angleerror, double T_complexity);//�˼�ϸ�ڵļ򻯼����۵�����
	void calculateDelCostMSADO(PointSet* Set, int type, bool success, double dc, double de);//��������򻯼����۵�����

	//void calculateDelCostUV(PointSet* set);//�������������������۵�����
	void calculateDelCostUV_Boundary(PointSet* set, cv::Mat mat1, cv::Mat mat2);//�������������������۵�����(����߽��)
	double calculateDelCostUV_Boundary1(PointSet* set, int xyz_end, int uv_end, cv::Mat mat1, cv::Mat mat2);//�������������������۵����ۣ������յ�(����߽��)
	void calculateDelCostUV_twoTotwo(PointSet* set, int TexturePoiny_index, cv::Mat mat1, cv::Mat mat2);//���������ѷ촦�۵�����

	double calcuateAngleError(PointSet* set, int strat_Point, int end_Point);//����ӷ�������ڱ߽Ƕ����
	//bool isFeaturePair(PointSet* set);
	//Pair operator =(Pair& p);
	bool computeRotationMatrix(cv::Vec3d plane[3], osg::Matrix &R, osg::Matrix &R_i);//����ת�����������������,Ȼ������ת����
	osg::Vec3d getXYZ(cv::Vec3d P, osg::Matrix R);//��ת
	bool affineTransformMatrix(osg::Vec3d plane[3], cv::Point2d uv[3], osg::Vec3d P, cv::Point2d &UV);//�������任����,�����з���任
};

#endif
