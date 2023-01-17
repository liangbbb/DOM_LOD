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
	int v1;//点标识
	int v2;//点标识
	int strat_Point;//起点
	int end_Point;//指向点
	int Edge_type;//边的类别（0：不可折叠，1：内部边，2：相接边界边；3：重合边界边）

	SPoint bestPoint;//最佳折叠点
	double delCost;//边的折叠代价
	bool changeable;//边是否可折叠

	Pair();
	Pair(int _v1, int _v2);
	void sort();
	bool JudgeBoundary(PointSet* set);//判断是否为边界线
	bool JudgeUVBoundary(PointSet* set, int &V1, int &V2);//判断是否可折叠的纹理边
	int JudgeTex(PointSet* set, int &V1, int &V2);//判断边的类型（1、p1,p2中有一个点是单纹理点；2、p1,p2均为多纹理点；3、其他）
	int JudgeStrategy(PointSet* set);//判断边的类型（1、内部边；2、边界边（相接）；3、边界边（重叠）；4、其他）
	int JudgeStrategy_MSADO(PointSet* set);//判断边的类型（1、两平滑点；2、一个平滑点；3、两折痕点；4、其他）
	double calculate_TexComplexity(PointSet* set);
	void calculateBestPoint(PointSet* set, bool &success);//基本二次误差矩阵计算最优点
	void calculateBestPointUV(PointSet* set, bool &success);//带纹理二次误差矩阵计算最优点
	void calculateBestPointQEM(PointSet* Set, bool &success);//带纹理二次误差矩阵计算最优点
	void calculateBestPointQEM1(PointSet* Set, bool &success);//带纹理二次误差矩阵计算最优点
	void calculateBestPointD_QEM(PointSet* Set, bool &success, int edge_type);//顾及细节的简化计算最优点
	void calculateBestPointMSADO(PointSet* Set, bool &success, int edge_type);//外观驱动计算最优点
	void calculateBest(PointSet* set, bool &success);//基本二次误差矩阵计算最优点

	double calculateMaxNormalDeviation(PointSet* set);//计算最大法向偏转
	bool JudgeAbnormalCollpase(PointSet* set);//判断异常折叠
	bool ChangeTopology(PointSet* set);//判断拓扑是否改变
	bool calculateVertexSharpness(PointSet* set, int v1, double &Sharpness);//计算点的尖锐度
	bool CalculateVertexSharpness(PointSet* set, int v1, double &Sharpness);//计算点的尖锐度
	double calculateTriangularArea(cv::Vec3d P1, cv::Vec3d P2, cv::Vec3d P3);//计算三角形面积

	bool calculateSeamError(PointSet* set, int v1, double &SeamError);//接缝角度误差

	void calculateBestUV(PointSet* set);
	cv::Vec3d calculateProjectionPoint(cv::Vec3d plane[3], cv::Vec3d P);
	bool JudgeInclusion(cv::Vec3d plane[3], cv::Vec3d P);
	void calculateDelCost(PointSet* set);//基本二次误差矩阵计算折叠代价
	void calculateDelCostUV(PointSet* set);//带纹理二次误差矩阵计算折叠代价
	void calculateDelCostDQEMUV(PointSet* set);//带纹理二次误差矩阵计算折叠代价
	void calculateDelCostQEM(PointSet* set);//带纹理二次误差矩阵计算折叠代价
	void calculateDelCostQEM1(PointSet* set);//带纹理二次误差矩阵计算折叠代价
	void calculateDelCostD_QEM(PointSet* set, int type, double V_sharpness, double S_angleerror, double T_complexity);//顾及细节的简化计算折叠代价
	void calculateDelCostMSADO(PointSet* Set, int type, bool success, double dc, double de);//外观驱动简化计算折叠代价

	//void calculateDelCostUV(PointSet* set);//带纹理二次误差矩阵计算折叠代价
	void calculateDelCostUV_Boundary(PointSet* set, cv::Mat mat1, cv::Mat mat2);//带纹理二次误差矩阵计算折叠代价(计算边界边)
	double calculateDelCostUV_Boundary1(PointSet* set, int xyz_end, int uv_end, cv::Mat mat1, cv::Mat mat2);//带纹理二次误差矩阵计算折叠代价，给定终点(计算边界边)
	void calculateDelCostUV_twoTotwo(PointSet* set, int TexturePoiny_index, cv::Mat mat1, cv::Mat mat2);//计算纹理裂缝处折叠代价

	double calcuateAngleError(PointSet* set, int strat_Point, int end_Point);//计算接缝边与相邻边角度误差
	//bool isFeaturePair(PointSet* set);
	//Pair operator =(Pair& p);
	bool computeRotationMatrix(cv::Vec3d plane[3], osg::Matrix &R, osg::Matrix &R_i);//求旋转后坐标轴的三个标量,然后求旋转矩阵
	osg::Vec3d getXYZ(cv::Vec3d P, osg::Matrix R);//旋转
	bool affineTransformMatrix(osg::Vec3d plane[3], cv::Point2d uv[3], osg::Vec3d P, cv::Point2d &UV);//计算仿射变换矩阵,并进行仿射变换
};

#endif
