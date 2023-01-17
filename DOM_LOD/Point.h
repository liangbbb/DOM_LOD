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
	cv::Vec3d Point_Vector;//代表点坐标(xyz)向量矩阵             *
	cv::Vec3d Point_rgb;//代表点坐标(rgb)向量矩阵
	cv::Point2d PointUV;//代表点纹理坐标点
	cv::Vec2d PointUV_Vector;//代表点纹理坐标点向量矩阵          *
	cv::Vec2d PointUV_Vector2;//代表点纹理坐标点向量矩阵
	set<int> SPointUV_index;//纹理坐标点标识
	//Vector3 cdt;//代表点坐标向量
	vector<int> neighbor;//一环邻域点标识
	//Mat4 error;//点的误差矩阵（随时更新）
	cv::Mat error;//点的误差矩阵（随时更新）
	cv::Mat Texture_error;//带纹理点的误差矩阵（随时更新）
	vector<cv::Mat> Texture_errors;//具有多个纹理点的误差矩阵集合（随时更新）

	bool point_changeable;//点是否可折叠(边界点不可折叠)
	//bool pointPair_changeable;//会引起法线翻转的边的顶点不可折叠

	SPoint();
	//Point(Vector3 _cdt);
	//Point operator= (Point& _p);
	bool hasNeighbor(int neiId);
	void addNeighbor(int neiId);
	void removeNeighbor(int neiId);

	void calculateMat(PointSet* set);//计算基本二次误差矩阵
	void calculateUVMat(PointSet* set);//计算带纹理二次误差矩阵
	void calculateUVMat1(PointSet* set);//计算基本二次误差矩阵
	void calculateQEMMat(PointSet* set);//计算带纹理扩展二次误差矩阵
	void calculateD_QEMMat(PointSet* set);//计算多种细节参数的带纹理二次误差矩阵

	void error_reset();//初始化基本二次误差矩阵
	void Texture_error_reset();//初始化带纹理二次误差矩阵
	void Texture_errors_reset();//初始化带纹理二次误差矩阵集合
	//void accelerateFinding(PointSet* set);

	double SeamAngleError(PointSet* set, int num_SPoint_UV);//计算接缝角度误差

	bool operator == (const SPoint & obj) const //重载 “==” 操作符，函数最后的 const 别忘了，否则会报错。
	{
		return Point_Vector == obj.Point_Vector;//具体匹配条件，可以自己设定
	}
};


struct SPoint_UV
{
	cv::Vec2d PointUV_Vector;//代表纹理坐标点(uv)向量矩阵
	vector<int> neighbor;//一环邻域纹理点标识

	SPoint_UV();
	bool hasNeighbor(int neiId);
	void addNeighbor(int neiId);

	bool operator == (const SPoint_UV & obj) const //重载 “==” 操作符，函数最后的 const 别忘了，否则会报错。
	{
		return PointUV_Vector == obj.PointUV_Vector;//具体匹配条件，可以自己设定
	}
};
#endif

