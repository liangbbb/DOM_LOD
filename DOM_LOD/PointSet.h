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
	vector<SPoint>point;//动态存储所有点
	deque<bool>enabled;//动态存储所有点的有效性
	//Point* point;
	int count;//有效点的个数，初始为0
	int maxpos;//所有点的标识，初始为0
	 //bool* enabled;

	vector<SPoint_UV>point_uv;//动态存储所有纹理点
	deque<bool>enabled_uv;//动态存储所有纹理点的有效性
	int count_uv;//有效纹理点的个数，初始为0
	int maxpos_uv;//所有纹理点的标识，初始为0

	PointSet();//存储所有点
	int addPoint(SPoint& _p);
	int addPoint(SPoint& _p, int index);

	int addPoint_uv(SPoint_UV& _p);
	int addPoint_uv(SPoint_UV& _p, int index);

	void calculateErrorMat();//计算基本二次误差矩阵（不带纹理扩展）
	void calculateUVErrorMat();//计算带纹理二次误差矩阵
	void calculateUVErrorMat1();//计算带纹理二次误差矩阵
	void calculateQEMUVMat();//计算二次误差矩阵（QSlim带纹理扩展）
	void calculateD_QEMUVMat();//计算带纹理二次误差矩阵（顾及细节）
	void calculateMSADOMat();//计算二次误差矩阵（外观驱动）

	void delPoint(int pos);
	void delPoint_uv(int pos);

	int Repeat(SPoint& _p);//判断点是否重复，并处理
	int Repeat_UV(SPoint_UV& _p);//判断UV点是否重复，并处理
	map<int, int> Ptr_SPoint;//
	map<int, int> Ptr_SPoint_UV;//

	void updata();//更新点操作对象
};
#endif
