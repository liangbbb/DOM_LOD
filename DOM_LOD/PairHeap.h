#pragma once
#ifndef PAIRHEAP_H
#define PAIRHEAP_H

#include "Pair.h"
#include "PointSet.h"
#include <queue>
#include <vector>
#include <map>
#include "CU.h"

using namespace std;

struct PairHeap
{
	struct cmp
	{
		bool operator ()(Pair& p1, Pair& p2);
	};

	priority_queue<Pair, vector<Pair>, cmp> pairQueue;//边对队列（优先队列）
	map<pair<int, int>, bool> mapper;//边对（true/false）
	int count;//边对数目

	PairHeap();
	void setupHeap(PointSet* set, double Max_Distance);//基本二次误差测度
	void setupHeapUV(PointSet* set);//带纹理二次误差测度（纹理接缝处保留）
	void setupHeapUV_maxSim(PointSet* set);//带纹理二次误差测度（纹理接缝处不保留）
	void setupHeapQEM(PointSet* Set);//带纹理二次误差测度
	void setupHeapQEM1(PointSet* Set);//带纹理二次误差测度
	void setupHeapD_QEM(PointSet* Set);//顾及细节二次误差测度
	void setupHeapMSADO(PointSet* Set, CU cu);//外观驱动

	bool HeapClassify_No1(PointSet* set, Pair &toAdd, int V1, int V2, double Tex_Complexity);//边类别1，计算最佳点及折叠代价
	bool HeapClassify_No2(PointSet* set, Pair &toAdd);//边分类2，计算最佳点及折叠代价
	bool HeapClassify_No3(PointSet* set, Pair &toAdd, int V1, int V2, double Tex_Complexity);//边分类3，计算最佳点及折叠代价


	void addPair(Pair* p);
	void deletePair(Pair* p);//删除边对
	void skipPair(Pair* p);//跳过边对
	Pair top(PointSet* set);//获取顶端的边对

	void updata();//更新边操作对象
};
#endif
