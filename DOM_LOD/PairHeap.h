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

	priority_queue<Pair, vector<Pair>, cmp> pairQueue;//�߶Զ��У����ȶ��У�
	map<pair<int, int>, bool> mapper;//�߶ԣ�true/false��
	int count;//�߶���Ŀ

	PairHeap();
	void setupHeap(PointSet* set, double Max_Distance);//�������������
	void setupHeapUV(PointSet* set);//�������������ȣ�����ӷ촦������
	void setupHeapUV_maxSim(PointSet* set);//�������������ȣ�����ӷ촦��������
	void setupHeapQEM(PointSet* Set);//��������������
	void setupHeapQEM1(PointSet* Set);//��������������
	void setupHeapD_QEM(PointSet* Set);//�˼�ϸ�ڶ��������
	void setupHeapMSADO(PointSet* Set, CU cu);//�������

	bool HeapClassify_No1(PointSet* set, Pair &toAdd, int V1, int V2, double Tex_Complexity);//�����1��������ѵ㼰�۵�����
	bool HeapClassify_No2(PointSet* set, Pair &toAdd);//�߷���2��������ѵ㼰�۵�����
	bool HeapClassify_No3(PointSet* set, Pair &toAdd, int V1, int V2, double Tex_Complexity);//�߷���3��������ѵ㼰�۵�����


	void addPair(Pair* p);
	void deletePair(Pair* p);//ɾ���߶�
	void skipPair(Pair* p);//�����߶�
	Pair top(PointSet* set);//��ȡ���˵ı߶�

	void updata();//���±߲�������
};
#endif
