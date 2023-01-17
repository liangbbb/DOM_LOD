#include "PointSet.h"
#include "Point.h"
#include <iostream>

PointSet::PointSet()
{
	count = 0;
	count_uv = 0;
	maxpos = 0;
	maxpos_uv = 0;
	/*for (int i = 0; i < 9000000; ++i)
		enabled[i] = true;*/
		//enabled[0] = false;
}

int PointSet::addPoint(SPoint& _p)
{
	count++;
	point.push_back(_p);
	enabled.push_back(true);
	//Ptr_SPoint[maxpos] = maxpos;
	return maxpos++;
}

int PointSet::addPoint(SPoint & _p, int index)
{
	count++;
	point.push_back(_p);
	enabled.push_back(true);
	Ptr_SPoint[maxpos] = index;
	return maxpos++;
}

int PointSet::addPoint_uv(SPoint_UV & _p)
{
	count_uv++;
	point_uv.push_back(_p);
	enabled_uv.push_back(true);
	//Ptr_SPoint[maxpos] = maxpos;
	return maxpos_uv++;
}

int PointSet::addPoint_uv(SPoint_UV & _p, int index)
{
	count_uv++;
	point_uv.push_back(_p);
	enabled_uv.push_back(true);
	Ptr_SPoint_UV[maxpos_uv] = index;
	return maxpos_uv++;
}

void PointSet::calculateErrorMat()
{
	for (int p = 0; p < count; ++p) {
		point.at(p).calculateMat(this);
	}
}

void PointSet::calculateUVErrorMat()
{
	for (int p = 0; p < count; ++p) {
		point.at(p).calculateUVMat(this);
	}

	/*SPoint p3 = point.at(100);
	double lll = 0.0;
	for (int p = 0; p < 6; p++) {
		for (int q = 0; q < 6; q++) {
			lll = p3.Texture_error.at<double>(p, q);
		}
	}*/
}

void PointSet::calculateUVErrorMat1()
{
	for (int p = 0; p < count; ++p) {
		point.at(p).calculateUVMat1(this);
	}
}

void PointSet::calculateQEMUVMat()
{
	for (int p = 0; p < count; ++p) {
		point.at(p).calculateQEMMat(this);
	}
}

void PointSet::calculateD_QEMUVMat()
{
	for (int p = 0; p < count; ++p) {
		point.at(p).calculateD_QEMMat(this);
	}
}

void PointSet::calculateMSADOMat()
{
	for (int p = 0; p < count; ++p) {
		point.at(p).calculateMat(this);
	}
}


void PointSet::delPoint(int pos)
{
	enabled[pos] = false;
	count--;
}

void PointSet::delPoint_uv(int pos)
{
	enabled_uv[pos] = false;
	count_uv--;
}

int PointSet::Repeat(SPoint & _p)
{
	int index = 0;

	auto iter = std::find(point.begin(), point.end(), _p);
	if (iter != point.end())
	{
		index = std::distance(point.begin(), iter);
		//return index;
	}
	else
	{
		index = point.size();
	}

	/*for (int i = 0; i < point.size();i++) {
		if (point[i].Point_Vector  == _p.Point_Vector) {
			return index;
		}
		index++;
	}*/
	/*for (auto iter = point.cbegin(); iter != point.cend(); iter++) {
		if (iter->Point_Vector == _p.Point_Vector) {
			return index;
		}
		index++;
	}*/
	return index;
}

int PointSet::Repeat_UV(SPoint_UV & _p)
{
	int index = 0;

	auto iter = std::find(point_uv.begin(), point_uv.end(), _p);
	if (iter != point_uv.end())
	{
		index = std::distance(point_uv.begin(), iter);
		//return index;
	}
	else
	{
		index = point_uv.size();
	}
	return index;
}

void PointSet::updata()
{
	point.swap(vector<SPoint>());//清空vector<SPoint>point
	count = 0;//有效点的个数，更新为0
	maxpos = 0;//所有点的标识，更新为0
	enabled.clear();//清空deque<bool>enabled;

	point_uv.swap(vector<SPoint_UV>());//清空vector<SPoint_UV>point_uv
	count_uv = 0;//有效纹理点的个数，更新为0
	maxpos_uv = 0;//所有纹理点的标识，更新为0
	enabled_uv.clear();//清空deque<bool>enabled_uv

	Ptr_SPoint.erase(Ptr_SPoint.begin(), Ptr_SPoint.end());//清空map<int, int> Ptr_SPoint
}



