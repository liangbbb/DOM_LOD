#include "Pair.h"
#include <cmath>
#include<FitPlane.h>
#include <iostream>


using namespace std;
using namespace cv;
Pair::Pair()
{
	v1 = 0;
	v2 = 0;
	Edge_type = 0;
	delCost = 0;
}

Pair::Pair(int _v1, int _v2)
{
	v1 = _v1;
	v2 = _v2;
	delCost = 0;
	Edge_type = 0;
	changeable = true;
}

void Pair::sort()
{
	if (v1 > v2)
	{
		int tmp = v1;
		v1 = v2;
		v2 = tmp;
	}
}

bool Pair::JudgeBoundary(PointSet * set)
{
	int count = 0;
	for (auto iter = set->point.at(v1).neighbor.cbegin(); iter != set->point.at(v1).neighbor.cend(); iter++)
	{
		int pppp = *iter;
		if (std::find(set->point.at(v2).neighbor.begin(), set->point.at(v2).neighbor.end(), *iter) != set->point.at(v2).neighbor.end()) {
			count++;
		}
	}
	/*for (std::vector<int>::const_iterator nIterator = set->point.at(v1).neighbor.begin(); nIterator != set->point.at(v1).neighbor.end(); nIterator++)
	{
		if (std::find(set->point.at(v2).neighbor.begin(), set->point.at(v2).neighbor.end(), *nIterator) != set->point.at(v2).neighbor.end())
			count++;
	}*/
	if (count == 1) {//Ϊ�߽��
		return true;
	}
	//if (count <= 0 || count > 3) {//������
	//	return true;
	//}
	//if (count == 3) {//Ϊ������ѹ�����
	//	return true;
	//}
	return false;
}

bool Pair::JudgeUVBoundary(PointSet * set, int &V1, int &V2)
{
	int UVnum_v1 = set->point.at(v1).SPointUV_index.size();
	int UVnum_v2 = set->point.at(v2).SPointUV_index.size();

	if (UVnum_v1 == 1 || UVnum_v2 == 1) {//���۵�
		if (UVnum_v1 == 1 && UVnum_v2 == 1) {
			V1 = v1;
			V2 = v2;
			return true;
		}
		else
		{
			if (UVnum_v1 == 1) {
				V1 = v1;
				V2 = -1;
				return true;
			}
			else
			{
				V1 = -1;
				V2 = v2;
				return true;
			}
		}
	}
	else
	{
		V1 = v1;
		V2 = v2;
		return false;//�����۵�
	}
}

int Pair::JudgeTex(PointSet * set, int & V1, int & V2)
{
	int UVnum_v1 = set->point.at(v1).SPointUV_index.size();
	int UVnum_v2 = set->point.at(v2).SPointUV_index.size();

	if (UVnum_v1 == 1 || UVnum_v2 == 1) {
		V1 = UVnum_v1;
		V2 = UVnum_v2;
		return 1;//p1,p2����һ�����ǵ������
	}
	else
	{
		if (UVnum_v1 >= 3 && UVnum_v2 >= 3) {
			V1 = UVnum_v1;
			V2 = UVnum_v2;
			return 2;//p1,p2��Ϊ������㣨�����۵���
		}
		else
		{
			V1 = UVnum_v1;
			V2 = UVnum_v2;
			return 3;//����
		}
	}
	return 0;
}

int Pair::JudgeStrategy(PointSet * set)
{
	int UVnum_v1 = set->point.at(v1).SPointUV_index.size();
	int UVnum_v2 = set->point.at(v2).SPointUV_index.size();

	if (UVnum_v1 == 1 && UVnum_v2 == 1) {//situation1
		Edge_type = 1;//����ʶ1
		return 1;
	}
	else
	{
		if (UVnum_v1 == 1 && UVnum_v2 == 2) {//situation2
			Edge_type = 2;//����ʶ2
			return 2;
		}
		if (UVnum_v1 == 2 && UVnum_v2 == 1) {//situation2
			Edge_type = 2;//����ʶ2
			return 2;
		}
		//if (UVnum_v1 == 1) {//situation2
		//	Edge_type = 2;//����ʶ2
		//	return 2;
		//}
		//if (UVnum_v2 == 1) {//situation2
		//	Edge_type = 2;//����ʶ2
		//	return 2;
		//}

		if (UVnum_v1 == 2 && UVnum_v2 == 2) {//situation3
			Edge_type = 3;//����ʶ3
			return 3;
		}
		return 4;
	}
}

int Pair::JudgeStrategy_MSADO(PointSet * set)
{
	int UVnum_v1 = set->point.at(v1).SPointUV_index.size();
	int UVnum_v2 = set->point.at(v2).SPointUV_index.size();

	if (UVnum_v1 == 1 && UVnum_v2 == 1) {//situation1
		Edge_type = 1;//����ʶ1
		return 1;
	}
	else
	{
		if (UVnum_v1 == 1 && UVnum_v2 == 2) {//situation2
			Edge_type = 2;//����ʶ2
			return 2;
		}
		if (UVnum_v1 == 2 && UVnum_v2 == 1) {//situation2
			Edge_type = 2;//����ʶ2
			return 2;
		}
		if (UVnum_v1 == 2 && UVnum_v2 == 2) {//situation3
			Edge_type = 3;//����ʶ3
			return 3;
		}
		return 4;
	}
}

double Pair::calculate_TexComplexity(PointSet * Set)
{
	double TexComplexity = 0.0;

	set<int> neighbors;//����v1,v2�ܵ��ھӼ�����
	for (int i = 0; i < (int)Set->point.at(v1).neighbor.size(); ++i)//�����߶�v1���һ���������е�
	{
		int k = Set->point.at(v1).neighbor[i];
		neighbors.insert(k);//v1������һ��������ʶ
		//if (k != v2)
		//	neighbors.insert(k);//����v2�⣬v1������һ��������ʶ
	}
	for (int i = 0; i < (int)Set->point.at(v2).neighbor.size(); ++i)//�����߶�v2���һ���������е�
	{
		int k = Set->point.at(v2).neighbor[i];
		neighbors.insert(k);//v2������һ��������ʶ
		//if (k != v1)
		//	neighbors.insert(k);//����v1�⣬v2������һ��������ʶ
	}

	double denominator = neighbors.size();//һ���������
	double molecule = 0;//һ�������������
	for (auto a : neighbors) {
		molecule += Set->point.at(a).SPointUV_index.size();
	}

	TexComplexity = molecule / denominator;

	return TexComplexity;
}

void Pair::calculateBestPoint(PointSet* set, bool &success)
{
	//vector<int> neighborP1;//�����ڵ㼯��
	//vector<int> neighborP2;//�������ڵ㼯��
	//for (auto iter = set->point.at(v1).neighbor.cbegin(); iter != set->point.at(v1).neighbor.cend(); iter++)
	//{
	//	int pppp = *iter;
	//	if (std::find(set->point.at(v2).neighbor.begin(), set->point.at(v2).neighbor.end(), *iter) != set->point.at(v2).neighbor.end()) {
	//		//�ҵ���ͬ�ĵ�
	//		neighborP1.push_back(*iter);
	//	}
	//	else
	//	{
	//		//δ�ҵ���ͬ�ĵ�
	//		if (*iter != v2) {//�ų�v2
	//			neighborP2.push_back(*iter);
	//		}
	//	}
	//}
	//for (auto iter = set->point.at(v2).neighbor.cbegin(); iter != set->point.at(v2).neighbor.cend(); iter++)
	//{
	//	if (std::find(set->point.at(v1).neighbor.begin(), set->point.at(v1).neighbor.end(), *iter) == set->point.at(v1).neighbor.end()) {
	//		//δ�ҵ���ͬ�ĵ�
	//		if (*iter != v1) {//�ų�v2
	//			neighborP2.push_back(*iter);
	//		}
	//	}
	//}

	//cv::Point2d UV1; 
	//int UV1count = neighborP1.size();
	//if (UV1count == 0) {
	//	int p = 0;
	//}
	//for (int i = 0; i < neighborP1.size(); i++) {
	//	UV1 = UV1 + set->point.at(neighborP1[i]).PointUV;
	//}
	//cv::Point2d UV3 = UV1 / UV1count;
	//cv::Point2d UV2;
	//int UV2count = neighborP2.size();
	//for (int i = 0; i < neighborP2.size(); i++) {
	//	UV2 = UV2 + set->point.at(neighborP2[i]).PointUV;
	//}
	//if (UV2count == 0) {
	//	int p = 0;
	//}
	//cv::Point2d UV4 = UV2 / UV2count;
	//cv::Point2d Point1 = set->point.at(v1).PointUV + set->point.at(v2).PointUV;
	//cv::Point2d Point2 = 0.50*Point1;
	//bestPoint.PointUV = Point2;//��������
	//bestPoint.PointUV = 0.5*(set->point.at(v1).PointUV + set->point.at(v2).PointUV);//��������


	//ֱ��ȡ�е�
	success = false;
	bestPoint.Point_Vector = 0.5*(set->point.at(v1).Point_Vector + set->point.at(v2).Point_Vector);//xyz����
	bestPoint.PointUV = 0.5*(set->point.at(v1).PointUV + set->point.at(v2).PointUV);//��������
	//bestPoint.PointUV = 0.5*(set->point.at(v1).PointUV + set->point.at(v2).PointUV) + 0.25*(UV3 - UV4);//��������

	//linear optimization���ø�˹��Ԫ��
	cv::Mat data = cv::Mat::zeros(4, 4, CV_64F);//��ʼ����ǰ���������
	/*double ppppp;
	cv::Mat data1 = set->point.at(v1).error;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			ppppp = data1.at<double>(i, j);
		}
	}
	cv::Mat data2 = set->point.at(v2).error;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			ppppp = data2.at<double>(i, j);
		}
	}*/
	data = set->point.at(v1).error + set->point.at(v2).error;//�ϲ��󶥵��������
	data.at<double>(3, 0) = 0;
	data.at<double>(3, 1) = 0;
	data.at<double>(3, 2) = 0;
	data.at<double>(3, 3) = 1;

	/*double ppp = data.at<double>(0, 0);
	 ppp = data.at<double>(0, 1);
	 ppp = data.at<double>(0, 2);
	 ppp = data.at<double>(0, 3);
	 ppp = data.at<double>(1, 0);
	 ppp = data.at<double>(1, 1);
	 ppp = data.at<double>(1, 2);
	 ppp = data.at<double>(1, 3);
	 ppp = data.at<double>(2, 0);*/

	 //Vector4 Y(Vector3(0, 0, 0), 1);

	for (int i = 0; i < 4; ++i)
		data.at<double>(i, 3) *= -1;

	//���ɽ���
	for (int i = 0; i < 3; ++i)//����ÿ������
	{
		int j;
		for (j = 0; j < 3; ++j)//���ڷ�����ߵ�ϵ��
			if (abs(data.at<double>(i, j)) >= 1e-6)//��⵽�в�Ϊ0��ϵ��
				break;

		if (j == 3)//���ϵ����Ϊ0
			return;
		for (int p = 0; p < 3; ++p)//���ڷ�ԭ���̵�ÿ������
			if (p != i)
			{
				double d = data.at<double>(p, j) / data.at<double>(i, j);
				for (int k = 0; k < 4; ++k)//����ÿ������
					data.at<double>(p, k) -= data.at<double>(i, k) * d;
			}
	}

	for (int i = 0; i < 3; ++i)
	{
		int count = 0;
		for (int j = 0; j < 3; ++j)
			if (abs(data.at<double>(i, j)) < 1e-6) count++;
		if (count == 3) return;
	}

	double index[3];
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (abs(data.at<double>(i, j)) > 1e-6)
				index[j] = data.at<double>(i, 3) / data.at<double>(i, j);
		}
	}

	bestPoint.Point_Vector = cv::Vec3d(index[0], index[1], index[2]);//��ѵ㸳ֵ
	success = true;
	//ȡ���������
	 /*double distance1;
	 cv::Vec3d Vec = bestPoint.Point_Vector - set->point.at(v1).Point_Vector;
	 distance1 = sqrtf(Vec[0] * Vec[0] + Vec[1] * Vec[1] + Vec[2] * Vec[2]);
	 double distance2;
	 cv::Vec3d Vec1 = bestPoint.Point_Vector - set->point.at(v2).Point_Vector;
	 distance2 = sqrtf(Vec1[0] * Vec1[0] + Vec1[1] * Vec1[1] + Vec1[2] * Vec1[2]);
	 if (distance1> distance2) {
		 bestPoint.PointUV = set->point.at(v2).PointUV;
	 }
	 else
	 {
		 bestPoint.PointUV = set->point.at(v1).PointUV;
	 }*/

}

void Pair::calculateBestPointUV(PointSet * set, bool & success)
{
	//ֱ��ȡ�е�
	success = false;
	bestPoint.Point_Vector = 0.5*(set->point.at(v1).Point_Vector + set->point.at(v2).Point_Vector);//xyz����
	bestPoint.PointUV_Vector[0] = 0.5*(set->point.at(v1).PointUV.x + set->point.at(v2).PointUV.x);//x
	bestPoint.PointUV_Vector[1] = 0.5*(set->point.at(v1).PointUV.y + set->point.at(v2).PointUV.y);//y
	bestPoint.PointUV = 0.5*(set->point.at(v1).PointUV + set->point.at(v2).PointUV);//��������

	//linear optimization���ø�˹��Ԫ��
	cv::Mat data = cv::Mat::zeros(6, 6, CV_64F);//��ʼ����ǰ���������
	data = set->point.at(v1).Texture_error + set->point.at(v2).Texture_error;//�ϲ��󶥵��������
	data.at<double>(5, 0) = 0;
	data.at<double>(5, 1) = 0;
	data.at<double>(5, 2) = 0;
	data.at<double>(5, 3) = 0;
	data.at<double>(5, 4) = 0;
	data.at<double>(5, 5) = 1;

	for (int i = 0; i < 6; ++i)
		data.at<double>(i, 5) *= -1;

	//���ɽ���
	for (int i = 0; i < 5; ++i)//����ÿ������
	{
		int j;
		for (j = 0; j < 5; ++j)//���ڷ�����ߵ�ϵ��
			if (abs(data.at<double>(i, j)) >= 1e-6)//��⵽�в�Ϊ0��ϵ��
				break;

		if (j == 5)//���ϵ����Ϊ0
			return;
		for (int p = 0; p < 5; ++p)//���ڷ�ԭ���̵�ÿ������
			if (p != i)
			{
				double d = data.at<double>(p, j) / data.at<double>(i, j);
				for (int k = 0; k < 6; ++k)//����ÿ������
					data.at<double>(p, k) -= data.at<double>(i, k) * d;
			}
	}

	for (int i = 0; i < 5; ++i)
	{
		int count = 0;
		for (int j = 0; j < 5; ++j)
			if (abs(data.at<double>(i, j)) < 1e-6) count++;
		if (count == 5) return;
	}

	double index[5];
	for (int i = 0; i < 5; ++i) {
		for (int j = 0; j < 5; ++j) {
			if (abs(data.at<double>(i, j)) > 1e-6)
				index[j] = data.at<double>(i, 5) / data.at<double>(i, j);
		}
	}


	bestPoint.Point_Vector = cv::Vec3d(index[0], index[1], index[2]);//��ѵ㸳ֵ
	bestPoint.PointUV_Vector[0] = index[3];//x
	bestPoint.PointUV_Vector[1] = index[4];//y
	bestPoint.PointUV = cv::Point2d(index[3], index[4]);//���UV�㸳ֵ
	success = true;
}

void Pair::calculateBestPointQEM(PointSet * Set, bool & success)
{
	//ֱ��ȡ�е�
	success = false;
	bestPoint.Point_Vector = 0.5*(Set->point.at(v1).Point_Vector + Set->point.at(v2).Point_Vector);//xyz����
	//bestPoint.PointUV_Vector[0] = 0.5*(Set->point.at(v1).PointUV.x + Set->point.at(v2).PointUV.x);//x
	//bestPoint.PointUV_Vector[1] = 0.5*(Set->point.at(v1).PointUV.y + Set->point.at(v2).PointUV.y);//y
	//bestPoint.PointUV = 0.5*(Set->point.at(v1).PointUV + Set->point.at(v2).PointUV);//��������

	int n = Set->point.at(v1).SPointUV_index.size() + Set->point.at(v2).SPointUV_index.size();
	for (set<int>::iterator it = Set->point.at(v1).SPointUV_index.begin(); it != Set->point.at(v1).SPointUV_index.end(); it++)
	{
		cout << *it << " occurs " << endl;
		bestPoint.PointUV_Vector[0] += Set->point_uv.at(*it).PointUV_Vector[0];
		bestPoint.PointUV_Vector[1] += Set->point_uv.at(*it).PointUV_Vector[1];
	}
	for (set<int>::iterator it = Set->point.at(v2).SPointUV_index.begin(); it != Set->point.at(v2).SPointUV_index.end(); it++)
	{
		bestPoint.PointUV_Vector[0] += Set->point_uv.at(*it).PointUV_Vector[0];
		bestPoint.PointUV_Vector[1] += Set->point_uv.at(*it).PointUV_Vector[1];
	}
	bestPoint.PointUV_Vector[0] = bestPoint.PointUV_Vector[0] / n;//U����
	bestPoint.PointUV_Vector[1] = bestPoint.PointUV_Vector[1] / n;//V����

	//linear optimization���ø�˹��Ԫ��
	cv::Mat data = cv::Mat::zeros(6, 6, CV_64F);//��ʼ����ǰ���������
	data = Set->point.at(v1).Texture_error + Set->point.at(v2).Texture_error;//�ϲ��󶥵��������
	data.at<double>(5, 0) = 0;
	data.at<double>(5, 1) = 0;
	data.at<double>(5, 2) = 0;
	data.at<double>(5, 3) = 0;
	data.at<double>(5, 4) = 0;
	data.at<double>(5, 5) = 1;

	for (int i = 0; i < 6; ++i)
		data.at<double>(i, 5) *= -1;

	//���ɽ���
	for (int i = 0; i < 5; ++i)//����ÿ������
	{
		int j;
		for (j = 0; j < 5; ++j)//���ڷ�����ߵ�ϵ��
			if (abs(data.at<double>(i, j)) >= 1e-6)//��⵽�в�Ϊ0��ϵ��
				break;

		if (j == 5)//���ϵ����Ϊ0
			return;
		for (int p = 0; p < 5; ++p)//���ڷ�ԭ���̵�ÿ������
			if (p != i)
			{
				double d = data.at<double>(p, j) / data.at<double>(i, j);
				for (int k = 0; k < 6; ++k)//����ÿ������
					data.at<double>(p, k) -= data.at<double>(i, k) * d;
			}
	}

	for (int i = 0; i < 5; ++i)
	{
		int count = 0;
		for (int j = 0; j < 5; ++j)
			if (abs(data.at<double>(i, j)) < 1e-6) count++;
		if (count == 5) return;
	}

	double index[5];
	for (int i = 0; i < 5; ++i) {
		for (int j = 0; j < 5; ++j) {
			if (abs(data.at<double>(i, j)) > 1e-6)
				index[j] = data.at<double>(i, 5) / data.at<double>(i, j);
		}
	}

	bestPoint.Point_Vector = cv::Vec3d(index[0], index[1], index[2]);//��ѵ㸳ֵ
	bestPoint.PointUV_Vector[0] = index[3];//U
	bestPoint.PointUV_Vector[1] = index[4];//V
	//bestPoint.PointUV = cv::Point2d(index[3], index[4]);//���UV�㸳ֵ
	success = true;
}

void Pair::calculateBestPointQEM1(PointSet * set, bool & success)
{
	//ֱ��ȡ�е�
	success = false;
	bestPoint.Point_Vector = 0.5*(set->point.at(v1).Point_Vector + set->point.at(v2).Point_Vector);//xyz����
	bestPoint.PointUV = 0.5*(set->point.at(v1).PointUV + set->point.at(v2).PointUV);//��������

	//linear optimization���ø�˹��Ԫ��
	cv::Mat data = cv::Mat::zeros(6, 6, CV_64F);//��ʼ����ǰ���������
	data = set->point.at(v1).Texture_error + set->point.at(v2).Texture_error;//�ϲ��󶥵��������
	data.at<double>(5, 0) = 0;
	data.at<double>(5, 1) = 0;
	data.at<double>(5, 2) = 0;
	data.at<double>(5, 3) = 0;
	data.at<double>(5, 4) = 0;
	data.at<double>(5, 5) = 1;

	for (int i = 0; i < 6; ++i)
		data.at<double>(i, 5) *= -1;

	//���ɽ���
	for (int i = 0; i < 5; ++i)//����ÿ������
	{
		int j;
		for (j = 0; j < 5; ++j)//���ڷ�����ߵ�ϵ��
			if (abs(data.at<double>(i, j)) >= 1e-6)//��⵽�в�Ϊ0��ϵ��
				break;

		if (j == 5)//���ϵ����Ϊ0
			return;
		for (int p = 0; p < 5; ++p)//���ڷ�ԭ���̵�ÿ������
			if (p != i)
			{
				double d = data.at<double>(p, j) / data.at<double>(i, j);
				for (int k = 0; k < 6; ++k)//����ÿ������
					data.at<double>(p, k) -= data.at<double>(i, k) * d;
			}
	}

	for (int i = 0; i < 5; ++i)
	{
		int count = 0;
		for (int j = 0; j < 5; ++j)
			if (abs(data.at<double>(i, j)) < 1e-6) count++;
		if (count == 5) return;
	}

	double index[5];
	for (int i = 0; i < 5; ++i) {
		for (int j = 0; j < 5; ++j) {
			if (abs(data.at<double>(i, j)) > 1e-6)
				index[j] = data.at<double>(i, 5) / data.at<double>(i, j);
		}
	}

	//bestPoint.Point_Vector = cv::Vec3d(index[0], index[1], index[2]);//��ѵ㸳ֵ

	//�쳣����
	cv::Vec3d P = cv::Vec3d(index[0], index[1], index[2]);
	cv::Vec3d tem = bestPoint.Point_Vector - P;
	double D = sqrt(tem[0] * tem[0] + tem[1] * tem[1] + tem[2] * tem[2]);
	if (D > 3) {
		success = false;
	}
	else
	{
		bestPoint.Point_Vector = cv::Vec3d(index[0], index[1], index[2]);//��ѵ㸳ֵ
		bestPoint.PointUV = cv::Point2d(index[3], index[4]);//���UV�㸳ֵ
		success = true;
	}
}

void Pair::calculateBestPointD_QEM(PointSet * Set, bool & success, int edge_type)
{
	int UVnum_v1 = Set->point.at(v1).SPointUV_index.size();//v1��������
	int UVnum_v2 = Set->point.at(v2).SPointUV_index.size();//v2��������

	switch (edge_type)
	{
	case 1://�����۵��������λ�ú�UV���н���㣬û���ȡ�е㣩
		calculateBestPointQEM(Set, success);
		break;
	case 2://����۵�
		int startP, endP;
		int p_v1uv;
		int uvsize;
		if (UVnum_v1 == 1) {//v1->v2
			startP = v1;
			endP = v2;
			strat_Point = v1;
			end_Point = v2;
		}
		else//v2->v1
		{
			startP = v2;
			endP = v1;
			strat_Point = v2;
			end_Point = v1;
		}
		bestPoint.Point_Vector = Set->point.at(endP).Point_Vector;//ȷ���۵���xyz����

		p_v1uv = *Set->point.at(startP).SPointUV_index.begin();//���������ʶ
		uvsize = Set->point.at(endP).SPointUV_index.size();//�յ���������
		for (auto uv : Set->point.at(endP).SPointUV_index) {//�����յ�������ʶ����
			if (Set->point_uv.at(uv).hasNeighbor(p_v1uv)) {//��������������ϵ
				bestPoint.PointUV_Vector = Set->point_uv.at(uv).PointUV_Vector;//ȷ���۵���uv����
				success = true;
			}
		}
		break;
	case 3://����۵����۵�λ��δ����
		break;
	case 4://��ֹ�۵�
		break;
	}
}

void Pair::calculateBestPointMSADO(PointSet * Set, bool & success, int edge_type)
{
	int UVnum_v1 = Set->point.at(v1).SPointUV_index.size();//v1��������
	int UVnum_v2 = Set->point.at(v2).SPointUV_index.size();//v2��������
	success = false;

	switch (edge_type)
	{
	case 1://�����۵��������λ�ã��н���㣬û���ȡ�е㣩,uv��ƽ��ֵ
		calculateBest(Set, success);
		break;
	case 2://����۵�
		int startP, endP;
		int p_v1uv;
		int uvsize;
		if (UVnum_v1 == 1) {//v1->v2
			startP = v1;
			endP = v2;
			strat_Point = v1;
			end_Point = v2;
		}
		else//v2->v1
		{
			startP = v2;
			endP = v1;
			strat_Point = v2;
			end_Point = v1;
		}
		bestPoint.Point_Vector = Set->point.at(endP).Point_Vector;//ȷ���۵���xyz����

		p_v1uv = *Set->point.at(startP).SPointUV_index.begin();//���������ʶ
		uvsize = Set->point.at(endP).SPointUV_index.size();//�յ���������
		for (auto uv : Set->point.at(endP).SPointUV_index) {//�����յ�������ʶ����
			if (Set->point_uv.at(uv).hasNeighbor(p_v1uv)) {//��������������ϵ
				bestPoint.PointUV_Vector = Set->point_uv.at(uv).PointUV_Vector;//ȷ���۵���uv����
				success = true;
			}
		}
		break;
	case 3://ƽ���۵���ȡ�е㣩
	{
		//ȷ���۵���xyz����
		bestPoint.Point_Vector = 0.5*(Set->point.at(v1).Point_Vector + Set->point.at(v2).Point_Vector);

		//����v1�����˳�������۵��������������
		int index = 0;
		int index1 = 0;
		for (auto uv1 : Set->point.at(v1).SPointUV_index) {//����v1������ʶ����
			index++;
			for (auto uv2 : Set->point.at(v2).SPointUV_index) {//����v2����������
				if (Set->point_uv.at(uv1).hasNeighbor(uv2)) {//��Գɹ�
					if (index == 1)
						bestPoint.PointUV_Vector = 0.5*(Set->point_uv.at(uv1).PointUV_Vector + Set->point_uv.at(uv2).PointUV_Vector);
					if (index == 2)
						bestPoint.PointUV_Vector2 = 0.5*(Set->point_uv.at(uv1).PointUV_Vector + Set->point_uv.at(uv2).PointUV_Vector);
					index1++;
					break;
				}
			}
		}
		if (index1 == 2) {
			success = true;//���������
		}
	}
	break;
	case 4://��ֹ�۵�
		break;
	}
}

void Pair::calculateBest(PointSet * Set, bool & success)
{
	//ֱ��ȡ�е�
	success = false;
	bestPoint.Point_Vector = 0.5*(Set->point.at(v1).Point_Vector + Set->point.at(v2).Point_Vector);//xyz����
	//bestPoint.PointUV = 0.5*(Set->point.at(v1).PointUV + Set->point.at(v2).PointUV);//��������

	int n = Set->point.at(v1).SPointUV_index.size() + Set->point.at(v2).SPointUV_index.size();
	for (set<int>::iterator it = Set->point.at(v1).SPointUV_index.begin(); it != Set->point.at(v1).SPointUV_index.end(); it++)
	{
		cout << *it << " occurs " << endl;
		bestPoint.PointUV_Vector[0] += Set->point_uv.at(*it).PointUV_Vector[0];
		bestPoint.PointUV_Vector[1] += Set->point_uv.at(*it).PointUV_Vector[1];
	}
	for (set<int>::iterator it = Set->point.at(v2).SPointUV_index.begin(); it != Set->point.at(v2).SPointUV_index.end(); it++)
	{
		bestPoint.PointUV_Vector[0] += Set->point_uv.at(*it).PointUV_Vector[0];
		bestPoint.PointUV_Vector[1] += Set->point_uv.at(*it).PointUV_Vector[1];
	}
	bestPoint.PointUV_Vector[0] = bestPoint.PointUV_Vector[0] / n;//U����
	bestPoint.PointUV_Vector[1] = bestPoint.PointUV_Vector[1] / n;//V����

	//linear optimization���ø�˹��Ԫ��
	cv::Mat data = cv::Mat::zeros(4, 4, CV_64F);//��ʼ����ǰ���������

	data = Set->point.at(v1).error + Set->point.at(v2).error;//�ϲ��󶥵��������
	data.at<double>(3, 0) = 0;
	data.at<double>(3, 1) = 0;
	data.at<double>(3, 2) = 0;
	data.at<double>(3, 3) = 1;

	for (int i = 0; i < 4; ++i)
		data.at<double>(i, 3) *= -1;

	//���ɽ���
	for (int i = 0; i < 3; ++i)//����ÿ������
	{
		int j;
		for (j = 0; j < 3; ++j)//���ڷ�����ߵ�ϵ��
			if (abs(data.at<double>(i, j)) >= 1e-6)//��⵽�в�Ϊ0��ϵ��
				break;

		if (j == 3)//���ϵ����Ϊ0
			return;
		for (int p = 0; p < 3; ++p)//���ڷ�ԭ���̵�ÿ������
			if (p != i)
			{
				double d = data.at<double>(p, j) / data.at<double>(i, j);
				for (int k = 0; k < 4; ++k)//����ÿ������
					data.at<double>(p, k) -= data.at<double>(i, k) * d;
			}
	}

	for (int i = 0; i < 3; ++i)
	{
		int count = 0;
		for (int j = 0; j < 3; ++j)
			if (abs(data.at<double>(i, j)) < 1e-6) count++;
		if (count == 3) return;
	}

	double index[3];
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (abs(data.at<double>(i, j)) > 1e-6)
				index[j] = data.at<double>(i, 3) / data.at<double>(i, j);
		}
	}

	bestPoint.Point_Vector = cv::Vec3d(index[0], index[1], index[2]);//��ѵ㸳ֵ
	success = true;
}


double Pair::calculateMaxNormalDeviation(PointSet * set)
{
	double Max_error = 0;
	//P1
	SPoint P1 = set->point.at(v1);
	int P1_neigsize = (int)set->point.at(v1).neighbor.size();
	for (int i = 0; i < P1_neigsize; ++i)//����P1���һ�����
	{
		SPoint temp1 = set->point[set->point.at(v1).neighbor[i]];
		for (int j = i + 1; j < P1_neigsize; ++j)
		{
			int temp2_index = set->point.at(v1).neighbor[j];
			if (temp1.hasNeighbor(temp2_index) && temp2_index != v2)//�ж��Ƿ����һ��������,�Ҳ�����ɾ����������
			{
				SPoint temp2 = set->point[temp2_index];
				cv::Vec3d Normal1 = (temp1.Point_Vector - temp2.Point_Vector).cross(P1.Point_Vector - temp1.Point_Vector);
				cv::Vec3d Unit_Normal1 = Normal1 / sqrt(Normal1[0] * Normal1[0] + Normal1[1] * Normal1[1] + Normal1[2] * Normal1[2]);//ԭʼ�����浥λ������
				double length = sqrt(Normal1[0] * Normal1[0] + Normal1[1] * Normal1[1] + Normal1[2] * Normal1[2]);
				cv::Vec3d Unit_Norm = Normal1 / length;

				cv::Vec3d Normal2 = (temp1.Point_Vector - temp2.Point_Vector).cross(bestPoint.Point_Vector - temp1.Point_Vector);
				cv::Vec3d Unit_Normal2 = Normal2 / sqrt(Normal2[0] * Normal2[0] + Normal2[1] * Normal2[1] + Normal2[2] * Normal2[2]);//���������浥λ������
				double error = 1 - Unit_Normal1.dot(Unit_Normal2);//�Ƕ����
				if (error > Max_error) {
					Max_error = error;
				}
			}
		}
	}
	//P2
	SPoint P2 = set->point.at(v2);
	int P2_neigsize = (int)set->point.at(v2).neighbor.size();
	for (int i = 0; i < P2_neigsize; ++i)//����P2���һ�����
	{
		SPoint temp1 = set->point[set->point.at(v2).neighbor[i]];
		for (int j = i + 1; j < P2_neigsize; ++j)
		{
			int temp2_index = set->point.at(v2).neighbor[j];
			if (temp1.hasNeighbor(temp2_index) && temp2_index != v1)//�ж��Ƿ����һ��������,�Ҳ�����ɾ����������
			{
				SPoint temp2 = set->point[temp2_index];
				cv::Vec3d Normal1 = (temp1.Point_Vector - temp2.Point_Vector).cross(P2.Point_Vector - temp1.Point_Vector);
				cv::Vec3d Unit_Normal1 = Normal1 / sqrt(Normal1[0] * Normal1[0] + Normal1[1] * Normal1[1] + Normal1[2] * Normal1[2]);//ԭʼ�����浥λ������

				cv::Vec3d Normal2 = (temp1.Point_Vector - temp2.Point_Vector).cross(bestPoint.Point_Vector - temp1.Point_Vector);
				cv::Vec3d Unit_Normal2 = Normal2 / sqrt(Normal2[0] * Normal2[0] + Normal2[1] * Normal2[1] + Normal2[2] * Normal2[2]);//���������浥λ������
				double error = 1 - Unit_Normal1.dot(Unit_Normal2);//�Ƕ����
				if (error > Max_error) {
					Max_error = error;
				}
			}
		}
	}
	return Max_error;
}

bool Pair::JudgeAbnormalCollpase(PointSet * set)
{
	SPoint P1 = set->point.at(v1);
	SPoint P2 = set->point.at(v2);
	cv::Vec3d P = 0.5*(P1.Point_Vector + P2.Point_Vector);
	cv::Vec3d tem = bestPoint.Point_Vector - P;
	double D = sqrt(tem[0] * tem[0] + tem[1] * tem[1] + tem[2] * tem[2]);
	if (D > 6) {
		return true;
	}
	else
	{
		return false;
	}

	//double Max_x = -999999999999;
	//double Max_y = -999999999999;
	//double Max_z = -999999999999;
	//double Min_x = 999999999999;
	//double Min_y = 999999999999;
	//double Min_z = 999999999999;

	////P1
	//SPoint P1 = set->point.at(v1);
	//int P1_neigsize = (int)set->point.at(v1).neighbor.size();
	//for (int i = 0; i < P1_neigsize; ++i)//����P1���һ�����
	//{
	//	SPoint temp1 = set->point[set->point.at(v1).neighbor[i]];
	//	temp1.Point_Vector[0] > Max_x ? Max_x = temp1.Point_Vector[0] : Max_x = Max_x;
	//	temp1.Point_Vector[0] < Min_x ? Min_x = temp1.Point_Vector[0] : Min_x = Min_x;
	//	temp1.Point_Vector[1] > Max_y ? Max_y = temp1.Point_Vector[1] : Max_y = Max_y;
	//	temp1.Point_Vector[1] < Min_y ? Min_y = temp1.Point_Vector[1] : Min_y = Min_y;
	//	temp1.Point_Vector[2] > Max_z ? Max_z = temp1.Point_Vector[2] : Max_z = Max_z;
	//	temp1.Point_Vector[2] < Min_z ? Min_z = temp1.Point_Vector[2] : Min_z = Min_z;
	//}

	////P2
	//SPoint P2 = set->point.at(v2);
	//int P2_neigsize = (int)set->point.at(v2).neighbor.size();
	//for (int i = 0; i < P2_neigsize; ++i)//����P2���һ�����
	//{
	//	SPoint temp2 = set->point[set->point.at(v2).neighbor[i]];
	//	temp2.Point_Vector[0] > Max_x ? Max_x = temp2.Point_Vector[0] : Max_x = Max_x;
	//	temp2.Point_Vector[0] < Min_x ? Min_x = temp2.Point_Vector[0] : Min_x = Min_x;
	//	temp2.Point_Vector[1] > Max_y ? Max_y = temp2.Point_Vector[1] : Max_y = Max_y;
	//	temp2.Point_Vector[1] < Min_y ? Min_y = temp2.Point_Vector[1] : Min_y = Min_y;
	//	temp2.Point_Vector[2] > Max_z ? Max_z = temp2.Point_Vector[2] : Max_z = Max_z;
	//	temp2.Point_Vector[2] < Min_z ? Min_z = temp2.Point_Vector[2] : Min_z = Min_z;
	//}

	//double distance_x = Max_x - Min_x;
	//double distance_y = Max_y - Min_y;
	//double distance_z = Max_z - Min_z;

	//bool b1 = bestPoint.Point_Vector[0] > (Min_x - 1000 * distance_x) && bestPoint.Point_Vector[0] < (Max_x + 1000 * distance_x);
	//bool b2 = bestPoint.Point_Vector[1] > (Min_y - 1000 * distance_y) && bestPoint.Point_Vector[1] < (Max_y + 1000 * distance_y);
	//bool b3 = bestPoint.Point_Vector[2] > (Min_z - 1000 * distance_z) && bestPoint.Point_Vector[2] < (Max_z + 1000 * distance_z);

	//if (b1&&b2&&b3) {
	//	return false;
	//}
	//else//�쳣
	//{
	//	return true;
	//}

}

bool Pair::ChangeTopology(PointSet * set)
{
	//p1
	bool topologychange;
	SPoint P1 = set->point.at(v1);
	int P1_neigsize = (int)set->point.at(v1).neighbor.size();
	int count = 0;
	for (int i = 0; i < P1_neigsize; ++i)//����P1���һ�����
	{
		SPoint temp1 = set->point[set->point.at(v1).neighbor[i]];
		if (temp1.hasNeighbor(v2))//�ж��Ƿ����һ��������,�Ҳ�����ɾ����������
		{
			count++;
		}
	}
	if (count <= 2) {
		topologychange == false;
	}
	else
	{
		topologychange == true;
	}
	return topologychange;
}

bool Pair::calculateVertexSharpness(PointSet * Set, int v1, double &Sharpness)
{

	bool abnormal = false;

	//P1
	int P1_neigsize = (int)Set->point.at(v1).neighbor.size();//ȷ��һ�������������������θ���
	vector<int> neighborP;
	SPoint P1 = Set->point.at(v1);

	//��ʼ�������漯��
	for (int i = 0; i < P1_neigsize; ++i)//����P1���һ�����
	{
		SPoint temp1 = Set->point[Set->point.at(v1).neighbor[i]];
		for (int j = i + 1; j < P1_neigsize; ++j) {
			int temp2_index = Set->point.at(v1).neighbor[j];
			if (temp1.hasNeighbor(temp2_index))//�ж��Ƿ����һ��������
			{
				neighborP.push_back(Set->point.at(v1).neighbor[i]);
				neighborP.push_back(temp2_index);
			}
		}
	}

	//�����쳣���������ܵ��ӳٴ���
	if (neighborP.size() != P1_neigsize * 2) {
		abnormal = true;
		Sharpness = 100;
		return abnormal;
	}

	if (neighborP.size() != P1_neigsize * 2) {
		/*abnormal = true;
		Sharpness = 1000000;
		return abnormal;*/
		if (neighborP.size() != P1_neigsize * 2 + 2) {
			int uuu = 66;
		}
		//Set->point.at(v1).point_changeable = false;//v1������Ϊ�����۵�
		//return 1000;
		//����쳣ֵ
		set<int> Outlier;
		for (int i = 0; i < neighborP.size(); i++) {
			int Detection_target = neighborP[i];//���Ŀ��
			int count = 0;
			for (int j = 0; j < neighborP.size(); j++) {
				if (Detection_target == neighborP[j]) {
					count++;
				}
			}
			if (count == 3) {
				Outlier.insert(Detection_target);
			}
		}

		/*if (Outlier.begin()== Outlier.end()) {
			Sharpness = 1000000;
			return abnormal;
		}*/

		//����
		for (int i = 0; i < neighborP.size(); i++) {
			if (neighborP[i] == *Outlier.begin()) {
				if (i % 2 == 0) {
					if (neighborP[i + 1] == *--Outlier.end()) {
						neighborP.erase(neighborP.begin() + i, neighborP.begin() + i + 2);
					}
				}
				else
				{
					if (neighborP[i - 1] == *--Outlier.end()) {
						neighborP.erase(neighborP.begin() + i - 1, neighborP.begin() + i + 1);
					}
				}
			}
		}
	}

	/*if ((neighborP.size() != P1_neigsize * 2) && (neighborP.size() != P1_neigsize * 2 + 2)) {
		return 1;
	}*/
	if (neighborP.size() <= 2) {
		//return 100000000;
	}

	//��ʼ�������淽��
	vector<int> NewneighborP;
	int startP = 0;
	int endP = 1;
	NewneighborP.push_back(neighborP[0]);
	NewneighborP.push_back(neighborP[1]);
	int newstartP = neighborP[endP];
	for (int j = 0; j < neighborP.size(); j++) {
		if (neighborP[j] == newstartP && (j != endP)) {
			if ((j + 1) % 2 == 0) {
				startP = j;
				endP = j - 1;
			}
			else
			{
				startP = j;
				endP = j + 1;
			}
			NewneighborP.push_back(neighborP[startP]);
			NewneighborP.push_back(neighborP[endP]);
			j = -1;//��ͷ��ʼ
			newstartP = neighborP[endP];
		}
		if (newstartP == neighborP[0]) {
			break;
		}
	}

	//�����ε㣨�����ܵ��ӳٴ���
	if (NewneighborP.size() != neighborP.size()) {
		abnormal = true;
		Sharpness = 100;
		return abnormal;
	}

	//���������������
	double AreaSum = 0;
	for (int i = 0; i < P1_neigsize; i++) {
		cv::Vec3d p111 = Set->point.at(NewneighborP[2 * i]).Point_Vector;//��1
		cv::Vec3d p222 = Set->point.at(v1).Point_Vector;//��2
		cv::Vec3d p333 = Set->point.at(NewneighborP[2 * i + 1]).Point_Vector;//��3

		double Area = calculateTriangularArea(p111, p222, p333);//���������
		AreaSum = AreaSum + Area;
	}

	//��ѭ�����㷨ʸ�н�
	double angle = 0.0;//��ʸ�нǣ������ƣ�
	for (int i = 0; i < P1_neigsize; i++) {
		int T1_index = 2 * i;
		cv::Vec3d p1 = Set->point.at(NewneighborP[T1_index]).Point_Vector;//��1
		cv::Vec3d p2 = Set->point.at(v1).Point_Vector;//��2
		cv::Vec3d p3 = Set->point.at(NewneighborP[T1_index + 1]).Point_Vector;//��3

		cv::Vec3d V1 = p2 - p1;//����1
		cv::Vec3d V2 = p3 - p2;//����2

		cv::Vec3d vn1 = V1.cross(V2);//������
		double L1 = sqrt(vn1.dot(vn1));
		vn1[0] = vn1[0] / L1;//��λ��
		vn1[1] = vn1[1] / L1;
		vn1[2] = vn1[2] / L1;

		double Area1 = calculateTriangularArea(p1, p2, p3);//������1���

		for (int j = i + 1; j < P1_neigsize; j++) {
			int T2_index = 2 * j;

			cv::Vec3d p11 = Set->point.at(NewneighborP[T2_index]).Point_Vector;//��1
			cv::Vec3d p22 = Set->point.at(v1).Point_Vector;//��2
			cv::Vec3d p33 = Set->point.at(NewneighborP[T2_index + 1]).Point_Vector;//��3

			cv::Vec3d V11 = p22 - p11;//����1
			cv::Vec3d V22 = p33 - p22;//����2

			cv::Vec3d vn11 = V11.cross(V22);//������
			double L11 = sqrt(vn11.dot(vn11));
			vn11[0] = vn11[0] / L11;//��λ��
			vn11[1] = vn11[1] / L11;
			vn11[2] = vn11[2] / L11;

			//angle = acos(vn1.dot(vn11));//�н�
			double Area2 = calculateTriangularArea(p11, p22, p33);//������2���
			angle = (Area1 + Area2) / (2 * AreaSum)*acos(vn1.dot(vn11)) + angle;//��Ȩ

		}
	}
	Sharpness = angle;
	return abnormal;
}

bool Pair::CalculateVertexSharpness(PointSet * Set, int v1, double & Sharpness)
{
	bool abnormal = false;
	//P1
	int P1_neigsize = (int)Set->point.at(v1).neighbor.size();//ȷ��һ�������������������θ���
	vector<int> neighborP;
	SPoint P1 = Set->point.at(v1);

	//��ʼ�������漯��
	for (int i = 0; i < P1_neigsize; ++i)//����P1���һ�����
	{
		SPoint temp1 = Set->point[Set->point.at(v1).neighbor[i]];
		for (int j = i + 1; j < P1_neigsize; ++j) {
			int temp2_index = Set->point.at(v1).neighbor[j];
			if (temp1.hasNeighbor(temp2_index))//�ж��Ƿ����һ��������
			{
				neighborP.push_back(Set->point.at(v1).neighbor[i]);
				neighborP.push_back(temp2_index);
			}
		}
	}

	//�����쳣��

	if (neighborP.size() != P1_neigsize * 2) {
		/*abnormal = true;
		Sharpness = 1000000;
		return abnormal;*/
		if (neighborP.size() != P1_neigsize * 2 + 2) {
			int uuu = 66;
		}
		//Set->point.at(v1).point_changeable = false;//v1������Ϊ�����۵�
		//return 1000;
		//����쳣ֵ
		set<int> Outlier;
		for (int i = 0; i < neighborP.size(); i++) {
			int Detection_target = neighborP[i];//���Ŀ��
			int count = 0;
			for (int j = 0; j < neighborP.size(); j++) {
				if (Detection_target == neighborP[j]) {
					count++;
				}
			}
			if (count == 3) {
				Outlier.insert(Detection_target);
			}
		}

		/*if (Outlier.begin()== Outlier.end()) {
			Sharpness = 1000000;
			return abnormal;
		}*/

		//����
		for (int i = 0; i < P1_neigsize; ++i)//����P1���һ�����
		{
			SPoint temp1 = Set->point[Set->point.at(v1).neighbor[i]];
			if (temp1.hasNeighbor(*Outlier.begin()) && temp1.hasNeighbor(*--Outlier.end())) {
				Set->point[Set->point.at(v1).neighbor[i]].removeNeighbor(*Outlier.begin());
				Set->point[Set->point.at(v1).neighbor[i]].removeNeighbor(*--Outlier.end());
				Set->point[Set->point.at(v1).neighbor[i]].removeNeighbor(v1);
				Set->point[*Outlier.begin()].removeNeighbor(Set->point.at(v1).neighbor[i]);
				Set->point[*--Outlier.end()].removeNeighbor(Set->point.at(v1).neighbor[i]);
				Set->point[v1].removeNeighbor(Set->point.at(v1).neighbor[i]);
			}
		}

		vector<int>().swap(neighborP);//���neighborP
		//�ٴγ�ʼ�������漯��
		P1_neigsize = (int)Set->point.at(v1).neighbor.size();//ȷ��һ�������������������θ���
		for (int i = 0; i < P1_neigsize; ++i)//����P1���һ�����
		{
			SPoint temp11 = Set->point[Set->point.at(v1).neighbor[i]];
			for (int j = i + 1; j < P1_neigsize; ++j) {
				int temp2_index = Set->point.at(v1).neighbor[j];
				if (temp11.hasNeighbor(temp2_index))//�ж��Ƿ����һ��������
				{
					neighborP.push_back(Set->point.at(v1).neighbor[i]);
					neighborP.push_back(temp2_index);
				}
			}
		}
	}

	//��ʼ�������淽��
	vector<int> NewneighborP;
	int startP = 0;
	int endP = 1;
	NewneighborP.push_back(neighborP[0]);
	NewneighborP.push_back(neighborP[1]);
	int newstartP = neighborP[endP];
	for (int j = 0; j < neighborP.size(); j++) {
		if (neighborP[j] == newstartP && (j != endP)) {
			if ((j + 1) % 2 == 0) {
				startP = j;
				endP = j - 1;
			}
			else
			{
				startP = j;
				endP = j + 1;
			}
			NewneighborP.push_back(neighborP[startP]);
			NewneighborP.push_back(neighborP[endP]);
			j = -1;//��ͷ��ʼ
			newstartP = neighborP[endP];
		}
		if (newstartP == neighborP[0]) {
			break;
		}
	}

	//�����ε�
	if (NewneighborP.size() != neighborP.size()) {
		//Set->point.at(v1).point_changeable = false;//v1������Ϊ�����۵�
		abnormal = true;
		Sharpness = 1000000;
		return abnormal;
	}

	//���������������
	double AreaSum = 0;
	for (int i = 0; i < P1_neigsize; i++) {
		cv::Vec3d p111 = Set->point.at(NewneighborP[2 * i]).Point_Vector;//��1
		cv::Vec3d p222 = Set->point.at(v1).Point_Vector;//��2
		cv::Vec3d p333 = Set->point.at(NewneighborP[2 * i + 1]).Point_Vector;//��3

		double Area = calculateTriangularArea(p111, p222, p333);//���������
		AreaSum = AreaSum + Area;
	}

	//��ѭ�����㷨ʸ�н�
	double angle = 0.0;//��ʸ�нǣ������ƣ�
	for (int i = 0; i < P1_neigsize; i++) {
		int T1_index = 2 * i;
		cv::Vec3d p1 = Set->point.at(NewneighborP[T1_index]).Point_Vector;//��1
		cv::Vec3d p2 = Set->point.at(v1).Point_Vector;//��2
		cv::Vec3d p3 = Set->point.at(NewneighborP[T1_index + 1]).Point_Vector;//��3

		cv::Vec3d V1 = p2 - p1;//����1
		cv::Vec3d V2 = p3 - p2;//����2

		cv::Vec3d vn1 = V1.cross(V2);//������
		double L1 = sqrt(vn1.dot(vn1));
		vn1[0] = vn1[0] / L1;//��λ��
		vn1[1] = vn1[1] / L1;
		vn1[2] = vn1[2] / L1;

		double Area1 = calculateTriangularArea(p1, p2, p3);//������1���

		for (int j = i + 1; j < P1_neigsize; j++) {
			int T2_index = 2 * j;

			cv::Vec3d p11 = Set->point.at(NewneighborP[T2_index]).Point_Vector;//��1
			cv::Vec3d p22 = Set->point.at(v1).Point_Vector;//��2
			cv::Vec3d p33 = Set->point.at(NewneighborP[T2_index + 1]).Point_Vector;//��3

			cv::Vec3d V11 = p22 - p11;//����1
			cv::Vec3d V22 = p33 - p22;//����2

			cv::Vec3d vn11 = V11.cross(V22);//������
			double L11 = sqrt(vn11.dot(vn11));
			vn11[0] = vn11[0] / L11;//��λ��
			vn11[1] = vn11[1] / L11;
			vn11[2] = vn11[2] / L11;

			//angle = acos(vn1.dot(vn11));//�н�
			double Area2 = calculateTriangularArea(p11, p22, p33);//������2���
			angle = (Area1 + Area2) / (2 * AreaSum)*acos(vn1.dot(vn11)) + angle;//��Ȩ

		}
	}
	Sharpness = angle;
	return abnormal;
}

double Pair::calculateTriangularArea(cv::Vec3d P1, cv::Vec3d P2, cv::Vec3d P3)
{
	double area = 0.0;
	double dis;//�����εĸ�
	double side[3];//�洢�����ߵĳ���;

	side[0] = sqrt(pow(P1[0] - P2[0], 2) + pow(P1[1] - P2[1], 2) + pow(P1[2] - P2[2], 2));
	side[1] = sqrt(pow(P1[0] - P3[0], 2) + pow(P1[1] - P3[1], 2) + pow(P1[2] - P3[2], 2));
	side[2] = sqrt(pow(P3[0] - P2[0], 2) + pow(P3[1] - P2[1], 2) + pow(P3[2] - P2[2], 2));

	double p = (side[0] + side[1] + side[2]) / 2; //���ܳ�;
	area = sqrt(p * (p - side[0]) * (p - side[1]) * (p - side[2]));//���

	return area;
}

bool Pair::calculateSeamError(PointSet * set, int v1, double & SeamError)
{
	double E;
	int mun_uv = set->point.at(v1).SPointUV_index.size();//��������
	E = set->point.at(v1).SeamAngleError(set, mun_uv);//�Ƕ����
	SeamError = E;
	return true;
}


void Pair::calculateBestUV(PointSet * set)
{
	double MinDistance = 99999999999;
	cv::Vec3d Effective_plane[3];//ȷ��������XYZ
	cv::Point2d Effective_plane_UV[3];//ȷ��������UV
	cv::Vec3d Effective_point;//ȷ��ͶӰ��

	int P1_neigsize = (int)set->point.at(v1).neighbor.size();
	for (int i = 0; i < P1_neigsize; ++i)//����P1���һ�����
	{
		SPoint temp1 = set->point[set->point.at(v1).neighbor[i]];
		for (int j = i + 1; j < P1_neigsize; ++j)
		{
			int temp2_index = set->point.at(v1).neighbor[j];
			if (temp1.hasNeighbor(temp2_index))//�ж��Ƿ����һ��������
			{
				cv::Vec3d Point1 = set->point.at(v1).Point_Vector;//P1
				cv::Vec3d Point2 = temp1.Point_Vector;//P2
				cv::Vec3d Point3 = set->point.at(temp2_index).Point_Vector;//P3
				cv::Vec3d plane[3] = { Point1 ,Point2 ,Point3 };//������

				cv::Vec3d ProjectionPoint = calculateProjectionPoint(plane, bestPoint.Point_Vector);//����ͶӰ��
				if (JudgeInclusion(plane, ProjectionPoint)) {//�ж�ͶӰ���Ƿ���Ч
					cv::Vec3d tem = ProjectionPoint - bestPoint.Point_Vector;
					double Distance = sqrt(tem.dot(tem));//�������
					if (Distance < MinDistance) {
						MinDistance = Distance;
						Effective_point = ProjectionPoint;
						Effective_plane[0] = Point1;
						Effective_plane[1] = Point2;
						Effective_plane[2] = Point3;
						Effective_plane_UV[0] = set->point.at(v1).PointUV;
						Effective_plane_UV[1] = temp1.PointUV;
						Effective_plane_UV[2] = set->point.at(temp2_index).PointUV;
					}
				}
			}
		}
	}

	int P2_neigsize = (int)set->point.at(v2).neighbor.size();
	for (int i = 0; i < P2_neigsize; ++i)//����P1���һ�����
	{
		SPoint temp1 = set->point[set->point.at(v2).neighbor[i]];
		for (int j = i + 1; j < P2_neigsize; ++j)
		{
			int temp2_index = set->point.at(v2).neighbor[j];
			if (temp1.hasNeighbor(temp2_index))//�ж��Ƿ����һ��������
			{
				cv::Vec3d Point1 = set->point.at(v2).Point_Vector;//P1
				cv::Vec3d Point2 = temp1.Point_Vector;//P2
				cv::Vec3d Point3 = set->point.at(temp2_index).Point_Vector;//P3
				cv::Vec3d plane[3] = { Point1 ,Point2 ,Point3 };//������

				cv::Vec3d ProjectionPoint = calculateProjectionPoint(plane, bestPoint.Point_Vector);//����ͶӰ��
				if (JudgeInclusion(plane, ProjectionPoint)) {//�ж�ͶӰ���Ƿ���Ч
					cv::Vec3d tem = ProjectionPoint - bestPoint.Point_Vector;
					double Distance = sqrt(tem.dot(tem));//�������
					if (Distance < MinDistance) {
						MinDistance = Distance;
						Effective_point = ProjectionPoint;
						Effective_plane[0] = Point1;
						Effective_plane[1] = Point2;
						Effective_plane[2] = Point3;
						Effective_plane_UV[0] = set->point.at(v1).PointUV;
						Effective_plane_UV[1] = temp1.PointUV;
						Effective_plane_UV[2] = set->point.at(temp2_index).PointUV;
					}
				}
			}
		}
	}
	//�������UV
	if (Effective_point[0] == 0) {

	}
	else
	{
		osg::Matrix R, R_i;
		computeRotationMatrix(Effective_plane, R, R_i);//������ת���������
		osg::Vec3d point1, point2, point3, P;
		point1 = getXYZ(Effective_plane[0], R);//��ת
		point2 = getXYZ(Effective_plane[1], R);
		point3 = getXYZ(Effective_plane[2], R);
		P = getXYZ(Effective_point, R);
		osg::Vec3d plane1[3] = { point1 ,point2 ,point3 };
		//�����任����,����������
		affineTransformMatrix(plane1, Effective_plane_UV, P, bestPoint.PointUV);
	}

}

cv::Vec3d Pair::calculateProjectionPoint(cv::Vec3d plane[3], cv::Vec3d P)
{
	cv::Vec3d AB = plane[0] - plane[1];
	cv::Vec3d AC = plane[0] - plane[2];
	cv::Vec3d Normal = AB.cross(AC);

	cv::Vec3d ProjectionPoint;
	ProjectionPoint[0] = (Normal[0] * Normal[1] * plane[0][1] + Normal[1] * Normal[1] * P[0] - Normal[0] * Normal[1] * P[1] + Normal[0] * Normal[2] * plane[0][2] + Normal[2] * Normal[2] * P[0] - Normal[0] * Normal[2] * P[2] + Normal[0] * Normal[0] * plane[0][0]) / Normal.dot(Normal);
	ProjectionPoint[1] = (Normal[1] * Normal[2] * plane[0][2] + Normal[2] * Normal[2] * P[1] - Normal[1] * Normal[2] * P[2] + Normal[1] * Normal[0] * plane[0][0] + Normal[0] * Normal[0] * P[1] - Normal[0] * Normal[1] * P[0] + Normal[1] * Normal[1] * plane[0][1]) / Normal.dot(Normal);
	ProjectionPoint[2] = (Normal[0] * plane[0][0] * Normal[2] + Normal[0] * Normal[0] * P[2] - Normal[0] * P[0] * Normal[2] + Normal[1] * plane[0][1] * Normal[2] + Normal[1] * Normal[1] * P[2] - Normal[1] * P[1] * Normal[2] + Normal[2] * Normal[2] * plane[0][2]) / Normal.dot(Normal);
	return ProjectionPoint;
}

bool Pair::JudgeInclusion(cv::Vec3d plane[3], cv::Vec3d P)
{
	bool toLeft = true;
	for (int i = 0; i < 3; i++) {
		cv::Vec3d Vec = plane[(i + 1) % 3] - plane[i];
		cv::Vec3d Vec1 = P - plane[i];
		if (i != 0) {
			if (toLeft != Vec[0] * Vec1[1] - Vec[1] * Vec1[0] > 0) {
				return false;
			}
		}
		toLeft = Vec[0] * Vec1[1] - Vec[1] * Vec1[0] > 0;
	}
	return true;
}



void Pair::calculateDelCost(PointSet* set)
{
	cv::Mat v = cv::Mat(1, 4, CV_64F);//����
	v.at<double>(0, 0) = bestPoint.Point_Vector[0];
	v.at<double>(0, 1) = bestPoint.Point_Vector[1];
	v.at<double>(0, 2) = bestPoint.Point_Vector[2];
	v.at<double>(0, 3) = 1.0;

	cv::Mat Q = set->point.at(v1).error + set->point.at(v2).error;
	cv::Mat Cost = v * Q * v.t();
	delCost = Cost.at<double>(0, 0);//�õ��۵�����
}

void Pair::calculateDelCostUV(PointSet * set)
{
	cv::Mat v = cv::Mat(1, 6, CV_64F);//����
	v.at<double>(0, 0) = bestPoint.Point_Vector[0];
	v.at<double>(0, 1) = bestPoint.Point_Vector[1];
	v.at<double>(0, 2) = bestPoint.Point_Vector[2];
	v.at<double>(0, 3) = bestPoint.PointUV.x;
	v.at<double>(0, 4) = bestPoint.PointUV.y;
	v.at<double>(0, 5) = 1.0;

	cv::Mat Q = set->point.at(v1).Texture_error + set->point.at(v2).Texture_error;
	cv::Mat Cost = v * Q * v.t();
	delCost = Cost.at<double>(0, 0);//�õ��۵�����
}

void Pair::calculateDelCostDQEMUV(PointSet * set)
{
	cv::Mat v = cv::Mat(1, 6, CV_64F);//����
	v.at<double>(0, 0) = bestPoint.Point_Vector[0];
	v.at<double>(0, 1) = bestPoint.Point_Vector[1];
	v.at<double>(0, 2) = bestPoint.Point_Vector[2];
	v.at<double>(0, 3) = bestPoint.PointUV_Vector[0];
	v.at<double>(0, 4) = bestPoint.PointUV_Vector[1];
	v.at<double>(0, 5) = 1.0;

	cv::Mat Q = set->point.at(v1).Texture_error + set->point.at(v2).Texture_error;
	cv::Mat Cost = v * Q * v.t();
	delCost = Cost.at<double>(0, 0);//�õ��۵�����
}

void Pair::calculateDelCostQEM(PointSet * set)
{
	cv::Mat v = cv::Mat(1, 6, CV_64F);//����
	v.at<double>(0, 0) = bestPoint.Point_Vector[0];
	v.at<double>(0, 1) = bestPoint.Point_Vector[1];
	v.at<double>(0, 2) = bestPoint.Point_Vector[2];
	v.at<double>(0, 3) = bestPoint.PointUV_Vector[0];
	v.at<double>(0, 4) = bestPoint.PointUV_Vector[1];
	v.at<double>(0, 5) = 1.0;

	cv::Mat Q = set->point.at(v1).Texture_error + set->point.at(v2).Texture_error;
	cv::Mat Cost = v * Q * v.t();
	delCost = Cost.at<double>(0, 0);//�õ��۵�����
}

void Pair::calculateDelCostQEM1(PointSet * set)
{
	cv::Mat v = cv::Mat(1, 6, CV_64F);//����
	v.at<double>(0, 0) = bestPoint.Point_Vector[0];
	v.at<double>(0, 1) = bestPoint.Point_Vector[1];
	v.at<double>(0, 2) = bestPoint.Point_Vector[2];
	v.at<double>(0, 3) = bestPoint.PointUV.x;
	v.at<double>(0, 4) = bestPoint.PointUV.y;
	v.at<double>(0, 5) = 1.0;

	cv::Mat Q = set->point.at(v1).Texture_error + set->point.at(v2).Texture_error;
	cv::Mat Cost = v * Q * v.t();
	delCost = Cost.at<double>(0, 0);//�õ��۵�����
}

void Pair::calculateDelCostD_QEM(PointSet * Set, int type, double V_sharpness, double S_angleerror, double T_complexity)
{
	switch (type)
	{
	case 1://��������۵�
		calculateDelCostDQEMUV(Set);//����ߴ������������۵�����
		//delCost = delCost * S_angleerror* pow(3, V_sharpness);//�������3
		delCost = delCost * pow(3, V_sharpness) * T_complexity * S_angleerror;//�������3
		break;
	case 2://����۵���λ���Ѷ���
	{
		int p_v1uv = *Set->point.at(strat_Point).SPointUV_index.begin();//���������ʶ
		int uvsize = Set->point.at(end_Point).SPointUV_index.size();//�յ���������

		for (auto uv : Set->point.at(end_Point).SPointUV_index) {
			if (Set->point_uv.at(uv).hasNeighbor(p_v1uv)) {//������������ƥ��
				//�����۵�����
				set<int>::iterator pos1 = find(Set->point.at(end_Point).SPointUV_index.begin(), Set->point.at(end_Point).SPointUV_index.end(), uv);
				int dex = distance(Set->point.at(end_Point).SPointUV_index.begin(), pos1);
				cv::Mat mat1 = Set->point.at(strat_Point).Texture_error;//�õ��������������
				cv::Mat mat2 = Set->point.at(end_Point).Texture_errors[dex];//�õ��յ�����������,*
				//cv::Mat mat2 = Set->point.at(end_Point).Texture_errors[0]+ Set->point.at(end_Point).Texture_errors[1];//�õ��յ�����������

				calculateDelCostUV_Boundary(Set, mat1, mat2); //����߽��(���)�������������۵�����
				//delCost = delCost * S_angleerror* pow(3, V_sharpness);//�������3
				delCost = delCost * pow(3, V_sharpness) * T_complexity * S_angleerror;//�������3
				break;
			}
		}
	}
	break;
	case 3://����۵���λ��δ����
	{   double delCost_v1 = 0;//v1Ϊ���ʱ�����ֵ
	double delCost_v2 = 0;//v2Ϊ���ʱ�����ֵ

	for (auto uv1 : Set->point.at(v1).SPointUV_index) {//����v1����������
		int UV1, UV2;
		set<int>::iterator pos2;
		set<int>::iterator pos3;
		bool special = true;//�������
		pos2 = find(Set->point.at(v1).SPointUV_index.begin(), Set->point.at(v1).SPointUV_index.end(), uv1);
		for (auto uv2 : Set->point.at(v2).SPointUV_index) {//����v2����������
			pos3 = find(Set->point.at(v2).SPointUV_index.begin(), Set->point.at(v2).SPointUV_index.end(), uv2);
			if (Set->point_uv.at(uv1).hasNeighbor(uv2)) {//��Գɹ�
				UV1 = uv1;
				UV2 = uv2;
				special = false;
				break;
			}
		}
		if (special == true) {//���������������ֹ�۵�
			delCost = 1000000000000;
			return;
		}
		else
		{
			int dex1 = distance(Set->point.at(v1).SPointUV_index.begin(), pos2);
			int dex2 = distance(Set->point.at(v2).SPointUV_index.begin(), pos3);
			cv::Mat mat1 = Set->point.at(v1).Texture_errors[dex1];//�õ�v1������������
			cv::Mat mat2 = Set->point.at(v2).Texture_errors[dex2];//�õ�v2������������

			delCost_v1 += calculateDelCostUV_Boundary1(Set, v2, UV2, mat1, mat2);
			delCost_v2 += calculateDelCostUV_Boundary1(Set, v1, UV1, mat1, mat2);
		}
	}

	//ȷ���۵�����
	if (delCost_v1 < delCost_v2) {
		//ȷ���۵�����
		strat_Point = v1;
		end_Point = v2;
	}
	else
	{
		//ȷ���۵�����
		strat_Point = v2;
		end_Point = v1;
	}

	//ȷ���۵����������������
	bestPoint.Point_Vector = Set->point.at(end_Point).Point_Vector;//ȷ���۵����xyz����
	int a = 0;
	for (auto uv3 : Set->point.at(end_Point).SPointUV_index) {//�����յ���������
		a++;
		if (a == 1) {
			bestPoint.PointUV_Vector = Set->point_uv.at(uv3).PointUV_Vector;//��һ�����������
		}
		else
		{
			bestPoint.PointUV_Vector2 = Set->point_uv.at(uv3).PointUV_Vector;//�ڶ�����������
		}
	}

	//�����۵����
	//delCost = (delCost_v1 < delCost_v2) ? delCost_v1 : delCost_v2 * S_angleerror* pow(3, V_sharpness);//�������3
	delCost = (delCost_v1 < delCost_v2) ? delCost_v1 : delCost_v2;//�������3
	delCost = delCost * pow(3, V_sharpness) * T_complexity * S_angleerror;
	//delCost = 1000000000000;
	}
	break;
	case 4://��ֹ�۵�
		delCost = 1000000000000000000;
		break;
	}
}

void Pair::calculateDelCostMSADO(PointSet * Set, int type, bool success, double dc, double de)
{
	switch (type)
	{
	case 1://��������۵�
	case 2://����۵���λ���Ѷ���
	{
		calculateDelCost(Set);//�����������۵�����
		delCost = delCost * de * dc;//�������
		break;
	}
	break;
	case 3://ƽ���۵�
	{
		if (success == true) {
			calculateDelCost(Set);//�����������۵�����
			delCost = delCost * de * dc;//�������
		}
		else//���������������ֹ�۵�
		{
			delCost = 1000000000000;
		}
	}
	break;
	case 4://��ֹ�۵�
		delCost = 1000000000000000000;
		break;
	}
}

void Pair::calculateDelCostUV_Boundary(PointSet * set, cv::Mat mat1, cv::Mat mat2)
{
	cv::Mat v = cv::Mat(1, 6, CV_64F);//����
	v.at<double>(0, 0) = bestPoint.Point_Vector[0];
	v.at<double>(0, 1) = bestPoint.Point_Vector[1];
	v.at<double>(0, 2) = bestPoint.Point_Vector[2];
	v.at<double>(0, 3) = bestPoint.PointUV_Vector[0];
	v.at<double>(0, 4) = bestPoint.PointUV_Vector[1];
	v.at<double>(0, 5) = 1.0;

	cv::Mat Q = mat1 + mat2;
	cv::Mat Cost = v * Q * v.t();
	delCost = Cost.at<double>(0, 0);//�õ��۵�����
	//�޸������
	//delCost = delCost + 50*delCost*Texture_num;//0.2Ϊ�������ɵ���
}

double Pair::calculateDelCostUV_Boundary1(PointSet * set, int xyz_end, int uv_end, cv::Mat mat1, cv::Mat mat2)
{
	cv::Mat v = cv::Mat(1, 6, CV_64F);//����
	v.at<double>(0, 0) = set->point.at(xyz_end).Point_Vector[0];
	v.at<double>(0, 1) = set->point.at(xyz_end).Point_Vector[1];
	v.at<double>(0, 2) = set->point.at(xyz_end).Point_Vector[2];
	v.at<double>(0, 3) = set->point_uv.at(uv_end).PointUV_Vector[0];
	v.at<double>(0, 4) = set->point_uv.at(uv_end).PointUV_Vector[1];
	v.at<double>(0, 5) = 1.0;

	cv::Mat Q = mat1 + mat2;
	cv::Mat Cost = v * Q * v.t();
	double cost = Cost.at<double>(0, 0);//�õ��۵�����
	return cost;
}


void Pair::calculateDelCostUV_twoTotwo(PointSet * set, int TexturePoiny_index, cv::Mat mat1, cv::Mat mat2)
{
	cv::Mat v = cv::Mat(1, 6, CV_64F);//����
	v.at<double>(0, 0) = bestPoint.Point_Vector[0];
	v.at<double>(0, 1) = bestPoint.Point_Vector[1];
	v.at<double>(0, 2) = bestPoint.Point_Vector[2];
	if (TexturePoiny_index == 0) {
		v.at<double>(0, 3) = bestPoint.PointUV_Vector[0];
		v.at<double>(0, 4) = bestPoint.PointUV_Vector[1];
	}
	else
	{
		v.at<double>(0, 3) = bestPoint.PointUV_Vector2[0];
		v.at<double>(0, 4) = bestPoint.PointUV_Vector2[1];
	}
	v.at<double>(0, 5) = 1.0;

	cv::Mat Q = mat1 + mat2;
	cv::Mat Cost = v * Q * v.t();
	delCost = delCost + Cost.at<double>(0, 0);//�õ��۵�����
}

double Pair::calcuateAngleError(PointSet * set, int strat_Point, int end_Point)
{
	int strat_neightorSize = set->point.at(strat_Point).neighbor.size();//��ʼ���һ��������
	int tem_texSize = 1;
	bool tem = false;
	SPoint P;
	for (int i = 0; i < strat_neightorSize; i++) {
		if (set->point.at(set->point.at(strat_Point).neighbor[i]).SPointUV_index.size() > tem_texSize) {
			tem_texSize = set->point.at(set->point.at(strat_Point).neighbor[i]).SPointUV_index.size();
			P = set->point.at(set->point.at(strat_Point).neighbor[i]);
			tem = true;
		}
	}

	cv::Vec3d v0, v1, v2;
	v0 = set->point.at(strat_Point).Point_Vector;//���
	v1 = set->point.at(end_Point).Point_Vector;//�յ�
	v2 = P.Point_Vector;//��ص�

	//����Ƕ����
	double E = 0.0;
	if (tem == true) {
		E = (v0 - v1).dot(v2 - v1) / sqrt((v0 - v1).dot(v0 - v1))* sqrt((v2 - v0).dot(v2 - v0));
	}
	else
	{
		E = 0.0;
	}
	return E;
}

bool Pair::computeRotationMatrix(cv::Vec3d plane[3], osg::Matrix & R, osg::Matrix & R_i)
{
	double directX[3];
	double directZ[3];

	cv::Vec3d vec1 = plane[1] - plane[0];
	cv::Vec3d vec2 = plane[2] - plane[1];
	cv::Vec3d Normal = vec1.cross(vec2);
	cv::Vec3d Unit_Normal = Normal / sqrt(Normal.dot(Normal));//��λ������
	cv::Vec3d Unit_vec1 = vec1 / sqrt(vec1.dot(vec1));//vec1�ĵ�λ����
	//X�S
	directX[0] = Unit_vec1[0];
	directX[1] = Unit_vec1[1];
	directX[2] = Unit_vec1[2];
	//Z�S
	directZ[0] = (double)(Unit_Normal[1] * Unit_vec1[2] - Unit_Normal[2] * Unit_vec1[1]);
	directZ[1] = (double)(Unit_Normal[2] * Unit_vec1[0] - Unit_Normal[0] * Unit_vec1[2]);
	directZ[2] = (double)(Unit_Normal[0] * Unit_vec1[1] - Unit_Normal[1] * Unit_vec1[0]);

	/*��ת����	*/
	R.set(directX[0], Unit_Normal[0], directZ[0], 0,
		directX[1], Unit_Normal[1], directZ[1], 0,
		directX[2], Unit_Normal[2], directZ[2], 0,
		0, 0, 0, 1);
	R_i = osg::Matrixd::inverse(R);

	return true;
}

osg::Vec3d Pair::getXYZ(cv::Vec3d P, osg::Matrix R)
{
	osg::Vec3d tem_P;
	tem_P[0] = P[0];
	tem_P[1] = P[1];
	tem_P[2] = P[2];
	osg::Vec3d rotation_P = R.preMult(tem_P);
	return rotation_P;
}

bool Pair::affineTransformMatrix(osg::Vec3d plane[3], cv::Point2d uv[3], osg::Vec3d P, cv::Point2d & UV)
{
	// Դ����(xy)��Ŀ�����꣨uv��
	cv::Point2f srcTri[3];
	cv::Point2f dstTri[3];
	dstTri[0] = uv[0];
	dstTri[1] = uv[1];
	dstTri[2] = uv[2];
	srcTri[0] = cv::Point2d(plane[0][0], plane[0][2]);
	srcTri[1] = cv::Point2d(plane[1][0], plane[1][2]);
	srcTri[2] = cv::Point2d(plane[2][0], plane[2][2]);
	// ��÷���任����
	cv::Mat warp_mat(2, 3, CV_64F);
	warp_mat = getAffineTransform(srcTri, dstTri);
	float affMatrix[6];
	affMatrix[0] = warp_mat.at<double>(0, 0);
	affMatrix[1] = warp_mat.at<double>(0, 1);
	affMatrix[2] = warp_mat.at<double>(0, 2);
	affMatrix[3] = warp_mat.at<double>(1, 0);
	affMatrix[4] = warp_mat.at<double>(1, 1);
	affMatrix[5] = warp_mat.at<double>(1, 2);
	//���з���任
	UV.x = affMatrix[0] * P[0] + affMatrix[1] * P[2] + affMatrix[2];
	UV.y = affMatrix[3] * P[0] + affMatrix[4] * P[2] + affMatrix[5];
	return true;
}


//bool Pair::isFeaturePair(PointSet* set)
//{
//	int count = 0;
//	for (int i = 0; i < (int)set->point[v1].neighbor.size(); ++i)
//		if (set->point[v2].hasNeighbor(set->point[v1].neighbor[i]))
//			count++;
//	if (count != 2)
//		return true;
//	else return false;
//}

//Pair Pair::operator=(Pair& p)
//{
//	v1 = p.v1;
//	v2 = p.v2;
//	bestPoint = p.bestPoint;
//	delCost = p.delCost;
//	return *this;
//}

