#include "PairHeap.h"
#include <map>
#include <iostream>
#include <fstream>
#include "CU.h"

using namespace std;
PairHeap::PairHeap()
{
	count = 0;
}

void PairHeap::setupHeap(PointSet* set, double Max_Distance)
{
	vector<pair<int, int>> boundary;//�߽��߶μ���

	for (int i = 0; i < set->count; ++i) {//��ĸ���
		for (int j = 0; j < (int)set->point.at(i).neighbor.size(); ++j)//����ÿһ�����ȫ��һ�������
		{
			int k = set->point.at(i).neighbor[j];//������ʶ
			if (i < k)//�����ظ�
			{
				Pair toAdd(i, k);//���һ��
				bool Success;
				if (toAdd.JudgeBoundary(set) == true) {
					boundary.push_back(make_pair(i, k));
					set->point.at(i).point_changeable = false;//p1������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
					set->point.at(k).point_changeable = false;//p2������Ϊ�����۵�
				}//�ж��Ƿ�Ϊ�߽��߶�
				else
				{

				}

				toAdd.calculateBestPoint(set, Success);//�����۵��������λ�ã��н���㣬û���ȡ�е㣩
				if (Success == true) {
					//toAdd.calculateBestUV(set);//�����۵�������������UV
				}
				toAdd.calculateDelCost(set);//����ߵ��۵�����

				double Sharpness1 = 0;
				double Sharpness2 = 0;

				//if (toAdd.calculateVertexSharpness(set, toAdd.v1, Sharpness1)) {//���㶥�����ȣ����Ƴ������쳣�����۵�
				//	Sharpness1 = 3;
				//}
				//if (toAdd.calculateVertexSharpness(set, toAdd.v2, Sharpness2)) {
				//	Sharpness2 = 3;
				//}


				double Sharpness = Sharpness1 + Sharpness2;
				//double Sharpness = toAdd.calculateVertexSharpness(set, toAdd.v1) + toAdd.calculateVertexSharpness(set, toAdd.v2);//���㶥��ļ���ȣ��쳣������
				//toAdd.delCost = toAdd.delCostSharpness;
				//toAdd.delCost = toAdd.delCost*pow(3, Sharpness);
				//toAdd.delCost = toAdd.delCost + Sharpness;
				//if (toAdd.ChangeTopology(set) == true) {
				//	toAdd.delCost = toAdd.delCost + 10000;
				//}//�ж������쳣

				if (toAdd.calculateMaxNormalDeviation(set) >= 1) {
					//set->point.at(i).pointPair_changeable = false;//p1������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
					//set->point.at(k).pointPair_changeable = false;//p2������Ϊ�����۵�
					//toAdd.delCost = toAdd.delCost + 10000000;
				}//���������ƫת
				addPair(&toAdd);
			}
		}
	}//�������������˽ṹѡ�߶�

	//double Max_D = 0.000006;//����
	//for (int i = 0; i < set->count; ++i) {//��ĸ���
	//	for (int j = i + 1; j < set->count; ++j) {//����ÿһ�����ȫ��һ�������
	//		cv::Vec3d tem = set->point.at(i).Point_Vector - set->point.at(j).Point_Vector; 
	//		if (sqrt(tem.dot(tem))< Max_D) {
	//			//��������������
	//			Pair toAdd(i, j);//���һ��
	//			toAdd.calculateBestPoint(set);//�����۵��������λ�ã��н���㣬û���ȡ�е㣩
	//			toAdd.calculateDelCost(set);//����ߵ��۵�����
	//			addPair(&toAdd);
	//		}
	//		/*if (sqrt(tem.dot(tem)) == 0) {
	//			int pppp = 99;
	//		}*/
	//		//if (D< Max_Distance) {
	//		//	Pair toAdd(i, j);//���һ��
	//		//	toAdd.calculateBestPoint(set);//�����۵��������λ�ã��н���㣬û���ȡ�е㣩
	//		//	toAdd.calculateDelCost(set);//����ߵ��۵�����
	//		//	addPair(&toAdd);
	//		//}
	//	}
	//}//�������ڵ�ľ���ѡ�߶�


	//for (auto iter = boundary.cbegin(); iter != boundary.cend(); iter++)
	//{
	//	int p1 = iter->first;
	//	int p2 = iter->second;
	//	mapper[make_pair(p1, p2)] = false;
	//	set->point.at(p1).point_changeable = false;//p1������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
	//	set->point.at(p2).point_changeable = false;//p2������Ϊ�����۵�

	//	for (int i = 0; i < (int)set->point.at(p1).neighbor.size(); ++i) {
	//		int j = set->point.at(p1).neighbor[i];//������ʶ
	//		if (j < p1) {
	//			mapper[make_pair(j, p1)] = false;
	//		}
	//		else
	//		{
	//			mapper[make_pair(p1, j)] = false;
	//		}
	//	}//����p1���ȫ��һ�������
	//	for (int i = 0; i < (int)set->point.at(p2).neighbor.size(); ++i) {
	//		int j = set->point.at(p2).neighbor[i];//������ʶ
	//		if (j < p2) {
	//			mapper[make_pair(j, p2)] = false;
	//		}
	//		else
	//		{
	//			mapper[make_pair(p2, j)] = false;
	//		}
	//	}//����p2���ȫ��һ�������
	//}//�������б߽��߶εĵ㣨p1p2����������p1p2��һ���������ɵıߣ����ñ�Ϊ�����۵�
	//
}

void PairHeap::setupHeapUV(PointSet * Set)
{
	//vector<pair<int, int>> boundary;//�߽��߶μ���

	for (int i = 0; i < Set->count; ++i) {//��ĸ���
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//����ÿһ�����ȫ��һ�������
		{
			int k = Set->point.at(i).neighbor[j];//������ʶ
			if (i < k)//�����ظ�
			{
				bool test = false;
				Pair toAdd(i, k);//���һ��
				if (toAdd.JudgeBoundary(Set) == true) {
					//boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
					Set->point.at(k).point_changeable = false;//p2������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
				}//�ж��Ƿ�Ϊ�߽��߶�

				int p_v1, p_v2;
				if (toAdd.JudgeUVBoundary(Set, p_v1, p_v2) == true) {
					if (p_v1 != -1 && p_v2 != -1) {//�������������
						bool Success;
						toAdd.calculateBestPointUV(Set, Success);//�����۵��������λ�ú�UV���н���㣬û���ȡ�е㣩
						toAdd.calculateDelCostUV(Set);//����ߵ��۵�����
						test = true;
					}
					else//����۵�
					{
						int startP, endP;
						if (p_v1 != -1) {//p_v1->p_v2
							startP = p_v1;
							endP = toAdd.v2;
						}
						else//p_v2->p_v1
						{
							startP = p_v2;
							endP = toAdd.v1;
						}

						toAdd.bestPoint.Point_Vector = Set->point.at(endP).Point_Vector;//xyz����

						int p_v1uv = *Set->point.at(startP).SPointUV_index.begin();
						int uvsize = Set->point.at(endP).SPointUV_index.size();//��ֹ�����������
						for (auto uv : Set->point.at(endP).SPointUV_index) {
							toAdd.bestPoint.SPointUV_index.insert(uv);//bestPoint�������ԭʼuv���ʶ
							if (Set->point_uv.at(uv).hasNeighbor(p_v1uv)) {//�ҵ�������������
								toAdd.bestPoint.SPointUV_index.erase(uv);//bestPointɾ�����ڵ�uv���ʶ
								toAdd.bestPoint.PointUV_Vector = Set->point_uv.at(uv).PointUV_Vector;//��������
								//�����۵�����
								set<int>::iterator pos1 = find(Set->point.at(endP).SPointUV_index.begin(), Set->point.at(endP).SPointUV_index.end(), uv);
								int dex = distance(Set->point.at(endP).SPointUV_index.begin(), pos1);
								cv::Mat mat1 = Set->point.at(startP).Texture_error;//�õ���ʼ������������
								cv::Mat mat2 = Set->point.at(endP).Texture_errors[dex];//�õ���ֹ������������

								toAdd.calculateDelCostUV_Boundary(Set, mat1, mat2); //��������ߵ��۵�����
								test = true;
							}
						}
					}

				}
				else//�����۵�
				{
					toAdd.delCost = 10000000;
					test = true;
				}

				////����߶Ե�һ�����������Ӷ�
				//double TextrueComplexity = toAdd.calculate_TexComplexity(Set);
				//toAdd.delCost = toAdd.delCost * 100000 * (TextrueComplexity - 0.99);

				//����߶ԵĶ�������
				double Sharpness1 = 0;
				double Sharpness2 = 0;
				toAdd.calculateVertexSharpness(Set, toAdd.v1, Sharpness1);
				toAdd.calculateVertexSharpness(Set, toAdd.v2, Sharpness2);
				double Sharpness = Sharpness1 + Sharpness2;
				toAdd.delCost = toAdd.delCost*pow(3, Sharpness);

				if (test == false) {
					int  uuuuu = 6;
				}

				if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
					//set->point.at(i).pointPair_changeable = false;//p1������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
					//set->point.at(k).pointPair_changeable = false;//p2������Ϊ�����۵�
					toAdd.delCost = toAdd.delCost + 10000000;
				}//���������ƫת
				addPair(&toAdd);
			}
		}
	}//�������������˽ṹѡ�߶�

}

void PairHeap::setupHeapUV_maxSim(PointSet * Set)
{
	vector<pair<int, int>> boundary;//�߽��߶μ���

	for (int i = 0; i < Set->count; ++i) {//��ĸ���
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//����ÿһ�����ȫ��һ�������
		{
			int k = Set->point.at(i).neighbor[j];//������ʶ
			if (i < k)//�����ظ�
			{
				/*if (i == 168 && k == 169) {
					int g = 6;
				}*/
				Pair toAdd(i, k);//���һ��
				if (toAdd.JudgeBoundary(Set) == true) {
					boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
					Set->point.at(k).point_changeable = false;//p2������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
				}//�ж��Ƿ�Ϊģ�ͱ߽��߶�

				//����߶Ե�һ�����������Ӷ�
				double TextrueComplexity = toAdd.calculate_TexComplexity(Set);

				//�������ۣ�������ѵ㼰�۵�����
				int p_v1, p_v2;
				switch (toAdd.JudgeTex(Set, p_v1, p_v2))
				{
				case 1://p1,p2������һ�����ǵ������
					if (HeapClassify_No1(Set, toAdd, p_v1, p_v2, TextrueComplexity) == false) {
						int  oooo = 6;
					}
					break;
				case 2://p1,p2��Ϊ������㣨�����۵���
					HeapClassify_No2(Set, toAdd);
					break;
				case 3://��������Ҫ����2-2
					if (HeapClassify_No3(Set, toAdd, p_v1, p_v2, TextrueComplexity) == false) {
						int ooooo = 6;
					}

					break;

				default:
					break;
				}


				if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
					//set->point.at(i).pointPair_changeable = false;//p1������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
					//set->point.at(k).pointPair_changeable = false;//p2������Ϊ�����۵�
					double ppp = toAdd.delCost;
					toAdd.delCost = toAdd.delCost + 10000000;
				}//���������ƫת
				//if (toAdd.ChangeTopology(Set) == true) {
				//	toAdd.delCost = toAdd.delCost + 10000000;
				//}//�ж������Ƿ�ı�
				addPair(&toAdd);
			}
		}
	}//�������������˽ṹѡ�߶�
}

void PairHeap::setupHeapQEM(PointSet * Set)
{
	for (int i = 0; i < Set->count; ++i) {//��ĸ���
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//����ÿһ�����ȫ��һ�������
		{
			int k = Set->point.at(i).neighbor[j];//������ʶ
			if (i < k)//�����ظ�
			{
				Pair toAdd(i, k);//���һ��
				if (toAdd.JudgeBoundary(Set) == true) {
					//boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
					Set->point.at(k).point_changeable = false;//p2������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
				}//�ж��Ƿ�Ϊ�߽��߶�/������ѹ�����

				bool Success;
				toAdd.calculateBestPointQEM(Set, Success);//�����۵��������λ�ú�UV���н���㣬û���ȡ�е㣩
				/*if (Success==false) {
					int ss = 6;
				}*/
				toAdd.calculateDelCostQEM(Set);//����ߵ��۵�����

				////����1�����㶥������
				//double Sharpness1 = 0;
				//double Sharpness2 = 0;
				//toAdd.calculateVertexSharpness(Set, toAdd.v1, Sharpness1);
				//toAdd.calculateVertexSharpness(Set, toAdd.v2, Sharpness2);
				//double Vertex_sharpness = Sharpness1 + Sharpness2;
				//Vertex_sharpness = Vertex_sharpness / 3.14;
				//
				////����2
				//double Texture_complexity = toAdd.calculate_TexComplexity(Set);
				//toAdd.delCost = toAdd.delCost * Vertex_sharpness * Texture_complexity;

				if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
					double ppp = toAdd.delCost;
					toAdd.delCost = toAdd.delCost + 10000000;
				}//���������ƫת

				addPair(&toAdd);
			}
		}
	}
}

void PairHeap::setupHeapQEM1(PointSet * Set)
{
	for (int i = 0; i < Set->count; ++i) {//��ĸ���
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//����ÿһ�����ȫ��һ�������
		{
			int k = Set->point.at(i).neighbor[j];//������ʶ
			if (i < k)//�����ظ�
			{
				Pair toAdd(i, k);//���һ��
				if (toAdd.JudgeBoundary(Set) == true) {
					//boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
					Set->point.at(k).point_changeable = false;//p2������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
				}//�ж��Ƿ�Ϊ�߽��߶�/������ѹ�����

				bool Success;
				toAdd.calculateBestPointQEM1(Set, Success);//�����۵��������λ�ú�UV���н���㣬û���ȡ�е㣩
				/*if (Success==false) {
					int ss = 6;
				}*/
				toAdd.calculateDelCostQEM1(Set);//����ߵ��۵�����

				//if (toAdd.JudgeAbnormalCollpase(Set) == true) {
				//	toAdd.delCost = toAdd.delCost + 1000000000000000000;
				//}//�쳣ֵ����

				//if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
				//	double ppp = toAdd.delCost;
				//	toAdd.delCost = toAdd.delCost + 10000000000;
				//}//���������ƫת

				addPair(&toAdd);
			}
		}
	}
}

void PairHeap::setupHeapD_QEM(PointSet * Set)
{
	int s1 = 0;
	int s2 = 0;

	int edge = 0;
	for (int i = 0; i < Set->count; ++i) {//��ĸ���
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//����ÿһ�����ȫ��һ�������
		{
			int k = Set->point.at(i).neighbor[j];//������ʶ
			if (i < k)//�����ظ�
			{
				edge++;
				Pair toAdd(i, k);//���һ��
				if (toAdd.JudgeBoundary(Set) == true) {
					//boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
					Set->point.at(k).point_changeable = false;//p2������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
				}//�ж��Ƿ�Ϊ�߽��߶�/������ѹ�����

				//����1�����㶥������
				double Sharpness1 = 0;
				double Sharpness2 = 0;
				toAdd.calculateVertexSharpness(Set, toAdd.v1, Sharpness1);
				toAdd.calculateVertexSharpness(Set, toAdd.v2, Sharpness2);
				double Vertex_sharpness = Sharpness1 + Sharpness2;
				//����2������ӷ�Ƕ����
				/*double Seam1 = 0;
				double Seam2 = 0;
				toAdd.calculateSeamError(Set, toAdd.v1, Seam1);
				toAdd.calculateSeamError(Set, toAdd.v2, Seam2);
				double Seam_angleerror = (Seam1 + Seam2) / 2;*/
				double Seam_angleerror = 1;
				//����3�����������Ӷ�
				//double Texture_complexity = toAdd.calculate_TexComplexity(Set);
				double Texture_complexity = 1;

				//�������ۣ���������۵���
				int T_edge = toAdd.JudgeStrategy(Set);//�жϱ����
				bool Success;
				toAdd.calculateBestPointD_QEM(Set, Success, T_edge);//ȷ������ߵ�����۵�λ��
				/*if (Success==false) {
					int ss = 6;
				}*/
				toAdd.calculateDelCostD_QEM(Set, T_edge, Vertex_sharpness, Seam_angleerror, Texture_complexity);//����ߵ��۵�����
				if (toAdd.delCost == 1000000000000) {
					s1++;
				}
				if (toAdd.delCost == 1000000000000000000) {
					s2++;
				}

				if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
					double ppp = toAdd.delCost;
					toAdd.delCost = toAdd.delCost + 1000000000000000000;
				}//���������ƫת,�����۵�

				addPair(&toAdd);
			}
		}
	}
	int uu = 6;
}

void PairHeap::setupHeapMSADO(PointSet * Set, CU cu)
{
	int s1 = 0;
	int s2 = 0;

	int edge = 0;
	for (int i = 0; i < Set->count; ++i) {//��ĸ���
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//����ÿһ�����ȫ��һ�������
		{
			int k = Set->point.at(i).neighbor[j];//������ʶ
			if (i < k)//�����ظ�
			{
				edge++;
				Pair toAdd(i, k);//���һ��
				if (toAdd.JudgeBoundary(Set) == true) {
					//boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
					Set->point.at(k).point_changeable = false;//p2������Ϊ�����۵�(�߽��ߵĵ�����Ϊ�����۵�)
				}//�ж��Ƿ�Ϊ�߽��߶�/������ѹ�����

				//����������dc,de
				float dc, dc1, dc2 = 0;
				float de, de1, de2 = 0;
				/*cu.calculation(toAdd.v1, Set, dc1, de1);
				cu.calculation(toAdd.v2, Set, dc2, de2);
				dc = 0.5*(dc1 + dc2);
				de = 0.5*(de1 + de2);*/
				dc = 1;
				de = 1;

				//�������ۣ���������۵���
				int T_edge = toAdd.JudgeStrategy_MSADO(Set);//�жϱ����
				bool Success;
				toAdd.calculateBestPointMSADO(Set, Success, T_edge);//ȷ������ߵ�����۵�λ��
				toAdd.calculateDelCostMSADO(Set, T_edge, Success, dc, de);//����ߵ��۵�����

				if (toAdd.delCost == 1000000000000) {
					s1++;
				}
				if (toAdd.delCost == 1000000000000000000) {
					s2++;
				}

				if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
					double ppp = toAdd.delCost;
					toAdd.delCost = toAdd.delCost + 1000000000000000000;
				}//���������ƫת,�����۵�

				addPair(&toAdd);
			}
		}
	}
	int uu = 6;
}

//p1,p2��������һ���ǵ������
bool PairHeap::HeapClassify_No1(PointSet* Set, Pair &toAdd, int V1, int V2, double Tex_Complexity)
{
	if (V1 == 1 && V2 == 1) {//�������������
		bool Success;
		toAdd.calculateBestPointUV(Set, Success);//�����۵��������λ�ú�UV���н���㣬û���ȡ�е㣩
		toAdd.calculateDelCostUV(Set);//����ߵĻ�����������۵�����

		toAdd.delCost = toAdd.delCost * 1000 * (Tex_Complexity - 0.99);
		return true;
	}
	else//����۵�
	{
		int startP, endP;
		if (V1 == 1) {//v1->v2
			startP = toAdd.v1;
			endP = toAdd.v2;
		}
		else//v2->v1
		{
			startP = toAdd.v2;
			endP = toAdd.v1;
		}

		toAdd.bestPoint.Point_Vector = Set->point.at(endP).Point_Vector;//xyz����

		int p_v1uv = *Set->point.at(startP).SPointUV_index.begin();
		int uvsize = Set->point.at(endP).SPointUV_index.size();//��ֹ�����������
		for (auto uv : Set->point.at(endP).SPointUV_index) {
			toAdd.bestPoint.SPointUV_index.insert(uv);//bestPoint�������ԭʼuv���ʶ
			if (Set->point_uv.at(uv).hasNeighbor(p_v1uv)) {//�ҵ�������������
				toAdd.bestPoint.SPointUV_index.erase(uv);//bestPointɾ�����ڵ�uv���ʶ
				toAdd.bestPoint.PointUV_Vector = Set->point_uv.at(uv).PointUV_Vector;//��������
				//�����۵�����
				set<int>::iterator pos1 = find(Set->point.at(endP).SPointUV_index.begin(), Set->point.at(endP).SPointUV_index.end(), uv);
				int dex = distance(Set->point.at(endP).SPointUV_index.begin(), pos1);
				cv::Mat mat1 = Set->point.at(startP).Texture_error;//�õ���ʼ������������
				cv::Mat mat2 = Set->point.at(endP).Texture_errors[dex];//�õ���ֹ������������

				toAdd.calculateDelCostUV_Boundary(Set, mat1, mat2); //����ߵĻ�����������۵�����
				//toAdd.delCost = toAdd.delCost * Tex_Complexity;//����ȹ�ʽ
				toAdd.delCost = toAdd.delCost * 1000 * (Tex_Complexity - 0.99);//����ȹ�ʽ
				return true;
			}
		}
	}

	return false;
}

//p1,p2��Ϊ�������
bool PairHeap::HeapClassify_No2(PointSet * set, Pair &toAdd)
{
	toAdd.delCost = 10000000;
	return true;
}

bool PairHeap::HeapClassify_No3(PointSet * Set, Pair &toAdd, int V1, int V2, double Tex_Complexity)
{
	bool Foldable = true;//�Ƿ���۵�
	//ɸѡ�������
	int v1_neighborSize = Set->point.at(toAdd.v1).neighbor.size();//v1���ڵ����
	int v2_neighborSize = Set->point.at(toAdd.v2).neighbor.size();//v2���ڵ����
	//for (int i = 0; i < v1_neighborSize; ++i) {
	//	if (Set->point.at(Set->point.at(toAdd.v1).neighbor[i]).hasNeighbor(toAdd.v2)) {
	//		SPoint P = Set->point.at(Set->point.at(toAdd.v1).neighbor[i]);
	//		if (P.SPointUV_index.size() != 1) {
	//			Foldable = false;//�����۵�
	//		}
	//	}
	//}

	int jl1 = 0;
	int jl2 = 0;
	for (int i = 0; i < v1_neighborSize; ++i) {
		int n1 = Set->point.at(Set->point.at(toAdd.v1).neighbor[i]).SPointUV_index.size();//��������
		if (n1 == 2) {
			jl1++;
		}
	}
	for (int i = 0; i < v2_neighborSize; ++i) {
		int n2 = Set->point.at(Set->point.at(toAdd.v2).neighbor[i]).SPointUV_index.size();//��������
		if (n2 == 2) {
			jl2++;
		}
	}
	if (jl1 == 2 && jl2 == 2) {
		Foldable = true;//���۵�
	}
	else
	{
		Foldable = false;//�����۵�
	}

	int strat, end;//�ж���ʼ���ָ���
	if (Foldable == true) {//���۵�
		if (V1 == 2 && V2 == 2) {
			if (toAdd.calcuateAngleError(Set, toAdd.v1, toAdd.v2) < toAdd.calcuateAngleError(Set, toAdd.v2, toAdd.v1)) {//����Ƕ����
				strat = toAdd.v1;
				end = toAdd.v2;
			}
			else
			{
				strat = toAdd.v2;
				end = toAdd.v1;
			}
			//��ʼ���ָ�����
			toAdd.strat_Point = strat;
			toAdd.end_Point = end;
			//����۵�������ۣ�2-2��
			toAdd.bestPoint.Point_Vector = Set->point.at(end).Point_Vector;//xyz����

			int stratP_uvsize = Set->point.at(strat).SPointUV_index.size();//��ʼ�����������=2
			int endP_uvsize = Set->point.at(end).SPointUV_index.size();//��ֹ�����������=2
			int strat_uv_index = 0;
			int end_uv_index = 0;
			cv::Mat mat1, mat2;
			int judge1 = 0;
			for (auto strat_uv : Set->point.at(strat).SPointUV_index) {//������ʼ��������
				for (auto end_uv : Set->point.at(end).SPointUV_index) {//������ֹ��������
					if (Set->point_uv.at(strat_uv).hasNeighbor(end_uv)) {//�ҵ�������������
						judge1++;
						if (strat_uv_index == 0) {
							toAdd.bestPoint.PointUV_Vector = Set->point_uv.at(end_uv).PointUV_Vector;//��������1
							mat1 = Set->point.at(strat).Texture_errors[0];//�õ���ʼ������������
							if (end_uv_index == 0) {
								mat2 = Set->point.at(end).Texture_errors[0];//�õ���ֹ������������
							}
							else
							{
								mat2 = Set->point.at(end).Texture_errors[1];//�õ���ֹ������������
							}
							toAdd.calculateDelCostUV_twoTotwo(Set, strat_uv_index, mat1, mat2);//���������ѷ��۵�����
						}
						else
						{
							toAdd.bestPoint.PointUV_Vector2 = Set->point_uv.at(end_uv).PointUV_Vector;//��������2

							mat1 = Set->point.at(strat).Texture_errors[1];//�õ���ʼ������������
							if (end_uv_index == 0) {
								mat2 = Set->point.at(end).Texture_errors[0];//�õ���ֹ������������
							}
							else
							{
								mat2 = Set->point.at(end).Texture_errors[1];//�õ���ֹ������������
							}
							toAdd.calculateDelCostUV_twoTotwo(Set, strat_uv_index, mat1, mat2);//���������ѷ��۵�����
						}

					}
					end_uv_index++;
				}
				strat_uv_index++;
			}
			double EAngle = toAdd.calcuateAngleError(Set, strat, end);//�Ƕ����
			toAdd.delCost = (toAdd.delCost + EAngle) * Tex_Complexity;//����ȹ�ʽ
			if (judge1 == 2) {
				return true;
			}
			else
			{
				toAdd.delCost = 10000000;
			}
		}
		else
		{
			if (V1 == 2) {
				strat = toAdd.v1;
				end = toAdd.v2;
			}
			else
			{
				strat = toAdd.v2;
				end = toAdd.v1;
			}
			//�����۵����ۣ�2-�ࣩ
			toAdd.delCost = 10000000;
			return true;
		}
	}
	else//�����۵�
	{
		toAdd.delCost = 10000000;
		return true;
	}

	//toAdd.delCost = 10000000;
	return false;
}


void PairHeap::addPair(Pair* p)
{
	count++;
	pairQueue.push(*p);//�������ȶ���
	mapper.insert(map<pair<int, int>, bool> ::value_type(make_pair(p->v1, p->v2), true));//����map
	//if (p->changeable == true) {
	//	mapper.insert(map<pair<int, int>, bool> ::value_type(make_pair(p->v1, p->v2), true));//����map
	//}
	//else
	//{
	//	mapper.insert(map<pair<int, int>, bool> ::value_type(make_pair(p->v1, p->v2), false));//����map
	//}
	//
	//mapper[make_pair(p->v1, p->v2)] = true;//����map
}

void PairHeap::deletePair(Pair* p)
{
	count--;
	mapper[make_pair(p->v1, p->v2)] = false;
}

void PairHeap::skipPair(Pair * p)
{
	count--;
	mapper[make_pair(p->v1, p->v2)] = false;
}

Pair PairHeap::top(PointSet* set)
{
	while (!pairQueue.empty())
	{
		Pair toret = pairQueue.top();//��ȡ���ȶ��е�һ��Ԫ��
		pairQueue.pop();//ɾ�����ȶ��е�һ��Ԫ��
		if (mapper[make_pair(toret.v1, toret.v2)] == true)
			if (set->point.at(toret.v1).point_changeable == true && set->point.at(toret.v2).point_changeable == true)
				return toret;
	}
	Pair empty(-1, -1);
	return empty;
}

void PairHeap::updata()
{
	count = 0;//�߶���Ŀ����Ϊ0
	while (!pairQueue.empty()) pairQueue.pop();//���priority_queue<Pair, vector<Pair>, cmp> pairQueue;�߶Զ��У����ȶ��У�
	mapper.erase(mapper.begin(), mapper.end());//���map<pair<int, int>, bool> mapper;�߶ԣ�true/false��
}

bool PairHeap::cmp::operator()(Pair& p1, Pair& p2)
{
	return p1.delCost > p2.delCost;
}

