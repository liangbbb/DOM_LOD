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
	vector<pair<int, int>> boundary;//边界线段集合

	for (int i = 0; i < set->count; ++i) {//点的个数
		for (int j = 0; j < (int)set->point.at(i).neighbor.size(); ++j)//遍历每一个点的全部一环领域点
		{
			int k = set->point.at(i).neighbor[j];//邻域点标识
			if (i < k)//避免重复
			{
				Pair toAdd(i, k);//添加一对
				bool Success;
				if (toAdd.JudgeBoundary(set) == true) {
					boundary.push_back(make_pair(i, k));
					set->point.at(i).point_changeable = false;//p1点设置为不可折叠(边界线的点设置为不可折叠)
					set->point.at(k).point_changeable = false;//p2点设置为不可折叠
				}//判断是否为边界线段
				else
				{

				}

				toAdd.calculateBestPoint(set, Success);//计算折叠后点的最佳位置（有解就算，没解就取中点）
				if (Success == true) {
					//toAdd.calculateBestUV(set);//计算折叠后点的最佳纹理点UV
				}
				toAdd.calculateDelCost(set);//计算边的折叠代价

				double Sharpness1 = 0;
				double Sharpness2 = 0;

				//if (toAdd.calculateVertexSharpness(set, toAdd.v1, Sharpness1)) {//计算顶点尖锐度，且推迟拓扑异常处的折叠
				//	Sharpness1 = 3;
				//}
				//if (toAdd.calculateVertexSharpness(set, toAdd.v2, Sharpness2)) {
				//	Sharpness2 = 3;
				//}


				double Sharpness = Sharpness1 + Sharpness2;
				//double Sharpness = toAdd.calculateVertexSharpness(set, toAdd.v1) + toAdd.calculateVertexSharpness(set, toAdd.v2);//计算顶点的尖锐度，异常处处理
				//toAdd.delCost = toAdd.delCostSharpness;
				//toAdd.delCost = toAdd.delCost*pow(3, Sharpness);
				//toAdd.delCost = toAdd.delCost + Sharpness;
				//if (toAdd.ChangeTopology(set) == true) {
				//	toAdd.delCost = toAdd.delCost + 10000;
				//}//判断拓扑异常

				if (toAdd.calculateMaxNormalDeviation(set) >= 1) {
					//set->point.at(i).pointPair_changeable = false;//p1点设置为不可折叠(边界线的点设置为不可折叠)
					//set->point.at(k).pointPair_changeable = false;//p2点设置为不可折叠
					//toAdd.delCost = toAdd.delCost + 10000000;
				}//计算最大法线偏转
				addPair(&toAdd);
			}
		}
	}//按照三角网拓扑结构选边对

	//double Max_D = 0.000006;//距离
	//for (int i = 0; i < set->count; ++i) {//点的个数
	//	for (int j = i + 1; j < set->count; ++j) {//遍历每一个点的全部一环领域点
	//		cv::Vec3d tem = set->point.at(i).Point_Vector - set->point.at(j).Point_Vector; 
	//		if (sqrt(tem.dot(tem))< Max_D) {
	//			//将这两个点连接
	//			Pair toAdd(i, j);//添加一对
	//			toAdd.calculateBestPoint(set);//计算折叠后点的最佳位置（有解就算，没解就取中点）
	//			toAdd.calculateDelCost(set);//计算边的折叠代价
	//			addPair(&toAdd);
	//		}
	//		/*if (sqrt(tem.dot(tem)) == 0) {
	//			int pppp = 99;
	//		}*/
	//		//if (D< Max_Distance) {
	//		//	Pair toAdd(i, j);//添加一对
	//		//	toAdd.calculateBestPoint(set);//计算折叠后点的最佳位置（有解就算，没解就取中点）
	//		//	toAdd.calculateDelCost(set);//计算边的折叠代价
	//		//	addPair(&toAdd);
	//		//}
	//	}
	//}//按照相邻点的距离选边对


	//for (auto iter = boundary.cbegin(); iter != boundary.cend(); iter++)
	//{
	//	int p1 = iter->first;
	//	int p2 = iter->second;
	//	mapper[make_pair(p1, p2)] = false;
	//	set->point.at(p1).point_changeable = false;//p1点设置为不可折叠(边界线的点设置为不可折叠)
	//	set->point.at(p2).point_changeable = false;//p2点设置为不可折叠

	//	for (int i = 0; i < (int)set->point.at(p1).neighbor.size(); ++i) {
	//		int j = set->point.at(p1).neighbor[i];//领域点标识
	//		if (j < p1) {
	//			mapper[make_pair(j, p1)] = false;
	//		}
	//		else
	//		{
	//			mapper[make_pair(p1, j)] = false;
	//		}
	//	}//遍历p1点的全部一环领域点
	//	for (int i = 0; i < (int)set->point.at(p2).neighbor.size(); ++i) {
	//		int j = set->point.at(p2).neighbor[i];//领域点标识
	//		if (j < p2) {
	//			mapper[make_pair(j, p2)] = false;
	//		}
	//		else
	//		{
	//			mapper[make_pair(p2, j)] = false;
	//		}
	//	}//遍历p2点的全部一环领域点
	//}//遍历所有边界线段的点（p1p2），搜索（p1p2）一环邻域点组成的边，设置边为不可折叠
	//
}

void PairHeap::setupHeapUV(PointSet * Set)
{
	//vector<pair<int, int>> boundary;//边界线段集合

	for (int i = 0; i < Set->count; ++i) {//点的个数
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//遍历每一个点的全部一环领域点
		{
			int k = Set->point.at(i).neighbor[j];//邻域点标识
			if (i < k)//避免重复
			{
				bool test = false;
				Pair toAdd(i, k);//添加一对
				if (toAdd.JudgeBoundary(Set) == true) {
					//boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1点设置为不可折叠(边界线的点设置为不可折叠)
					Set->point.at(k).point_changeable = false;//p2点设置为不可折叠(边界线的点设置为不可折叠)
				}//判断是否为边界线段

				int p_v1, p_v2;
				if (toAdd.JudgeUVBoundary(Set, p_v1, p_v2) == true) {
					if (p_v1 != -1 && p_v2 != -1) {//带纹理二次误差简化
						bool Success;
						toAdd.calculateBestPointUV(Set, Success);//计算折叠后点的最佳位置和UV（有解就算，没解就取中点）
						toAdd.calculateDelCostUV(Set);//计算边的折叠代价
						test = true;
					}
					else//半边折叠
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

						toAdd.bestPoint.Point_Vector = Set->point.at(endP).Point_Vector;//xyz坐标

						int p_v1uv = *Set->point.at(startP).SPointUV_index.begin();
						int uvsize = Set->point.at(endP).SPointUV_index.size();//终止点的纹理点个数
						for (auto uv : Set->point.at(endP).SPointUV_index) {
							toAdd.bestPoint.SPointUV_index.insert(uv);//bestPoint保留点的原始uv点标识
							if (Set->point_uv.at(uv).hasNeighbor(p_v1uv)) {//找到相邻纹理坐标
								toAdd.bestPoint.SPointUV_index.erase(uv);//bestPoint删除相邻点uv点标识
								toAdd.bestPoint.PointUV_Vector = Set->point_uv.at(uv).PointUV_Vector;//纹理坐标
								//计算折叠代价
								set<int>::iterator pos1 = find(Set->point.at(endP).SPointUV_index.begin(), Set->point.at(endP).SPointUV_index.end(), uv);
								int dex = distance(Set->point.at(endP).SPointUV_index.begin(), pos1);
								cv::Mat mat1 = Set->point.at(startP).Texture_error;//得到起始点纹理误差矩阵
								cv::Mat mat2 = Set->point.at(endP).Texture_errors[dex];//得到终止点纹理误差矩阵

								toAdd.calculateDelCostUV_Boundary(Set, mat1, mat2); //计算特殊边的折叠代价
								test = true;
							}
						}
					}

				}
				else//不可折叠
				{
					toAdd.delCost = 10000000;
					test = true;
				}

				////计算边对的一邻域点的纹理复杂度
				//double TextrueComplexity = toAdd.calculate_TexComplexity(Set);
				//toAdd.delCost = toAdd.delCost * 100000 * (TextrueComplexity - 0.99);

				//计算边对的顶点尖锐度
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
					//set->point.at(i).pointPair_changeable = false;//p1点设置为不可折叠(边界线的点设置为不可折叠)
					//set->point.at(k).pointPair_changeable = false;//p2点设置为不可折叠
					toAdd.delCost = toAdd.delCost + 10000000;
				}//计算最大法线偏转
				addPair(&toAdd);
			}
		}
	}//按照三角网拓扑结构选边对

}

void PairHeap::setupHeapUV_maxSim(PointSet * Set)
{
	vector<pair<int, int>> boundary;//边界线段集合

	for (int i = 0; i < Set->count; ++i) {//点的个数
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//遍历每一个点的全部一环领域点
		{
			int k = Set->point.at(i).neighbor[j];//邻域点标识
			if (i < k)//避免重复
			{
				/*if (i == 168 && k == 169) {
					int g = 6;
				}*/
				Pair toAdd(i, k);//添加一对
				if (toAdd.JudgeBoundary(Set) == true) {
					boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1点设置为不可折叠(边界线的点设置为不可折叠)
					Set->point.at(k).point_changeable = false;//p2点设置为不可折叠(边界线的点设置为不可折叠)
				}//判断是否为模型边界线段

				//计算边对的一邻域点的纹理复杂度
				double TextrueComplexity = toAdd.calculate_TexComplexity(Set);

				//分类讨论，计算最佳点及折叠代价
				int p_v1, p_v2;
				switch (toAdd.JudgeTex(Set, p_v1, p_v2))
				{
				case 1://p1,p2至少有一个点是单纹理点
					if (HeapClassify_No1(Set, toAdd, p_v1, p_v2, TextrueComplexity) == false) {
						int  oooo = 6;
					}
					break;
				case 2://p1,p2均为多纹理点（不可折叠）
					HeapClassify_No2(Set, toAdd);
					break;
				case 3://其他，主要讨论2-2
					if (HeapClassify_No3(Set, toAdd, p_v1, p_v2, TextrueComplexity) == false) {
						int ooooo = 6;
					}

					break;

				default:
					break;
				}


				if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
					//set->point.at(i).pointPair_changeable = false;//p1点设置为不可折叠(边界线的点设置为不可折叠)
					//set->point.at(k).pointPair_changeable = false;//p2点设置为不可折叠
					double ppp = toAdd.delCost;
					toAdd.delCost = toAdd.delCost + 10000000;
				}//计算最大法线偏转
				//if (toAdd.ChangeTopology(Set) == true) {
				//	toAdd.delCost = toAdd.delCost + 10000000;
				//}//判断拓扑是否改变
				addPair(&toAdd);
			}
		}
	}//按照三角网拓扑结构选边对
}

void PairHeap::setupHeapQEM(PointSet * Set)
{
	for (int i = 0; i < Set->count; ++i) {//点的个数
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//遍历每一个点的全部一环领域点
		{
			int k = Set->point.at(i).neighbor[j];//邻域点标识
			if (i < k)//避免重复
			{
				Pair toAdd(i, k);//添加一对
				if (toAdd.JudgeBoundary(Set) == true) {
					//boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1点设置为不可折叠(边界线的点设置为不可折叠)
					Set->point.at(k).point_changeable = false;//p2点设置为不可折叠(边界线的点设置为不可折叠)
				}//判断是否为边界线段/三角形压覆检测

				bool Success;
				toAdd.calculateBestPointQEM(Set, Success);//计算折叠后点的最佳位置和UV（有解就算，没解就取中点）
				/*if (Success==false) {
					int ss = 6;
				}*/
				toAdd.calculateDelCostQEM(Set);//计算边的折叠代价

				////参数1：计算顶点尖锐度
				//double Sharpness1 = 0;
				//double Sharpness2 = 0;
				//toAdd.calculateVertexSharpness(Set, toAdd.v1, Sharpness1);
				//toAdd.calculateVertexSharpness(Set, toAdd.v2, Sharpness2);
				//double Vertex_sharpness = Sharpness1 + Sharpness2;
				//Vertex_sharpness = Vertex_sharpness / 3.14;
				//
				////参数2
				//double Texture_complexity = toAdd.calculate_TexComplexity(Set);
				//toAdd.delCost = toAdd.delCost * Vertex_sharpness * Texture_complexity;

				if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
					double ppp = toAdd.delCost;
					toAdd.delCost = toAdd.delCost + 10000000;
				}//计算最大法线偏转

				addPair(&toAdd);
			}
		}
	}
}

void PairHeap::setupHeapQEM1(PointSet * Set)
{
	for (int i = 0; i < Set->count; ++i) {//点的个数
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//遍历每一个点的全部一环领域点
		{
			int k = Set->point.at(i).neighbor[j];//邻域点标识
			if (i < k)//避免重复
			{
				Pair toAdd(i, k);//添加一对
				if (toAdd.JudgeBoundary(Set) == true) {
					//boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1点设置为不可折叠(边界线的点设置为不可折叠)
					Set->point.at(k).point_changeable = false;//p2点设置为不可折叠(边界线的点设置为不可折叠)
				}//判断是否为边界线段/三角形压覆检测

				bool Success;
				toAdd.calculateBestPointQEM1(Set, Success);//计算折叠后点的最佳位置和UV（有解就算，没解就取中点）
				/*if (Success==false) {
					int ss = 6;
				}*/
				toAdd.calculateDelCostQEM1(Set);//计算边的折叠代价

				//if (toAdd.JudgeAbnormalCollpase(Set) == true) {
				//	toAdd.delCost = toAdd.delCost + 1000000000000000000;
				//}//异常值控制

				//if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
				//	double ppp = toAdd.delCost;
				//	toAdd.delCost = toAdd.delCost + 10000000000;
				//}//计算最大法线偏转

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
	for (int i = 0; i < Set->count; ++i) {//点的个数
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//遍历每一个点的全部一环领域点
		{
			int k = Set->point.at(i).neighbor[j];//邻域点标识
			if (i < k)//避免重复
			{
				edge++;
				Pair toAdd(i, k);//添加一对
				if (toAdd.JudgeBoundary(Set) == true) {
					//boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1点设置为不可折叠(边界线的点设置为不可折叠)
					Set->point.at(k).point_changeable = false;//p2点设置为不可折叠(边界线的点设置为不可折叠)
				}//判断是否为边界线段/三角形压覆检测

				//参数1：计算顶点尖锐度
				double Sharpness1 = 0;
				double Sharpness2 = 0;
				toAdd.calculateVertexSharpness(Set, toAdd.v1, Sharpness1);
				toAdd.calculateVertexSharpness(Set, toAdd.v2, Sharpness2);
				double Vertex_sharpness = Sharpness1 + Sharpness2;
				//参数2：计算接缝角度误差
				/*double Seam1 = 0;
				double Seam2 = 0;
				toAdd.calculateSeamError(Set, toAdd.v1, Seam1);
				toAdd.calculateSeamError(Set, toAdd.v2, Seam2);
				double Seam_angleerror = (Seam1 + Seam2) / 2;*/
				double Seam_angleerror = 1;
				//参数3：计算纹理复杂度
				//double Texture_complexity = toAdd.calculate_TexComplexity(Set);
				double Texture_complexity = 1;

				//分类讨论，计算最佳折叠点
				int T_edge = toAdd.JudgeStrategy(Set);//判断边类别
				bool Success;
				toAdd.calculateBestPointD_QEM(Set, Success, T_edge);//确定各类边的最佳折叠位置
				/*if (Success==false) {
					int ss = 6;
				}*/
				toAdd.calculateDelCostD_QEM(Set, T_edge, Vertex_sharpness, Seam_angleerror, Texture_complexity);//计算边的折叠代价
				if (toAdd.delCost == 1000000000000) {
					s1++;
				}
				if (toAdd.delCost == 1000000000000000000) {
					s2++;
				}

				if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
					double ppp = toAdd.delCost;
					toAdd.delCost = toAdd.delCost + 1000000000000000000;
				}//计算最大法线偏转,控制折叠

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
	for (int i = 0; i < Set->count; ++i) {//点的个数
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)//遍历每一个点的全部一环领域点
		{
			int k = Set->point.at(i).neighbor[j];//邻域点标识
			if (i < k)//避免重复
			{
				edge++;
				Pair toAdd(i, k);//添加一对
				if (toAdd.JudgeBoundary(Set) == true) {
					//boundary.push_back(make_pair(i, k));
					Set->point.at(i).point_changeable = false;//p1点设置为不可折叠(边界线的点设置为不可折叠)
					Set->point.at(k).point_changeable = false;//p2点设置为不可折叠(边界线的点设置为不可折叠)
				}//判断是否为边界线段/三角形压覆检测

				//参数：计算dc,de
				float dc, dc1, dc2 = 0;
				float de, de1, de2 = 0;
				/*cu.calculation(toAdd.v1, Set, dc1, de1);
				cu.calculation(toAdd.v2, Set, dc2, de2);
				dc = 0.5*(dc1 + dc2);
				de = 0.5*(de1 + de2);*/
				dc = 1;
				de = 1;

				//分类讨论，计算最佳折叠点
				int T_edge = toAdd.JudgeStrategy_MSADO(Set);//判断边类别
				bool Success;
				toAdd.calculateBestPointMSADO(Set, Success, T_edge);//确定各类边的最佳折叠位置
				toAdd.calculateDelCostMSADO(Set, T_edge, Success, dc, de);//计算边的折叠代价

				if (toAdd.delCost == 1000000000000) {
					s1++;
				}
				if (toAdd.delCost == 1000000000000000000) {
					s2++;
				}

				if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
					double ppp = toAdd.delCost;
					toAdd.delCost = toAdd.delCost + 1000000000000000000;
				}//计算最大法线偏转,控制折叠

				addPair(&toAdd);
			}
		}
	}
	int uu = 6;
}

//p1,p2中至少有一个是单纹理点
bool PairHeap::HeapClassify_No1(PointSet* Set, Pair &toAdd, int V1, int V2, double Tex_Complexity)
{
	if (V1 == 1 && V2 == 1) {//带纹理二次误差简化
		bool Success;
		toAdd.calculateBestPointUV(Set, Success);//计算折叠后点的最佳位置和UV（有解就算，没解就取中点）
		toAdd.calculateDelCostUV(Set);//计算边的基本二次误差折叠代价

		toAdd.delCost = toAdd.delCost * 1000 * (Tex_Complexity - 0.99);
		return true;
	}
	else//半边折叠
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

		toAdd.bestPoint.Point_Vector = Set->point.at(endP).Point_Vector;//xyz坐标

		int p_v1uv = *Set->point.at(startP).SPointUV_index.begin();
		int uvsize = Set->point.at(endP).SPointUV_index.size();//终止点的纹理点个数
		for (auto uv : Set->point.at(endP).SPointUV_index) {
			toAdd.bestPoint.SPointUV_index.insert(uv);//bestPoint保留点的原始uv点标识
			if (Set->point_uv.at(uv).hasNeighbor(p_v1uv)) {//找到相邻纹理坐标
				toAdd.bestPoint.SPointUV_index.erase(uv);//bestPoint删除相邻点uv点标识
				toAdd.bestPoint.PointUV_Vector = Set->point_uv.at(uv).PointUV_Vector;//纹理坐标
				//计算折叠代价
				set<int>::iterator pos1 = find(Set->point.at(endP).SPointUV_index.begin(), Set->point.at(endP).SPointUV_index.end(), uv);
				int dex = distance(Set->point.at(endP).SPointUV_index.begin(), pos1);
				cv::Mat mat1 = Set->point.at(startP).Texture_error;//得到起始点纹理误差矩阵
				cv::Mat mat2 = Set->point.at(endP).Texture_errors[dex];//得到终止点纹理误差矩阵

				toAdd.calculateDelCostUV_Boundary(Set, mat1, mat2); //计算边的基本二次误差折叠代价
				//toAdd.delCost = toAdd.delCost * Tex_Complexity;//误差测度公式
				toAdd.delCost = toAdd.delCost * 1000 * (Tex_Complexity - 0.99);//误差测度公式
				return true;
			}
		}
	}

	return false;
}

//p1,p2均为多纹理点
bool PairHeap::HeapClassify_No2(PointSet * set, Pair &toAdd)
{
	toAdd.delCost = 10000000;
	return true;
}

bool PairHeap::HeapClassify_No3(PointSet * Set, Pair &toAdd, int V1, int V2, double Tex_Complexity)
{
	bool Foldable = true;//是否可折叠
	//筛选特殊情况
	int v1_neighborSize = Set->point.at(toAdd.v1).neighbor.size();//v1相邻点个数
	int v2_neighborSize = Set->point.at(toAdd.v2).neighbor.size();//v2相邻点个数
	//for (int i = 0; i < v1_neighborSize; ++i) {
	//	if (Set->point.at(Set->point.at(toAdd.v1).neighbor[i]).hasNeighbor(toAdd.v2)) {
	//		SPoint P = Set->point.at(Set->point.at(toAdd.v1).neighbor[i]);
	//		if (P.SPointUV_index.size() != 1) {
	//			Foldable = false;//不可折叠
	//		}
	//	}
	//}

	int jl1 = 0;
	int jl2 = 0;
	for (int i = 0; i < v1_neighborSize; ++i) {
		int n1 = Set->point.at(Set->point.at(toAdd.v1).neighbor[i]).SPointUV_index.size();//纹理点个数
		if (n1 == 2) {
			jl1++;
		}
	}
	for (int i = 0; i < v2_neighborSize; ++i) {
		int n2 = Set->point.at(Set->point.at(toAdd.v2).neighbor[i]).SPointUV_index.size();//纹理点个数
		if (n2 == 2) {
			jl2++;
		}
	}
	if (jl1 == 2 && jl2 == 2) {
		Foldable = true;//可折叠
	}
	else
	{
		Foldable = false;//不可折叠
	}

	int strat, end;//判断起始点和指向点
	if (Foldable == true) {//可折叠
		if (V1 == 2 && V2 == 2) {
			if (toAdd.calcuateAngleError(Set, toAdd.v1, toAdd.v2) < toAdd.calcuateAngleError(Set, toAdd.v2, toAdd.v1)) {//计算角度误差
				strat = toAdd.v1;
				end = toAdd.v2;
			}
			else
			{
				strat = toAdd.v2;
				end = toAdd.v1;
			}
			//起始点和指向点标记
			toAdd.strat_Point = strat;
			toAdd.end_Point = end;
			//半边折叠计算代价（2-2）
			toAdd.bestPoint.Point_Vector = Set->point.at(end).Point_Vector;//xyz坐标

			int stratP_uvsize = Set->point.at(strat).SPointUV_index.size();//起始点的纹理点个数=2
			int endP_uvsize = Set->point.at(end).SPointUV_index.size();//终止点的纹理点个数=2
			int strat_uv_index = 0;
			int end_uv_index = 0;
			cv::Mat mat1, mat2;
			int judge1 = 0;
			for (auto strat_uv : Set->point.at(strat).SPointUV_index) {//遍历起始点的纹理点
				for (auto end_uv : Set->point.at(end).SPointUV_index) {//遍历终止点的纹理点
					if (Set->point_uv.at(strat_uv).hasNeighbor(end_uv)) {//找到相邻纹理坐标
						judge1++;
						if (strat_uv_index == 0) {
							toAdd.bestPoint.PointUV_Vector = Set->point_uv.at(end_uv).PointUV_Vector;//纹理坐标1
							mat1 = Set->point.at(strat).Texture_errors[0];//得到起始点纹理误差矩阵
							if (end_uv_index == 0) {
								mat2 = Set->point.at(end).Texture_errors[0];//得到终止点纹理误差矩阵
							}
							else
							{
								mat2 = Set->point.at(end).Texture_errors[1];//得到终止点纹理误差矩阵
							}
							toAdd.calculateDelCostUV_twoTotwo(Set, strat_uv_index, mat1, mat2);//计算纹理裂缝折叠代价
						}
						else
						{
							toAdd.bestPoint.PointUV_Vector2 = Set->point_uv.at(end_uv).PointUV_Vector;//纹理坐标2

							mat1 = Set->point.at(strat).Texture_errors[1];//得到起始点纹理误差矩阵
							if (end_uv_index == 0) {
								mat2 = Set->point.at(end).Texture_errors[0];//得到终止点纹理误差矩阵
							}
							else
							{
								mat2 = Set->point.at(end).Texture_errors[1];//得到终止点纹理误差矩阵
							}
							toAdd.calculateDelCostUV_twoTotwo(Set, strat_uv_index, mat1, mat2);//计算纹理裂缝折叠代价
						}

					}
					end_uv_index++;
				}
				strat_uv_index++;
			}
			double EAngle = toAdd.calcuateAngleError(Set, strat, end);//角度误差
			toAdd.delCost = (toAdd.delCost + EAngle) * Tex_Complexity;//误差测度公式
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
			//计算折叠代价（2-多）
			toAdd.delCost = 10000000;
			return true;
		}
	}
	else//不可折叠
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
	pairQueue.push(*p);//进入优先队列
	mapper.insert(map<pair<int, int>, bool> ::value_type(make_pair(p->v1, p->v2), true));//进入map
	//if (p->changeable == true) {
	//	mapper.insert(map<pair<int, int>, bool> ::value_type(make_pair(p->v1, p->v2), true));//进入map
	//}
	//else
	//{
	//	mapper.insert(map<pair<int, int>, bool> ::value_type(make_pair(p->v1, p->v2), false));//进入map
	//}
	//
	//mapper[make_pair(p->v1, p->v2)] = true;//进入map
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
		Pair toret = pairQueue.top();//获取优先队列第一个元素
		pairQueue.pop();//删除优先队列第一个元素
		if (mapper[make_pair(toret.v1, toret.v2)] == true)
			if (set->point.at(toret.v1).point_changeable == true && set->point.at(toret.v2).point_changeable == true)
				return toret;
	}
	Pair empty(-1, -1);
	return empty;
}

void PairHeap::updata()
{
	count = 0;//边对数目更新为0
	while (!pairQueue.empty()) pairQueue.pop();//清空priority_queue<Pair, vector<Pair>, cmp> pairQueue;边对队列（优先队列）
	mapper.erase(mapper.begin(), mapper.end());//清空map<pair<int, int>, bool> mapper;边对（true/false）
}

bool PairHeap::cmp::operator()(Pair& p1, Pair& p2)
{
	return p1.delCost > p2.delCost;
}

