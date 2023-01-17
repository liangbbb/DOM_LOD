#pragma once
#ifndef SIMPLIFICATION_H
#define SIMPLIFICATION_H
#include <string>
#include "PointSet.h"
#include "PairHeap.h"
#include <opencv2/opencv.hpp>
#include <osg/Group>
#include <osg/NodeVisitor>
#include <qlist.h>
#include "CU.h"
#include "gocReadDom.h"


using std::string;
class Simplification
{
public:
	string inputFile;//模型数据路径
	string inputMTL;//obj格式模型MTL数据路径
	string outputFile;//模型另存路径
	string Tex_absolute_path;//当前组对应纹理图片绝对路径
	string Tex_relative_path;//当前组对应纹理图片相对路径
	string outputTextureFile;//图片另存路径
	string outputfolder;     //模型另存文件夹
	string outputDomName;    //模型名称

	double ratio;//简化率
	double Tex_ratio;//纹理简化率

	PointSet pointSet;//点操作对象
	deque<cv::Point2f>pointUV;//动态存储所有纹理点
	PairHeap pairHeap;//边对操作对象


	myobj::Model readOBJToModel(string inputDOMfile);//Read the data and turn it into myobj::Model object
	
	
	bool ModGroInitial(myobj::Model model, std::map< myobj::ElementState, myobj::Model::ElementList > &elementStateMap, std::map< myobj::ElementState, myobj::Model::ElementList >::iterator &elementStateMap_index_itr, vector<int> Effective_element);// innitial myobj::Model object
	void updateData_UVPoint(Pair toDel);//更新纹理点坐标集合
	void updateData_Point(Pair toDel);//带纹理二次误差更新

	osg::Geode* LODCreateLeaf();  //创建新模型节点（分块节点纹理映射）

private:
	int Sim_Method;//简化方法
	bool ImageSample;//影像抽稀
	bool texSeamPreserve;

	bool updatepointSet();//更新点操作对象
	bool updatepairHeap();//更新边对操作对象

	int pointCount;//顶点个数（遍历过的组）
	int pointUVCount;//纹理点个数（遍历过的组）
	int faceCount;//三角面个数（当前组）
public:
	Simplification();

	void setMethod(int sim_method);//选择简化方法
	void setInput(string _input);//输入obj格式数据的绝对路径
	void setLODInput(string _input);//输入obj格式数据的绝对路径
	void setOutput(string _output);//输入简化后模型另存路径
	void setLODOutput(string _output);//输入LOD模型另存路径
	void setRatio(double _ratio);//设置简化比例
	void setImageSample(bool sample);//设置纹理影像是否抽稀

	


	osg::Geode* Simplify_lod(myobj::Model DOM_model, std::map<myobj::ElementState, myobj::Model::ElementList>& elementStateMap, std::map<myobj::ElementState, myobj::Model::ElementList>::iterator & elementStateMap_index_itr, QList<int*>  *pEleIndex, double DestructRate, double TexDestructRate, string Partfolder);//LOD调用QEM简化


	bool lodImagethin();//lod图像抽稀
	

	bool split(char* strLine, char ** &res); //字符串分割
	bool split1(char* strLine, char ** &res); //字符串分割
	ifstream & seek_to_line(ifstream & in, int line);//定位到文件的特定行
	void computeNormal(double v1[3], double v2[3], double v3[3], double n[3]);//计算法向量
	void computeNormalDirection(double v1[3], double v2[3], double v3[3], double n[3]);//计算法向量方向

	bool judgeTirSame(QList<int> &Tir_index, int Tir);//判断QList中是否包含某个三角形
	bool IsInBox(vector<int> pointindex, int index[3]);//判断三角形是否属于包围盒区域内
};

#endif
