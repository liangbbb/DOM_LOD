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
	string inputFile;//ģ������·��
	string inputMTL;//obj��ʽģ��MTL����·��
	string outputFile;//ģ�����·��
	string Tex_absolute_path;//��ǰ���Ӧ����ͼƬ����·��
	string Tex_relative_path;//��ǰ���Ӧ����ͼƬ���·��
	string outputTextureFile;//ͼƬ���·��
	string outputfolder;     //ģ������ļ���
	string outputDomName;    //ģ������

	double ratio;//����
	double Tex_ratio;//�������

	PointSet pointSet;//���������
	deque<cv::Point2f>pointUV;//��̬�洢���������
	PairHeap pairHeap;//�߶Բ�������


	myobj::Model readOBJToModel(string inputDOMfile);//Read the data and turn it into myobj::Model object
	
	
	bool ModGroInitial(myobj::Model model, std::map< myobj::ElementState, myobj::Model::ElementList > &elementStateMap, std::map< myobj::ElementState, myobj::Model::ElementList >::iterator &elementStateMap_index_itr, vector<int> Effective_element);// innitial myobj::Model object
	void updateData_UVPoint(Pair toDel);//������������꼯��
	void updateData_Point(Pair toDel);//���������������

	osg::Geode* LODCreateLeaf();  //������ģ�ͽڵ㣨�ֿ�ڵ�����ӳ�䣩

private:
	int Sim_Method;//�򻯷���
	bool ImageSample;//Ӱ���ϡ
	bool texSeamPreserve;

	bool updatepointSet();//���µ��������
	bool updatepairHeap();//���±߶Բ�������

	int pointCount;//������������������飩
	int pointUVCount;//�������������������飩
	int faceCount;//�������������ǰ�飩
public:
	Simplification();

	void setMethod(int sim_method);//ѡ��򻯷���
	void setInput(string _input);//����obj��ʽ���ݵľ���·��
	void setLODInput(string _input);//����obj��ʽ���ݵľ���·��
	void setOutput(string _output);//����򻯺�ģ�����·��
	void setLODOutput(string _output);//����LODģ�����·��
	void setRatio(double _ratio);//���ü򻯱���
	void setImageSample(bool sample);//��������Ӱ���Ƿ��ϡ

	


	osg::Geode* Simplify_lod(myobj::Model DOM_model, std::map<myobj::ElementState, myobj::Model::ElementList>& elementStateMap, std::map<myobj::ElementState, myobj::Model::ElementList>::iterator & elementStateMap_index_itr, QList<int*>  *pEleIndex, double DestructRate, double TexDestructRate, string Partfolder);//LOD����QEM��


	bool lodImagethin();//lodͼ���ϡ
	

	bool split(char* strLine, char ** &res); //�ַ����ָ�
	bool split1(char* strLine, char ** &res); //�ַ����ָ�
	ifstream & seek_to_line(ifstream & in, int line);//��λ���ļ����ض���
	void computeNormal(double v1[3], double v2[3], double v3[3], double n[3]);//���㷨����
	void computeNormalDirection(double v1[3], double v2[3], double v3[3], double n[3]);//���㷨��������

	bool judgeTirSame(QList<int> &Tir_index, int Tir);//�ж�QList���Ƿ����ĳ��������
	bool IsInBox(vector<int> pointindex, int index[3]);//�ж��������Ƿ����ڰ�Χ��������
};

#endif
