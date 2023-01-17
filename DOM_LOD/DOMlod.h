#pragma once
#include <iostream>
#include <osg/Vec3f>
#include <osg/Geometry>
#include <QString>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <string>
#include <qlist.h>
#include "QuadDOMTree.h"
#include "PointSet.h"
#include "gocReadDom.h"
#include<osgViewer/Viewer>
#include<osg/Node>
#include<osg/Group>
#include<osg/Geode>
#include<osg/PagedLOD>
#include<osg/PositionAttitudeTransform>
#include<osg/MatrixTransform>
#include<osgDB/ReadFile>
#include<osgDB/WriteFile>
#include<osgGA/TrackballManipulator>
#include<osgUtil/Optimizer>
#include<iostream>


using namespace std;
class DOMLod
{
public:
	DOMLod(string inputFile, string outputFile);
	~DOMLod();

	string inputDOMfile;//ԭʼģ��·��
	string inputMTLfile;//ģ�Ͷ�ӦMTL�ļ�·��
	string Tex_relative_path;//��ǰģ���Ӧ����ͼƬ���·��
	string Tex_absolute_path;//��ǰģ���Ӧ����ͼ�����·��
	string outputDOMLodfile; //LODģ�����·��
	string outputDOMLodFolder; //LODģ������ļ���
	string lodDomName;         //LODģ������
	string outputTexturefile;//ͼƬ���·��

	int CurrentPartID;//��ǰģ���ʶ
	int StartLine;//ĳһģ����ʼ��
	int EndLine;//ĳһģ����ֹ��
	osg::ref_ptr<osg::Vec3Array> modular;//ĳһģ��㼯��
	PointSet pointSet;//���������
	QList <osg::ref_ptr<osg::Vec3Array>> modular_List;//ģ������ģ��ĵ㼯��
	QList<QuadDOMTree> DomTrees;//ģ������ģ�鶥���Ĳ���

	bool buildDOMLod_Quadtree(); //�Ĳ����༶LOD����

	myobj::Model DOM_model;//myobj::Model object
	std::map<myobj::ElementState, myobj::Model::ElementList> groupElementStateMap;//myobj::ElementState to myobj::Model::ElementList mapping
	std::map< myobj::ElementState, myobj::Model::ElementList >::iterator groupElementStateMap_itr;
	myobj::Model readOBJToModel(string inputDOMfile);//Read the data and turn it into myobj::Model object
	bool QuadtreeInitial(std::map< myobj::ElementState,
		myobj::Model::ElementList > &elementStateMap,
		std::map< myobj::ElementState,
		myobj::Model::ElementList >::iterator &elementStateMap_index_itr);                                       //innitial myobj::Model Quadtree
	bool PartLod();                                                                                              //partition build LOD
	bool getLayerLod(string Partfolder, QuadDOMNode * node, int n_Levels, string ParentCode, int region);        //Recursive create LOD

	osg::Matrix R, R_i;//RΪ��ת����R_iΪ��ת�������
	double heading;//y
	double pitch;//x
	double roll;//z
private:
	int LodLayer;           //LODģ�Ͳ���

	double _Umin = 0.0;//��������U��Сֵ
	double _Umax = 1.0;//��������U���ֵ
	double _Vmin = 0.0;//��������V��Сֵ
	double _Vmax = 1.0;//��������V���ֵ

	double _Xmin;//��ת��ģ�͵�ƽ�淶Χ��������Сֵ
	double _Xmax;//��ת��ģ�͵�ƽ�淶Χ���������ֵ
	double _Ymin;//��ת��ģ�͵�ƽ�淶Χ��������Сֵ
	double _Ymax;//��ת��ģ�͵�ƽ�淶Χ���������ֵ
	double _Zmin;//��ת��ģ�͵�ƽ�淶Χ��������Сֵ
	double _Zmax;//��ת��ģ�͵�ƽ�淶Χ���������ֵ
	double _Xlength;//��ת��ģ�͵�ƽ�淶Χ�����곤�ȣ�_Xlength=_Xmax-_Xmin
	double _Ylength;//��ת��ģ�͵�ƽ�淶Χ�����곤�ȣ�_Ylength=_Ymax-_Ymin
	double _Zlength;//��ת��ģ�͵�ƽ�淶Χ�����곤�ȣ�_Zlength=_Zmax-_Zmin

	osg::Vec3 _obbcenter;   // obb ����
	osg::Vec3 _x;           // obb��x�ᣬ��λʸ��(���)
	osg::Vec3 _y;           // obb��y�ᣬ��λʸ�������
	osg::Vec3 _z;           // obb��z�ᣬ��λʸ��
	osg::ref_ptr<osg::Vec3Array> obb_corner;//��Χ��8������
	osg::ref_ptr<osg::Vec3Array> _corner;//��Χ��һ����
	osg::Vec3 _P;           // �Ƕ���
	osg::Vec3 P_x;          // obbx��Ķ���
	osg::Vec3 P_y;          // obby��Ķ���
	osg::Vec3 P_z;          // obbz��Ķ���

};
