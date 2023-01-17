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

	string inputDOMfile;//原始模型路径
	string inputMTLfile;//模型对应MTL文件路径
	string Tex_relative_path;//当前模块对应纹理图片相对路径
	string Tex_absolute_path;//当前模块对应纹理图像绝对路径
	string outputDOMLodfile; //LOD模型另存路径
	string outputDOMLodFolder; //LOD模型另存文件夹
	string lodDomName;         //LOD模型名称
	string outputTexturefile;//图片另存路径

	int CurrentPartID;//当前模块标识
	int StartLine;//某一模块起始行
	int EndLine;//某一模块终止行
	osg::ref_ptr<osg::Vec3Array> modular;//某一模块点集合
	PointSet pointSet;//点操作对象
	QList <osg::ref_ptr<osg::Vec3Array>> modular_List;//模型所有模块的点集合
	QList<QuadDOMTree> DomTrees;//模型所有模块顶点四叉树

	bool buildDOMLod_Quadtree(); //四叉树多级LOD构建

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

	osg::Matrix R, R_i;//R为旋转矩阵，R_i为旋转矩阵的逆
	double heading;//y
	double pitch;//x
	double roll;//z
private:
	int LodLayer;           //LOD模型层数

	double _Umin = 0.0;//纹理坐标U最小值
	double _Umax = 1.0;//纹理坐标U最大值
	double _Vmin = 0.0;//纹理坐标V最小值
	double _Vmax = 1.0;//纹理坐标V最大值

	double _Xmin;//旋转后模型的平面范围横坐标最小值
	double _Xmax;//旋转后模型的平面范围横坐标最大值
	double _Ymin;//旋转后模型的平面范围横坐标最小值
	double _Ymax;//旋转后模型的平面范围横坐标最大值
	double _Zmin;//旋转后模型的平面范围纵坐标最小值
	double _Zmax;//旋转后模型的平面范围纵坐标最大值
	double _Xlength;//旋转后模型的平面范围纵坐标长度，_Xlength=_Xmax-_Xmin
	double _Ylength;//旋转后模型的平面范围横坐标长度，_Ylength=_Ymax-_Ymin
	double _Zlength;//旋转后模型的平面范围纵坐标长度，_Zlength=_Zmax-_Zmin

	osg::Vec3 _obbcenter;   // obb 中心
	osg::Vec3 _x;           // obb的x轴，单位矢量(最短)
	osg::Vec3 _y;           // obb的y轴，单位矢量（最长）
	osg::Vec3 _z;           // obb的z轴，单位矢量
	osg::ref_ptr<osg::Vec3Array> obb_corner;//包围盒8个顶点
	osg::ref_ptr<osg::Vec3Array> _corner;//包围盒一个角
	osg::Vec3 _P;           // 角顶点
	osg::Vec3 P_x;          // obbx轴的顶点
	osg::Vec3 P_y;          // obby轴的顶点
	osg::Vec3 P_z;          // obbz轴的顶点

};
