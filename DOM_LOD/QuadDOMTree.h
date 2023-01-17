#pragma once
#include "QuadDOMNode.h"
#include <osg/Vec3f>
#include <osg/Geometry>
#include "gocReadDom.h"

class QuadDOMTree
{
public:
	int m_Levels;         //总层数
	QuadDOMNode * m_Root; //根结点
	//正视角坐标范围
	double m_MinY;
	double m_MaxY;
	double m_MinZ;
	double m_MaxZ;
	//纹理坐标范围
	double m_MinU;
	double m_MaxU;
	double m_MinV;
	double m_MaxV;
	osg::Vec3 *Verts; //模型所有顶点
	int *Verts_index; //模型所有顶点标识
	int m_nVertCount;                                   //模型所有顶点个数
	myobj::Element *m_Elements;                         //当前模块所有元素
	int *Elements_index;                                //当前模块元素标识
	osg::Vec2 *Texs;                                    //模型所有纹理坐标
	int m_elementCount;                                 //当前模块元素个数

	QuadDOMTree();
	QuadDOMTree(int levels,                             //深度
		double miny,                                    //外包矩形
		double maxy,
		double minz,
		double maxz,
		osg::Vec3 *verts,                               //模型所有顶点
		int *Verts_index,
		int vertCount);                                 //顶点个数

	~QuadDOMTree();
	/*
	搜索点所在的区域，并返回该区域的三角形列表
	*/
	//void getUv2XyzSubList(QList<osg::Vec3Array *> * &uv2XyzList, int &triCount,   //输出结果：三角形列表
	//	double u, double v,                                              //纹理坐标
	//	QuadDOMNode* &node);                                             //四叉树根结点

	/*
	获取四叉树根结点
	*/
	QuadDOMNode* getRootNode() {
		return this->m_Root;
	}

	/*
	OBJ模型构建四叉树
	*/
	bool OBJQuadTree(
		double minu,                                       //外包矩形
		double maxu,
		double minv,
		double maxv,
		myobj::Element *tem_Elements,                    //当前模块元素集合
		int *Elements_index,                              //当前模块元素标识集合
		osg::Vec2 *texcoord,                              //模型整体纹理坐标集合
		int elementCount
	);

private:
	//创建四叉树（递归）
	void createQuadDOMTree(QuadDOMNode* &node, int depth, double miny, double maxy, double minz, double maxz);
	//创建结点
	QuadDOMNode* createQuadDOMNode(double miny, double maxy, double minz, double maxz);
	//创建四叉树索引
	void createQuadDOMTreeIndex();
	//绑定顶点和四叉树
	void bindingVert2QuadDOMTree(osg::Vec3 *ver, int *i, QuadDOMNode* &node);

	//OBJ模型创建四叉树（递归）
	void createObjQuadDOMTree(QuadDOMNode* &node, QuadDOMNode* &Parentnode, int current_depth, double minu, double maxu, double minv, double maxv);
	//OBJ创建结点
	QuadDOMNode* createObjQuadDOMNode(double minu, double maxu, double minv, double maxv);
	//OBJ绑定元素与节点并返回个数
	int bindingEle2Node(QuadDOMNode *& node, QuadDOMNode* &Parentnode, double minu, double maxu, double minv, double maxv);
};

