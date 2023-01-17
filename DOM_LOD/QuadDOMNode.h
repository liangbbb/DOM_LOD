#pragma once
#include <QList.h>
#include <osg/Vec3f>
#include <osg/Geometry>
#include "gocReadDom.h"

//露头模型四叉树结点定义
typedef struct QuadDOMNode
{
public:
	double minY, minZ, maxY, maxZ;      //节点所代表的矩形区域
	double minU, minV, maxU, maxV;      //节点所代表的矩形区域（纹理坐标）
	int nCount;                         //节点所包含的所有空间对象个数
	int elementCount;                   //节点所包含的所有空间对象个数
	//QList<Uv2Xyz*>  *pUv2XyzList;     //空间对象指针数组
	QList<osg::Vec3 *>  *pVertList;     //空间对象指针数组
	QList<myobj::Element *>  *pEleList; //空间对象指针数组
	QList<int*>  *pVertIndex;           //空间对象标识数组
	QList<int*>  *pEleIndex;            //空间对象标识数组
	int         nChildCount;            //子节点个数
	int         nLevel;                 //该节点所属层数
	QuadDOMNode *children[4];           //指向节点的四个孩子
}QuadDOMNode;

