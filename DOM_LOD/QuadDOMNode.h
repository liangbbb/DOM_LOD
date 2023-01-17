#pragma once
#include <QList.h>
#include <osg/Vec3f>
#include <osg/Geometry>
#include "gocReadDom.h"

//¶ͷģ���Ĳ�����㶨��
typedef struct QuadDOMNode
{
public:
	double minY, minZ, maxY, maxZ;      //�ڵ�������ľ�������
	double minU, minV, maxU, maxV;      //�ڵ�������ľ��������������꣩
	int nCount;                         //�ڵ������������пռ�������
	int elementCount;                   //�ڵ������������пռ�������
	//QList<Uv2Xyz*>  *pUv2XyzList;     //�ռ����ָ������
	QList<osg::Vec3 *>  *pVertList;     //�ռ����ָ������
	QList<myobj::Element *>  *pEleList; //�ռ����ָ������
	QList<int*>  *pVertIndex;           //�ռ�����ʶ����
	QList<int*>  *pEleIndex;            //�ռ�����ʶ����
	int         nChildCount;            //�ӽڵ����
	int         nLevel;                 //�ýڵ���������
	QuadDOMNode *children[4];           //ָ��ڵ���ĸ�����
}QuadDOMNode;

