#pragma once
#include "QuadDOMNode.h"
#include <osg/Vec3f>
#include <osg/Geometry>
#include "gocReadDom.h"

class QuadDOMTree
{
public:
	int m_Levels;         //�ܲ���
	QuadDOMNode * m_Root; //�����
	//���ӽ����귶Χ
	double m_MinY;
	double m_MaxY;
	double m_MinZ;
	double m_MaxZ;
	//�������귶Χ
	double m_MinU;
	double m_MaxU;
	double m_MinV;
	double m_MaxV;
	osg::Vec3 *Verts; //ģ�����ж���
	int *Verts_index; //ģ�����ж����ʶ
	int m_nVertCount;                                   //ģ�����ж������
	myobj::Element *m_Elements;                         //��ǰģ������Ԫ��
	int *Elements_index;                                //��ǰģ��Ԫ�ر�ʶ
	osg::Vec2 *Texs;                                    //ģ��������������
	int m_elementCount;                                 //��ǰģ��Ԫ�ظ���

	QuadDOMTree();
	QuadDOMTree(int levels,                             //���
		double miny,                                    //�������
		double maxy,
		double minz,
		double maxz,
		osg::Vec3 *verts,                               //ģ�����ж���
		int *Verts_index,
		int vertCount);                                 //�������

	~QuadDOMTree();
	/*
	���������ڵ����򣬲����ظ�������������б�
	*/
	//void getUv2XyzSubList(QList<osg::Vec3Array *> * &uv2XyzList, int &triCount,   //���������������б�
	//	double u, double v,                                              //��������
	//	QuadDOMNode* &node);                                             //�Ĳ��������

	/*
	��ȡ�Ĳ��������
	*/
	QuadDOMNode* getRootNode() {
		return this->m_Root;
	}

	/*
	OBJģ�͹����Ĳ���
	*/
	bool OBJQuadTree(
		double minu,                                       //�������
		double maxu,
		double minv,
		double maxv,
		myobj::Element *tem_Elements,                    //��ǰģ��Ԫ�ؼ���
		int *Elements_index,                              //��ǰģ��Ԫ�ر�ʶ����
		osg::Vec2 *texcoord,                              //ģ�������������꼯��
		int elementCount
	);

private:
	//�����Ĳ������ݹ飩
	void createQuadDOMTree(QuadDOMNode* &node, int depth, double miny, double maxy, double minz, double maxz);
	//�������
	QuadDOMNode* createQuadDOMNode(double miny, double maxy, double minz, double maxz);
	//�����Ĳ�������
	void createQuadDOMTreeIndex();
	//�󶨶�����Ĳ���
	void bindingVert2QuadDOMTree(osg::Vec3 *ver, int *i, QuadDOMNode* &node);

	//OBJģ�ʹ����Ĳ������ݹ飩
	void createObjQuadDOMTree(QuadDOMNode* &node, QuadDOMNode* &Parentnode, int current_depth, double minu, double maxu, double minv, double maxv);
	//OBJ�������
	QuadDOMNode* createObjQuadDOMNode(double minu, double maxu, double minv, double maxv);
	//OBJ��Ԫ����ڵ㲢���ظ���
	int bindingEle2Node(QuadDOMNode *& node, QuadDOMNode* &Parentnode, double minu, double maxu, double minv, double maxv);
};

