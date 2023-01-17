#include "QuadDOMTree.h"

QuadDOMTree::QuadDOMTree()
{
	this->m_Levels = 0;//��ʼ������
}
//�Ĳ���������
QuadDOMTree::QuadDOMTree(int levels, double miny, double maxy, double minz, double maxz, osg::Vec3 *verts, int *Verts_index, int vertCount)
{
	this->m_Levels = levels;
	this->m_MinY = miny;
	this->m_MaxY = maxy;
	this->m_MinZ = minz;
	this->m_MaxZ = maxz;
	this->Verts = verts;
	this->Verts_index = Verts_index;
	this->m_nVertCount = vertCount;
	createQuadDOMTree(m_Root, levels, m_MinY, m_MaxY, m_MinZ, m_MaxZ);//����ģ���Ĳ���
	createQuadDOMTreeIndex();   //��������
}

QuadDOMTree::~QuadDOMTree()
{
}

bool QuadDOMTree::OBJQuadTree(double minu, double maxu, double minv, double maxv, myobj::Element *tem_Elements, int *Elements_index, osg::Vec2 *texcoord, int elementCount)
{
	this->m_MinU = minu;
	this->m_MaxU = maxu;
	this->m_MinV = minv;
	this->m_MaxV = maxv;
	this->m_Elements = tem_Elements;
	this->Elements_index = Elements_index;
	this->Texs = texcoord;
	this->m_elementCount = elementCount;

	int start_level = 0;
	createObjQuadDOMTree(m_Root, m_Root, start_level, m_MinU, m_MaxU, m_MinV, m_MaxV);//�Զ����¹���ģ���Ĳ���
	return true;
}

void QuadDOMTree::createQuadDOMTree(QuadDOMNode *& node, int depth, double miny, double maxy, double minz, double maxz)
{
	if (depth != 0)
	{
		//��������  
		node = createQuadDOMNode(miny, maxy, minz, maxz);
		node->nLevel = m_Levels - depth;//����
		double midy = (node->minY + node->maxY) / 2.0;
		double midz = (node->minZ + node->maxZ) / 2.0;
		//�����ĸ��ӽ��
		node->nChildCount = 4;
		/* һ��������������޻��֣�

		UL(2)   |    UR(3)
		----------|-----------
		LL(0)   |    LR(1)
		���¶Ը��������͵�ö��
		*/
		node->children[0] = createQuadDOMNode(node->minY, midy, node->minZ, midz);
		node->children[1] = createQuadDOMNode(midy, node->maxY, node->minZ, midz);
		node->children[2] = createQuadDOMNode(node->minY, midy, midz, node->maxZ);
		node->children[3] = createQuadDOMNode(midy, node->maxY, midz, node->maxZ);
		//Ϊ4���ӽڵ����������һ��
		createQuadDOMTree(node->children[0], depth - 1, node->minY, midy, node->minZ, midz);
		createQuadDOMTree(node->children[1], depth - 1, midy, node->maxY, node->minZ, midz);
		createQuadDOMTree(node->children[2], depth - 1, node->minY, midy, midz, node->maxZ);
		createQuadDOMTree(node->children[3], depth - 1, midy, node->maxY, midz, node->maxZ);
	}
}

void QuadDOMTree::createObjQuadDOMTree(QuadDOMNode *& node, QuadDOMNode* &Parentnode, int current_depth, double minu, double maxu, double minv, double maxv)
{
	node = createObjQuadDOMNode(minu, maxu, minv, maxv);                        //�����½ڵ� 
	node->nLevel = current_depth;                                               //�½ڵ���������     
	int numElement = bindingEle2Node(node, Parentnode, minu, maxu, minv, maxv); //��Ԫ�غ͵�ǰ�ڵ�
	double midu = (node->minU + node->maxU) / 2.0;
	double midv = (node->minV + node->maxV) / 2.0;

	if (numElement == 0) {
		node = NULL;//�ڵ���Ч
		Parentnode->nChildCount--;
	}
	else
	{
		if (numElement > 300) {
			//�����ĸ��ӽ��
			node->nChildCount = 4;
			/* һ��������������޻��֣�

			UL(2)   |    UR(3)
			----------|-----------
			LL(0)   |    LR(1)
			���¶Ը��������͵�ö��
			*/
			node->children[0] = createObjQuadDOMNode(node->minU, midu, node->minV, midv);
			node->children[1] = createObjQuadDOMNode(midu, node->maxU, node->minV, midv);
			node->children[2] = createObjQuadDOMNode(node->minU, midu, midv, node->maxV);
			node->children[3] = createObjQuadDOMNode(midu, node->maxU, midv, node->maxV);
			//Ϊ4���ӽڵ����������һ��
			createObjQuadDOMTree(node->children[0], node, node->nLevel + 1, node->minU, midu, node->minV, midv);
			createObjQuadDOMTree(node->children[1], node, node->nLevel + 1, midu, node->maxU, node->minV, midv);
			createObjQuadDOMTree(node->children[2], node, node->nLevel + 1, node->minU, midu, midv, node->maxV);
			createObjQuadDOMTree(node->children[3], node, node->nLevel + 1, midu, node->maxU, midv, node->maxV);
			//�ͷŶ����ڴ�
			//node->pEleList->clear();

		}
		else
		{
			//���ӽڵ�
			node->nChildCount = 0;
		}
		//�����Ĳ����������
		if (node->nLevel + 1 >= m_Levels) m_Levels = node->nLevel + 1;
	}
}

QuadDOMNode * QuadDOMTree::createObjQuadDOMNode(double minu, double maxu, double minv, double maxv)
{
	QuadDOMNode * node = (QuadDOMNode *)malloc(sizeof(QuadDOMNode));
	node->minU = minu;
	node->maxU = maxu;
	node->minV = minv;
	node->maxV = maxv;
	node->elementCount = 0;
	node->nChildCount = 0;
	node->nLevel = 0;
	node->children[0] = NULL;
	node->children[1] = NULL;
	node->children[2] = NULL;
	node->children[3] = NULL;
	node->pEleList = new QList<myobj::Element *>();
	node->pEleIndex = new QList<int*>();
	return node;
}


int QuadDOMTree::bindingEle2Node(QuadDOMNode *& node, QuadDOMNode *& Parentnode, double minu, double maxu, double minv, double maxv)
{
	int bindingNum = 0;
	if (Parentnode->elementCount == 0) {//���ڵ�

		for (int i = 0; i < m_elementCount; i++) {
			myobj::Element& element = m_Elements[i];
			node->pEleList->append(&m_Elements[i]);//��Ԫ��
			node->pEleIndex->append(&Elements_index[i]);//��Ԫ�صı�ʶ
			node->elementCount++;
			bindingNum++;
		}

	}
	else//�Ǹ��ڵ�
	{
		for (int j = 0; j < Parentnode->pEleIndex->size(); j++) {
			myobj::Element &element = *Parentnode->pEleList->at(j);//Ԫ��
			int *index = Parentnode->pEleIndex->at(j);//��ʶ
			osg::Vec2 t1 = Texs[element.texCoordIndices[0]];//�����1
			osg::Vec2 t2 = Texs[element.texCoordIndices[1]];//�����2
			osg::Vec2 t3 = Texs[element.texCoordIndices[2]];//�����3
			double u = (t1.x() + t2.x() + t3.x()) / 3.0;
			double v = (t1.y() + t2.y() + t3.y()) / 3.0;
			if (u > minu&&u < maxu&&v>minv&&v < maxv) {
				node->pEleList->append(Parentnode->pEleList->at(j));//��Ԫ��
				node->pEleIndex->append(Parentnode->pEleIndex->at(j));//��Ԫ�صı�ʶ
				node->elementCount++;
				bindingNum++;
			}
		}
	}

	return bindingNum;
}


QuadDOMNode * QuadDOMTree::createQuadDOMNode(double miny, double maxy, double minz, double maxz)
{
	QuadDOMNode * node = (QuadDOMNode *)malloc(sizeof(QuadDOMNode));
	node->minY = miny;
	node->maxY = maxy;
	node->minZ = minz;
	node->maxZ = maxz;
	node->nCount = 0;
	node->nChildCount = 0;
	node->nLevel = 0;
	node->children[0] = NULL;
	node->children[1] = NULL;
	node->children[2] = NULL;
	node->children[3] = NULL;
	node->pVertList = new QList<osg::Vec3*>();
	node->pVertIndex = new QList<int*>();
	return node;
}

//�����Ĳ�������
void QuadDOMTree::createQuadDOMTreeIndex()
{
	for (int i = 0; i < this->m_nVertCount; i++) {
		bindingVert2QuadDOMTree(&Verts[i], &Verts_index[i], m_Root);
	}
}

//�󶨶�����Ĳ���
void QuadDOMTree::bindingVert2QuadDOMTree(osg::Vec3 * ver, int *i, QuadDOMNode *& node)
{
	if (node == NULL) {
		return;
	}

	//node���������ε��������,�����ཻ����r
	if (!(ver->y() < node->minY || ver->y() > node->maxY || ver->z() < node->minZ || ver->z() > node->maxZ)) {
		if (node->nChildCount == 0) {//���²�
			node->pVertList->append(ver);//���ָ��
			node->pVertIndex->append(i);//���ı�ʶ
			node->nCount++;
		}
		else {//�ҵ��������ڵ����²�ڵ�
			for (int j = 0; j < 4; j++) {
				bindingVert2QuadDOMTree(ver, i, node->children[j]);
			}
		}
	}
}


