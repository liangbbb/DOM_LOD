#include "DOMLod.h"
#include <string>
#include <qstringlist.h>
#include "Simplification.h"

#include <osgDB/WriteFile>
#include <iostream>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <qfile.h>
#include <qdir.h>

using namespace std;
using namespace cv;
DOMLod::DOMLod(string inputFile, string outputFile)
{
	inputDOMfile = inputFile;//ԭʼģ��·��
	outputDOMLodfile = outputFile;//LODģ�����·��
}

DOMLod::~DOMLod()
{
}


bool DOMLod::buildDOMLod_Quadtree()
{
	/*�ļ���Ч�Լ��*/
	QFile DOMfile(QString::fromLocal8Bit(inputDOMfile.data()));
	if (!DOMfile.exists())return false;

	/*���lodģ������*/
	lodDomName = outputDOMLodfile.substr(outputDOMLodfile.find_last_of('/') + 1, outputDOMLodfile.find_last_of('.') - outputDOMLodfile.find_last_of('/') - 1);//ģ������;

	/*����LOD���ļ���*/
	outputDOMLodFolder = outputDOMLodfile.substr(0, outputDOMLodfile.find_last_of('/'));
	std::string LODFile = outputDOMLodfile.substr(0, outputDOMLodfile.find_last_of('/') + 1) + lodDomName;
	QDir dir;
	dir.mkdir(QString::fromLocal8Bit(LODFile.data()));//�����ļ���
	/*����Dataģ�����ļ���*/
	std::string DataFile = LODFile + "/Data";
	QDir dir1;
	dir1.mkdir(QString::fromLocal8Bit(DataFile.data()));//�����ļ���

	/*��ȡģ��myobj::Model����*/
	DOM_model = readOBJToModel(inputDOMfile);

	/*����ElementStateMap elementStateMap,ģ�ͷ��鹹��LOD*/
	groupElementStateMap = DOM_model.elementStateMap;
	groupElementStateMap_itr = groupElementStateMap.begin();
	int part_index = 0;//ģ���ʶ
	while (QuadtreeInitial(groupElementStateMap, groupElementStateMap_itr))
		//����1��model�����ʼ����ȷ��element�ķ����������Ĳ���
	{
		CurrentPartID = part_index; //ģ���ʶ��ֵ
		//����2����ģ��ģ�ͷ����ֲ�򻯽���LODģ��
		/*if (CurrentPartID < 9) {
			groupElementStateMap_itr++;
			part_index++;
			continue;
		}*/
		PartLod();

		groupElementStateMap_itr++;
		part_index++;
	}

	/*�����ܽ��*/
	osg::ref_ptr<osg::Group> group = new osg::Group();
	for (int i = 0; i < part_index; i++) {
		std::string Path = DataFile + "/" + lodDomName + "_L0_0_R0_" + to_string(i) + "/" + lodDomName + "_L0_0_R0_" + to_string(i) + ".osgb";
		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(Path);
		osg::ref_ptr<osg::PagedLOD> lod = new osg::PagedLOD();

		auto bs = node->getBound();
		auto c = bs.center();
		auto r = bs.radius();
		lod->setCenter(c);
		lod->setRadius(r);
		lod->setRangeMode(osg::LOD::RangeMode::PIXEL_SIZE_ON_SCREEN);

		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->getOrCreateStateSet();
		lod->addChild(geode.get());

		std::string relativeFilePath = "Data/" + lodDomName + "_L0_0_R0_" + to_string(i) + "/" + lodDomName + "_L0_0_R0_" + to_string(i) + ".osgb"; //���·��

		lod->setFileName(0, "");
		lod->setFileName(1, relativeFilePath);

		lod->setRange(0, 0, 1.0);			//��һ�㲻�ɼ�
		lod->setRange(1, 1.0, FLT_MAX);

		lod->setDatabasePath("");

		group->addChild(lod);
	}

	std::string outputLodFile = LODFile + "/" + lodDomName + ".osgb";
	osgDB::writeNodeFile(*group, outputLodFile);

	return true;
}

//999
myobj::Model DOMLod::readOBJToModel(string inputDOMfile)
{
	osgDB::ifstream fin(inputDOMfile.c_str());
	if (fin)
	{
		osg::ref_ptr<osgDB::ReaderWriter::Options> local_opt = new osgDB::ReaderWriter::Options;
		local_opt->getDatabasePathList().push_front(osgDB::getFilePath(inputDOMfile));
		//OBJ convert to model object
		myobj::Model model;
		model.setDatabasePath(osgDB::getFilePath(inputDOMfile.c_str()));
		model.readOBJ(fin, local_opt.get());
		return model;
	}
}
//999
bool DOMLod::QuadtreeInitial(std::map<myobj::ElementState, myobj::Model::ElementList>& elementStateMap, std::map<myobj::ElementState, myobj::Model::ElementList>::iterator & elementStateMap_index_itr)
{
	if (elementStateMap_index_itr != elementStateMap.end())
	{
		/*��ȡ����״̬����Ӧ��Ԫ�ؼ���*/
		myobj::ElementState tem_currentElementState = elementStateMap_index_itr->first;//currentElementState
		myobj::Model::ElementList tem_currentElementList = elementStateMap_index_itr->second;//currentElementList
		int ElementCount = tem_currentElementList.size();//Number of triangular surfaces

		/*��ȡ��������ͼƬ���·�������·��*/
		myobj::Material tem_currentMaterial = DOM_model.materialMap[tem_currentElementState.materialName];//���ʱ�ʶtem_currentElementState.materialName;
		for (int i = 0; i < tem_currentMaterial.maps.size(); ++i)
		{
			Tex_relative_path = tem_currentMaterial.maps[i].name;//���·��
			Tex_absolute_path = DOM_model.databasePath + "/" + Tex_relative_path;//����·��
			break;
		}

		/*�����Ĳ���*/
		QuadDOMTree tem_DomTree;
		int *modular_Eleindex = new int[ElementCount];                           //��ģ����Ԫ�ر�ʶ����
		myobj::Element *modular_Element = new myobj::Element[ElementCount];      //��ģ����Ԫ�ؼ���
		osg::Vec2 *texcoord = new osg::Vec2[DOM_model.texcoords.size()];         //ģ�������������꼯��
		for (int i = 0; i < ElementCount; i++) {
			modular_Element[i] = *tem_currentElementList[i];
			modular_Eleindex[i] = i;
		}
		for (int j = 0; j < DOM_model.texcoords.size(); j++) {
			texcoord[j] = DOM_model.texcoords[j];
		}
		tem_DomTree.OBJQuadTree(_Umin, _Umax, _Vmin, _Vmax, modular_Element, modular_Eleindex, texcoord, ElementCount);//��ģ���Ĳ���
		DomTrees.append(tem_DomTree);//�Ĳ������
		return true;
	}
	return false;
}

/*
1.������ģ���ļ���
2.��ȡ��ģ���Ĳ������ڵ�
3.�ݹ鹹��ÿ��lod
*/
//999
bool DOMLod::PartLod()
{
	//����1��������ģ���ļ���
	string Partfolder = outputDOMLodFolder + "/" + lodDomName + "/Data/" + lodDomName + "_L0_0_R0_" + to_string(CurrentPartID);
	QDir dir;
	dir.mkdir(QString::fromLocal8Bit(Partfolder.data()));
	//����2����ȡQList�д�ŵ����һ��ģ����Ĳ������ڵ���ܲ���
	QuadDOMNode * Root = DomTrees.at(DomTrees.size() - 1).m_Root;
	int n_Levels = DomTrees.at(DomTrees.size() - 1).m_Levels;
	//����3���ݹ鹹��ĳһģ��LOD
	getLayerLod(Partfolder, Root, n_Levels, "", 0);
	return true;
}

bool DOMLod::getLayerLod(string Partfolder, QuadDOMNode * node, int n_Levels, string ParentCode, int region)
{
	if (node == NULL)return false;
	double parm_DestructionRate = 0.8;                                                        //������ʣ���=0.8
	int Currentlayer = node->nLevel;                                                          //��ǰnode�������
	double DestructionRate = pow(parm_DestructionRate, n_Levels - Currentlayer - 1);          //��ǰnode�������
	double parm_TexDestructionRate = 0.2;                                                     //������ʣ���=0.2
	double TexDestructionRate = pow(parm_TexDestructionRate, n_Levels - Currentlayer - 1);    //��ǰnode�������
	if (node->nChildCount == 0) {
		DestructionRate = 1;
		TexDestructionRate = 1;
	}

	/*��ǰ��μ򻯽����
	����1��DOM model����
	����2��Ԫ��״ָ̬��Ԫ�ؼ��ϵ�ӳ�䣻
	����3��Ԫ��״ָ̬�룻
	����4����ЧԪ�ر�ʶ��
	����5�����ʣ�
	����6��ģ�����·����
	�򻯺����������ռ�µķ�Χ������������ת��*/
	Simplification part_sim;
	osg::Geode * leaf = part_sim.Simplify_lod(DOM_model, groupElementStateMap, groupElementStateMap_itr, node->pEleIndex, DestructionRate, TexDestructionRate, Partfolder);

	if (node->nChildCount == 0) { //��ǰ��λ�����һ��
		osgDB::writeNodeFile(*leaf, Partfolder + "/" + lodDomName + "_L" + std::to_string(Currentlayer) + "_" + ParentCode + to_string(region) + "_R" + to_string(region) + "_" + to_string(CurrentPartID) + ".osgb");
		return true;
	}

	/*---------------------------------------------------------------*/
	//������ǰ��ε�lod
	/* һ��������������޻��֣�
	UL(2)   |    UR(3)
  ----------|-----------
	LL(0)   |    LR(1)*/
	osg::ref_ptr<osg::Group> group = new osg::Group;
	int index = 0;
	auto bs = leaf->getBound();
	auto c = bs.center();
	auto r = bs.radius();
	for (int i = 0; i < 4; i++) {
		if (node->children[i] != NULL) {
			osg::ref_ptr<osg::PagedLOD> lod = new osg::PagedLOD();
			lod->setCenter(c);
			lod->setRadius(r);
			lod->setRangeMode(osg::LOD::RangeMode::PIXEL_SIZE_ON_SCREEN);
			/*osg::ref_ptr<osg::Geode> geode = new osg::Geode;
			geode->getOrCreateStateSet();
			lod->addChild(geode.get());*/
			lod->addChild(leaf);

			std::string relativeFilePath = "./" + lodDomName + "_L" + to_string(Currentlayer + 1) + "_" + ParentCode + to_string(region) + to_string(i) + "_R" + to_string(i) + "_" + to_string(CurrentPartID) + ".osgb";//�ӽڵ����·��
			lod->setFileName(0, "");
			lod->setFileName(1, relativeFilePath);

			QString Texname = QString(QString::fromLocal8Bit(Tex_absolute_path.c_str()));//ͼƬ����
			QFileInfo fileInfo = QFileInfo(Texname);
			QString TexName = fileInfo.fileName();
			string texname = string((const char *)TexName.toLocal8Bit());
			cv::Mat original = cv::imread(Partfolder + "/Texture/" + texname, 1);
			int s = original.cols > original.rows ? original.cols : original.rows;//��
			double GSD = (2 * r) / s;
			double Pixelsize_max = (2 * r) / GSD;
			double Minimum = 0;//��Ļ�ռ����
			double Maximum = Pixelsize_max;


			lod->setRange(0, Minimum, Maximum);//��ǰ�ڵ���Ļ�ֱ�������
			lod->setRange(1, Maximum, FLT_MAX);

			//lod->setDatabasePath("");

			group->addChild(lod);
		}
	}

	/*---------------------------------------------------------------*/
	string file = Partfolder + "/" + lodDomName + "_L" + std::to_string(Currentlayer) + "_" + ParentCode + to_string(region) + "_R" + to_string(region) + "_" + to_string(CurrentPartID) + ".osgb";
	osgDB::writeNodeFile(*group, file);//����

	ParentCode = ParentCode + to_string(region);  //���½ڵ��ʶ��

	getLayerLod(Partfolder, node->children[0], n_Levels, ParentCode, 0);//�ݹ�
	getLayerLod(Partfolder, node->children[1], n_Levels, ParentCode, 1);//�ݹ�
	getLayerLod(Partfolder, node->children[2], n_Levels, ParentCode, 2);//�ݹ�
	getLayerLod(Partfolder, node->children[3], n_Levels, ParentCode, 3);//�ݹ�

	return true;
	/*----------------------*/
}
