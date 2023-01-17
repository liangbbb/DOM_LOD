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
	inputDOMfile = inputFile;//原始模型路径
	outputDOMLodfile = outputFile;//LOD模型另存路径
}

DOMLod::~DOMLod()
{
}


bool DOMLod::buildDOMLod_Quadtree()
{
	/*文件有效性检测*/
	QFile DOMfile(QString::fromLocal8Bit(inputDOMfile.data()));
	if (!DOMfile.exists())return false;

	/*另存lod模型名称*/
	lodDomName = outputDOMLodfile.substr(outputDOMLodfile.find_last_of('/') + 1, outputDOMLodfile.find_last_of('.') - outputDOMLodfile.find_last_of('/') - 1);//模型名称;

	/*创建LOD总文件夹*/
	outputDOMLodFolder = outputDOMLodfile.substr(0, outputDOMLodfile.find_last_of('/'));
	std::string LODFile = outputDOMLodfile.substr(0, outputDOMLodfile.find_last_of('/') + 1) + lodDomName;
	QDir dir;
	dir.mkdir(QString::fromLocal8Bit(LODFile.data()));//创建文件夹
	/*创建Data模型总文件夹*/
	std::string DataFile = LODFile + "/Data";
	QDir dir1;
	dir1.mkdir(QString::fromLocal8Bit(DataFile.data()));//创建文件夹

	/*获取模型myobj::Model对象*/
	DOM_model = readOBJToModel(inputDOMfile);

	/*遍历ElementStateMap elementStateMap,模型分组构建LOD*/
	groupElementStateMap = DOM_model.elementStateMap;
	groupElementStateMap_itr = groupElementStateMap.begin();
	int part_index = 0;//模块标识
	while (QuadtreeInitial(groupElementStateMap, groupElementStateMap_itr))
		//步骤1：model分组初始化，确定element的分区，构建四叉树
	{
		CurrentPartID = part_index; //模块标识赋值
		//步骤2：该模块模型分区分层简化建立LOD模型
		/*if (CurrentPartID < 9) {
			groupElementStateMap_itr++;
			part_index++;
			continue;
		}*/
		PartLod();

		groupElementStateMap_itr++;
		part_index++;
	}

	/*建立总结点*/
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

		std::string relativeFilePath = "Data/" + lodDomName + "_L0_0_R0_" + to_string(i) + "/" + lodDomName + "_L0_0_R0_" + to_string(i) + ".osgb"; //相对路径

		lod->setFileName(0, "");
		lod->setFileName(1, relativeFilePath);

		lod->setRange(0, 0, 1.0);			//第一层不可见
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
		/*获取该组状态及对应的元素集合*/
		myobj::ElementState tem_currentElementState = elementStateMap_index_itr->first;//currentElementState
		myobj::Model::ElementList tem_currentElementList = elementStateMap_index_itr->second;//currentElementList
		int ElementCount = tem_currentElementList.size();//Number of triangular surfaces

		/*获取该组纹理图片相对路径与绝对路径*/
		myobj::Material tem_currentMaterial = DOM_model.materialMap[tem_currentElementState.materialName];//材质标识tem_currentElementState.materialName;
		for (int i = 0; i < tem_currentMaterial.maps.size(); ++i)
		{
			Tex_relative_path = tem_currentMaterial.maps[i].name;//相对路径
			Tex_absolute_path = DOM_model.databasePath + "/" + Tex_relative_path;//绝对路径
			break;
		}

		/*构建四叉树*/
		QuadDOMTree tem_DomTree;
		int *modular_Eleindex = new int[ElementCount];                           //该模块内元素标识集合
		myobj::Element *modular_Element = new myobj::Element[ElementCount];      //该模块内元素集合
		osg::Vec2 *texcoord = new osg::Vec2[DOM_model.texcoords.size()];         //模型总体纹理坐标集合
		for (int i = 0; i < ElementCount; i++) {
			modular_Element[i] = *tem_currentElementList[i];
			modular_Eleindex[i] = i;
		}
		for (int j = 0; j < DOM_model.texcoords.size(); j++) {
			texcoord[j] = DOM_model.texcoords[j];
		}
		tem_DomTree.OBJQuadTree(_Umin, _Umax, _Vmin, _Vmax, modular_Element, modular_Eleindex, texcoord, ElementCount);//该模块四叉树
		DomTrees.append(tem_DomTree);//四叉树入队
		return true;
	}
	return false;
}

/*
1.创建该模块文件夹
2.获取该模块四叉树根节点
3.递归构建每层lod
*/
//999
bool DOMLod::PartLod()
{
	//步骤1：创建该模块文件夹
	string Partfolder = outputDOMLodFolder + "/" + lodDomName + "/Data/" + lodDomName + "_L0_0_R0_" + to_string(CurrentPartID);
	QDir dir;
	dir.mkdir(QString::fromLocal8Bit(Partfolder.data()));
	//步骤2：获取QList中存放的最后一个模块的四叉树根节点和总层数
	QuadDOMNode * Root = DomTrees.at(DomTrees.size() - 1).m_Root;
	int n_Levels = DomTrees.at(DomTrees.size() - 1).m_Levels;
	//步骤3：递归构建某一模块LOD
	getLayerLod(Partfolder, Root, n_Levels, "", 0);
	return true;
}

bool DOMLod::getLayerLod(string Partfolder, QuadDOMNode * node, int n_Levels, string ParentCode, int region)
{
	if (node == NULL)return false;
	double parm_DestructionRate = 0.8;                                                        //网格简化率：θ=0.8
	int Currentlayer = node->nLevel;                                                          //当前node所属层次
	double DestructionRate = pow(parm_DestructionRate, n_Levels - Currentlayer - 1);          //当前node网格简化率
	double parm_TexDestructionRate = 0.2;                                                     //纹理简化率：α=0.2
	double TexDestructionRate = pow(parm_TexDestructionRate, n_Levels - Currentlayer - 1);    //当前node纹理简化率
	if (node->nChildCount == 0) {
		DestructionRate = 1;
		TexDestructionRate = 1;
	}

	/*当前层次简化结果（
	参数1：DOM model对象；
	参数2：元素状态指向元素集合的映射；
	参数3：元素状态指针；
	参数4：有效元素标识；
	参数5：简化率；
	参数6：模型另存路径）
	简化后计算纹理所占新的范围，并进行坐标转换*/
	Simplification part_sim;
	osg::Geode * leaf = part_sim.Simplify_lod(DOM_model, groupElementStateMap, groupElementStateMap_itr, node->pEleIndex, DestructionRate, TexDestructionRate, Partfolder);

	if (node->nChildCount == 0) { //当前层位于最后一层
		osgDB::writeNodeFile(*leaf, Partfolder + "/" + lodDomName + "_L" + std::to_string(Currentlayer) + "_" + ParentCode + to_string(region) + "_R" + to_string(region) + "_" + to_string(CurrentPartID) + ".osgb");
		return true;
	}

	/*---------------------------------------------------------------*/
	//构建当前层次的lod
	/* 一个矩形区域的象限划分：
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

			std::string relativeFilePath = "./" + lodDomName + "_L" + to_string(Currentlayer + 1) + "_" + ParentCode + to_string(region) + to_string(i) + "_R" + to_string(i) + "_" + to_string(CurrentPartID) + ".osgb";//子节点相对路径
			lod->setFileName(0, "");
			lod->setFileName(1, relativeFilePath);

			QString Texname = QString(QString::fromLocal8Bit(Tex_absolute_path.c_str()));//图片名称
			QFileInfo fileInfo = QFileInfo(Texname);
			QString TexName = fileInfo.fileName();
			string texname = string((const char *)TexName.toLocal8Bit());
			cv::Mat original = cv::imread(Partfolder + "/Texture/" + texname, 1);
			int s = original.cols > original.rows ? original.cols : original.rows;//宽
			double GSD = (2 * r) / s;
			double Pixelsize_max = (2 * r) / GSD;
			double Minimum = 0;//屏幕空间误差
			double Maximum = Pixelsize_max;


			lod->setRange(0, Minimum, Maximum);//当前节点屏幕分辨率设置
			lod->setRange(1, Maximum, FLT_MAX);

			//lod->setDatabasePath("");

			group->addChild(lod);
		}
	}

	/*---------------------------------------------------------------*/
	string file = Partfolder + "/" + lodDomName + "_L" + std::to_string(Currentlayer) + "_" + ParentCode + to_string(region) + "_R" + to_string(region) + "_" + to_string(CurrentPartID) + ".osgb";
	osgDB::writeNodeFile(*group, file);//保存

	ParentCode = ParentCode + to_string(region);  //更新节点标识符

	getLayerLod(Partfolder, node->children[0], n_Levels, ParentCode, 0);//递归
	getLayerLod(Partfolder, node->children[1], n_Levels, ParentCode, 1);//递归
	getLayerLod(Partfolder, node->children[2], n_Levels, ParentCode, 2);//递归
	getLayerLod(Partfolder, node->children[3], n_Levels, ParentCode, 3);//递归

	return true;
	/*----------------------*/
}
