#include "Simplification.h"
#include <fstream>
#include <iostream>
#include "Point.h"
#include <set>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include "Face.h"
#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/WriteFile>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <QProgressDialog>//进度条
#include <osg/CullFace>
#include <string>
#include <osgUtil/SmoothingVisitor>//法线

#include"DOMLod.h"
#include <osg/LOD>
#include "CU.h"
#include <qdir.h>


using namespace std;
myobj::Model Simplification::readOBJToModel(string inputDOMfile)
{
	osgDB::ifstream fin(inputDOMfile.c_str());
	if (fin)
	{
		osg::ref_ptr<osgDB::ReaderWriter::Options> local_opt = new osgDB::ReaderWriter::Options;
		local_opt->getDatabasePathList().push_front(osgDB::getFilePath(inputFile));
		//OBJ convert to model object
		myobj::Model model;
		model.setDatabasePath(osgDB::getFilePath(inputFile.c_str()));
		model.readOBJ(fin, local_opt.get());
		return model;
	}
}

bool Simplification::ModGroInitial(myobj::Model model, std::map<myobj::ElementState, myobj::Model::ElementList>& elementStateMap, std::map<myobj::ElementState, myobj::Model::ElementList>::iterator & elementStateMap_index_itr, vector<int> Effective_element)
{
	if (elementStateMap_index_itr != elementStateMap.end())
	{
		/*获取该组状态及对应的元素集合*/
		myobj::ElementState tem_currentElementState = elementStateMap_index_itr->first;//currentElementState
		myobj::Model::ElementList tem_currentElementList = elementStateMap_index_itr->second;//currentElementList
		faceCount = tem_currentElementList.size();//Number of triangular surfaces

		/*获取该组纹理图片相对路径与绝对路径*/
		myobj::Material tem_currentMaterial = model.materialMap[tem_currentElementState.materialName];//材质标识tem_currentElementState.materialName;
		for (int i = 0; i < tem_currentMaterial.maps.size(); ++i)
		{
			Tex_relative_path = tem_currentMaterial.maps[i].name;//相对路径
			Tex_absolute_path = model.databasePath + "/" + Tex_relative_path;//绝对路径
			break;
		}

		/*重置pointSet,重置pairHeap*/
		updatepointSet();
		updatepairHeap();

		/*获取顶点构建SPoint存入pointSet*/
		for (myobj::Model::Vec3Array::iterator v_index_itr = model.vertices.begin();
			v_index_itr != model.vertices.end();
			++v_index_itr)
		{
			osg::Vec3 tem_vertice = *v_index_itr;
			double x = tem_vertice.x();
			double y = tem_vertice.y();
			double z = tem_vertice.z();
			SPoint newPoint;
			newPoint.Point_Vector[0] = x;
			newPoint.Point_Vector[1] = y;
			newPoint.Point_Vector[2] = z;
			pointSet.addPoint(newPoint);
		}

		/*获取纹理点构建SPoint_UV存入pointSet*/
		for (myobj::Model::Vec2Array::iterator t_index_itr = model.texcoords.begin();
			t_index_itr != model.texcoords.end();
			++t_index_itr)
		{
			osg::Vec2 tem_texcoord = *t_index_itr;
			double u = tem_texcoord.x();
			double v = tem_texcoord.y();
			SPoint_UV newPoint_UV;
			newPoint_UV.PointUV_Vector[0] = u;
			newPoint_UV.PointUV_Vector[1] = v;
			pointSet.addPoint_uv(newPoint_UV);
		}

		/*筛选三角面建立临边关系*/
		for (int i = 0; i < Effective_element.size(); ++i) {
			int element_index = Effective_element[i];//有效元素标识
			myobj::Element element = *tem_currentElementList[element_index];//有效元素

			pointSet.point.at(element.vertexIndices[0]).SPointUV_index.insert(element.texCoordIndices[0]);//添加对应纹理坐标点标识
			pointSet.point.at(element.vertexIndices[1]).SPointUV_index.insert(element.texCoordIndices[1]);//添加对应纹理坐标点标识
			pointSet.point.at(element.vertexIndices[2]).SPointUV_index.insert(element.texCoordIndices[2]);//添加对应纹理坐标点标识

			for (int i = 0; i < 3; ++i) {
				for (int j = i + 1; j < 3; ++j) {
					if (!pointSet.point.at(element.vertexIndices[i]).hasNeighbor(element.vertexIndices[j]))
					{
						pointSet.point.at(element.vertexIndices[i]).addNeighbor(element.vertexIndices[j]);
						pointSet.point.at(element.vertexIndices[j]).addNeighbor(element.vertexIndices[i]);
					}
					if (!pointSet.point_uv.at(element.texCoordIndices[i]).hasNeighbor(element.texCoordIndices[j]))
					{
						pointSet.point_uv.at(element.texCoordIndices[i]).addNeighbor(element.texCoordIndices[j]);
						pointSet.point_uv.at(element.texCoordIndices[j]).addNeighbor(element.texCoordIndices[i]);
					}
				}
			}
		}
		return true;
	}
	return false;
}

void Simplification::updateData_UVPoint(Pair toDel)
{
	SPoint_UV newPoint_UV;//新纹理点
	newPoint_UV.PointUV_Vector[0] = toDel.bestPoint.PointUV_Vector[0];
	newPoint_UV.PointUV_Vector[1] = toDel.bestPoint.PointUV_Vector[1];
	int np = pointSet.addPoint_uv(newPoint_UV);//加入新纹理点，返回点的标识

	int v1_num_uv = pointSet.point.at(toDel.v1).SPointUV_index.size();//边v1点纹理个数
	int v2_num_uv = pointSet.point.at(toDel.v2).SPointUV_index.size();//边v2点纹理个数

	set<int> UVPoint_v1v2;//建立v1,v2的纹理点集
	set<int> neighbors;//建立v1,v2的纹理点邻居集容器
	for (set<int>::iterator it = pointSet.point.at(toDel.v1).SPointUV_index.begin(); it != pointSet.point.at(toDel.v1).SPointUV_index.end(); it++)
	{
		cout << *it << " occurs " << endl;
		UVPoint_v1v2.insert(*it);//V1点的纹理点
	}
	for (set<int>::iterator it = pointSet.point.at(toDel.v2).SPointUV_index.begin(); it != pointSet.point.at(toDel.v2).SPointUV_index.end(); it++)
	{
		UVPoint_v1v2.insert(*it);//V2点的纹理点
	}

	for (set<int>::iterator it = UVPoint_v1v2.begin(); it != UVPoint_v1v2.end(); it++)//遍历v1,v2的纹理点集
	{
		cout << *it << " occurs " << endl;
		for (int i = 0; i < (int)pointSet.point_uv.at(*it).neighbor.size(); ++i)//遍历v1,v2点纹理点的一环领域所有点
		{
			int k = pointSet.point_uv.at(*it).neighbor[i];//k为一环邻域纹理点标识
			if (UVPoint_v1v2.count(k) == 0)
				neighbors.insert(k);//除了v1v2的纹理点外，v1v2点纹理点的所有一环邻域点标识
		}
	}

	set<int>::iterator nb;
	for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)
	{
		pointSet.point_uv.at(np).neighbor.push_back(*nb);
	}//更新新纹理点的邻居集

	for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)
	{
		vector<int>::iterator iter;
		for (iter = pointSet.point_uv.at(*nb).neighbor.begin(); iter != pointSet.point_uv.at(*nb).neighbor.end();)
		{
			if (UVPoint_v1v2.count(*iter) == 1) {
				iter = pointSet.point_uv.at(*nb).neighbor.erase(iter);
			}
			else
			{
				++iter;
			}
		}
		pointSet.point_uv.at(*nb).addNeighbor(np);
	}//对邻居集的每一个纹理点删除原先邻居点集合中的的点v1,v2的纹理点，并加入新纹理点到邻居集中
}

void Simplification::updateData_Point(Pair toDel)
{
	pairHeap.deletePair(&toDel);//删除此边

	toDel.bestPoint.SPointUV_index.insert(pointSet.maxpos_uv - 1);//最佳折叠点的赋值纹理坐标标识
	int np = pointSet.addPoint(toDel.bestPoint);//加入新点，返回点的标识

	set<int> neighbors;//建立v1,v2总的邻居集容器
	for (int i = 0; i < (int)pointSet.point.at(toDel.v1).neighbor.size(); ++i)//遍历边对v1点的一环领域所有点
	{
		int k = pointSet.point.at(toDel.v1).neighbor[i];
		if (k != toDel.v2)
			neighbors.insert(k);//除了v2外，v1点所有一环邻域点标识
	}
	for (int i = 0; i < (int)pointSet.point.at(toDel.v2).neighbor.size(); ++i)//遍历边对v2点的一环领域所有点
	{
		int k = pointSet.point.at(toDel.v2).neighbor[i];
		if (k != toDel.v1)
			neighbors.insert(k);//除了v1外，v2点所有一环邻域点标识
	}

	set<int>::iterator nb;
	for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)
	{
		pointSet.point.at(np).neighbor.push_back(*nb);
	}//更新新点的邻居集

	for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)
	{
		vector<int>::iterator iter;
		for (iter = pointSet.point.at(*nb).neighbor.begin(); iter != pointSet.point.at(*nb).neighbor.end();)
		{
			if (*iter == toDel.v1 || *iter == toDel.v2)
				iter = pointSet.point.at(*nb).neighbor.erase(iter);
			else
				++iter;
		}
		pointSet.point.at(*nb).addNeighbor(np);
	}//对邻居集的每一个点删除原先邻居点集合中的的点v1,v2，并加入新点到邻居集中

	for (nb = neighbors.begin(); nb != neighbors.end(); ++nb) {
		pointSet.point.at(*nb).calculateQEMMat(&pointSet);
		//pointSet.point.at(*nb).calculateD_QEMMat(&pointSet);
	}//更新邻居集中每一个点的误差矩阵

	pointSet.point.at(np).calculateQEMMat(&pointSet);//计算新点的误差矩阵
	//pointSet.point.at(np).calculateD_QEMMat(&pointSet);//计算新点的误差矩阵

	for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)//更新边对 
	{
		//隐藏旧的关联边
		Pair p1 = Pair(*nb, toDel.v1);
		Pair p2 = Pair(*nb, toDel.v2);
		p1.sort();
		p2.sort();
		map<pair<int, int>, bool>::iterator iter;
		iter = pairHeap.mapper.find(make_pair(p1.v1, p1.v2));
		if (iter != pairHeap.mapper.end())
		{
			if (pairHeap.mapper[make_pair(p1.v1, p1.v2)] == true)
				pairHeap.deletePair(&Pair(p1.v1, p1.v2));
		}
		iter = pairHeap.mapper.find(make_pair(p2.v1, p2.v2));
		if (iter != pairHeap.mapper.end())
		{
			if (pairHeap.mapper[make_pair(p2.v1, p2.v2)] == true)
				pairHeap.deletePair(&Pair(p2.v1, p2.v2));
		}

		//加入新的关联边
		Pair toAdd = Pair(*nb, np);
		toAdd.sort();

		bool Success;
		toAdd.calculateBestPointQEM(&pointSet, Success);//计算折叠后点的最佳位置和UV（有解就算，没解就取中点）
		toAdd.calculateDelCostQEM(&pointSet);//计算边的折叠代价

		////参数1：计算顶点尖锐度
		//double Sharpness1 = 0;
		//double Sharpness2 = 0;
		//toAdd.calculateVertexSharpness(&pointSet, toAdd.v1, Sharpness1);
		//toAdd.calculateVertexSharpness(&pointSet, toAdd.v2, Sharpness2);
		//double Vertex_sharpness = Sharpness1 + Sharpness2;
		//Vertex_sharpness = Vertex_sharpness / 3.14;
		////toAdd.delCost = toAdd.delCost *  Vertex_sharpness;

		////参数2
		//double Texture_complexity = toAdd.calculate_TexComplexity(&pointSet);
		//toAdd.delCost = toAdd.delCost * Vertex_sharpness * Texture_complexity;

		if (toAdd.calculateMaxNormalDeviation(&pointSet) >= 1) {
			double ppp = toAdd.delCost;
			toAdd.delCost = toAdd.delCost + 10000000;
		}//计算最大法线偏转

		pairHeap.addPair(&toAdd);
	}

	pointSet.delPoint(toDel.v1);//将点v1隐藏
	pointSet.point.at(toDel.v1).neighbor.clear();
	pointSet.delPoint(toDel.v2);//将点v2隐藏
	pointSet.point.at(toDel.v2).neighbor.clear();
}


bool Simplification::updatepointSet()
{
	pointSet.updata();//更新点操作对象
	return true;
}

bool Simplification::updatepairHeap()
{
	pairHeap.updata();//更新边操作对象
	return true;
}

Simplification::Simplification()
{
	inputFile = "";
	inputMTL = "";
	//outputFile = "";
	ratio = 0.5;
	pointCount = 0;
	pointUVCount = 0;
	faceCount = 0;
	ImageSample = true;
}

void Simplification::setMethod(int sim_method)
{
	Sim_Method = sim_method;
}

void Simplification::setInput(string _input)
{
	inputFile = _input;
}

void Simplification::setLODInput(string _input)
{
	inputFile = _input;
}

void Simplification::setOutput(string _output)
{
	outputFile = _output;
}

void Simplification::setLODOutput(string _output)
{
	outputFile = _output;
}

void Simplification::setRatio(double _ratio)
{
	ratio = _ratio;
}

void Simplification::setImageSample(bool sample)
{
	ImageSample = sample;
}


//999
osg::Geode * Simplification::Simplify_lod(myobj::Model DOM_model, std::map<myobj::ElementState, myobj::Model::ElementList>& elementStateMap, std::map<myobj::ElementState, myobj::Model::ElementList>::iterator & elementStateMap_index_itr, QList<int*>* pEleIndex, double DestructRate, double TexDestructRate, string Partfolder)
{
	outputfolder = Partfolder;  //文件夹路径
	ratio = DestructRate;       //简化率
	Tex_ratio = TexDestructRate;//纹理简化率


	//步骤1：读取模块范围内的元素标识，装入Effective_element
	vector<int> Effective_element;
	for (int k = 0; k < pEleIndex->size(); k++) {
		int *index = pEleIndex->at(k);
		Effective_element.push_back(*index);
	}
	//步骤2：读取模块model对象，并筛选三角形
	ModGroInitial(DOM_model, elementStateMap, elementStateMap_index_itr, Effective_element);
	//步骤3：计算点的高维二次误差矩阵
	pointSet.calculateQEMUVMat();
	//步骤4：设置边对
	pairHeap.setupHeapQEM(&pointSet);
	//步骤5：模型简化
	int nowCount = faceCount;//三角面个数
	int EndFaceCount = faceCount * DestructRate;//目标三角面个数
	while (nowCount > EndFaceCount)
	{
		Pair toDel = pairHeap.top(&pointSet);
		/*if (toDel.delCost >= 1000000000000) {
			break;
		}*/
		if (toDel.v1 == -1 || toDel.v2 == -1) {
			break;
		}
		updateData_UVPoint(toDel); //更新纹理坐标点数据集
		updateData_Point(toDel);   //删除顶部边对并更新数据集
		nowCount -= 2;
	}
	//步骤6：抽稀模型纹理图像
	lodImagethin();

	//步骤7：创建模型节点
	osg::Geode* leaf = LODCreateLeaf();//创建简化模型节点(纹理切割)
	osg::ref_ptr<osg::Texture2D> texture;
	osg::ref_ptr<osg::Image> image;
	texture = new osg::Texture2D;
	image = osgDB::readImageFile(outputTextureFile);//最新纹理图像（抽稀切割后新图像/原始图像）
	texture->setResizeNonPowerOfTwoHint(false);
	texture->setUnRefImageDataAfterApply(true);
	texture->setNumMipmapLevels(3);
	texture->setImage(image.get());
	texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
	if (texture->getImage() != 0L) {
		leaf->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture);
		leaf->getOrCreateStateSet()->setTextureAttribute(0, new osg::TexEnv(osg::TexEnv::DECAL));
	}
	//osg::ref_ptr<osg::CullFace> cullface = new osg::CullFace(osg::CullFace::BACK);//背面剔除
	//leaf->getOrCreateStateSet()->setAttributeAndModes(cullface, osg::StateAttribute::ON);
	return leaf;
}


bool Simplification::split(char * strLine, char **& res)
{
	char *p = NULL;
	int count = 0;
	p = strtok(strLine, " ");

	while (p)
	{
		res[count] = p;
		p = strtok(NULL, " ");
		count++;
	}
	return true;
}

bool Simplification::split1(char * strLine, char **& res)
{
	char *p = NULL;
	int count = 0;
	p = strtok(strLine, "/");

	while (p)
	{
		res[count] = p;
		p = strtok(NULL, "/");
		count++;
	}
	return true;
}

ifstream & Simplification::seek_to_line(ifstream & in, int line)
{

	int i;
	char buf[1024];
	in.seekg(0, ios::beg);  //定位到文件开始。
	for (i = 0; i < line; i++)
	{
		in.getline(buf, sizeof(buf));//读取行。
	}
	return in;

}

void Simplification::computeNormal(double v1[3], double v2[3], double v3[3], double n[3])
{
	double length;
	n[0] = n[2] = 0; n[1] = 1.0;
	computeNormalDirection(v1, v2, v3, n);
	length = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
	if (length > 0.0) {
		n[0] /= length;
		n[1] /= length;
		n[2] /= length;
	}
}

void Simplification::computeNormalDirection(double v1[3], double v2[3], double v3[3], double n[3])
{
	double ax, ay, az, bx, by, bz;
	// order is important!!! maintain consistency with triangle vertex order
	ax = (double)v3[0] - v2[0];
	ay = (double)v3[1] - v2[1];
	az = (double)v3[2] - v2[2];
	bx = (double)v1[0] - v2[0];
	by = (double)v1[1] - v2[1];
	bz = (double)v1[2] - v2[2];
	n[0] = (double)(ay * bz - az * by);
	n[1] = (double)(az * bx - ax * bz);
	n[2] = (double)(ax * by - ay * bx);
}

bool Simplification::judgeTirSame(QList<int>& Tir_index, int Tir)
{
	for (int m = 0; m != Tir_index.size(); ++m)
	{
		int Tirs = Tir_index.at(m);
		if (Tirs == Tir)
		{
			return true;
		}
	}
	return false;
}

bool Simplification::IsInBox(vector<int> pointindex, int index[3])
{
	int pointcount = 0;
	for (int i = 0; i < 3; i++) {
		if (std::find(pointindex.begin(), pointindex.end(), index[i]) != pointindex.end()) {
			pointcount++;
		}
	}
	if (pointcount >= 1) {
		return true;
	}
	return false;
}

bool Simplification::lodImagethin()
{
	/*判断纹理文件夹是否存在*/
	std::string Texfolder = outputfolder + "/Texture";
	QDir dir(QString::fromLocal8Bit(Texfolder.data()));
	if (!dir.exists()) {
		QDir dir1;
		dir1.mkdir(QString::fromLocal8Bit(Texfolder.data()));//创建文件夹
	}

	QString Texname = QString(QString::fromLocal8Bit(Tex_absolute_path.c_str()));//图片名称
	QFileInfo fileInfo = QFileInfo(Texname);
	QString fileName = fileInfo.fileName();
	outputTextureFile = Texfolder + "/" + std::string((const char *)fileName.toLocal8Bit().constData());//设置纹理图片另存路径
	cv::Mat original = cv::imread(Tex_absolute_path, 1);

	double scale = Tex_ratio;//缩放
	/*while (original.cols*scale < 1 || original.rows*scale < 1) {
		scale = scale * 5;
	}*/
	cv::Size dsize;
	if (original.cols*scale < 10 || original.rows*scale < 10) {
		dsize = cv::Size(10, 10);
	}
	else
	{
		dsize = cv::Size(original.cols*scale, original.rows*scale);
	}
	cv::Mat thin;
	cv::resize(original, thin, dsize);
	cv::imwrite(outputTextureFile, thin);
	return true;
}

osg::Geode * Simplification::LODCreateLeaf()
{
	osg::ref_ptr<osg::Geode> leaf(new osg::Geode);
	//osg::ref_ptr<osg::Geometry> geometry(new osg::Geometry);
	deprecated_osg::Geometry* geometry = new deprecated_osg::Geometry;
	osg::Vec3Array * vertexArray = new osg::Vec3Array;       //顶点集合
	osg::UByteArray* vecindex = new osg::UByteArray;         //顶点索引数组
	osg::Vec2Array * textureCoord = new osg::Vec2Array;      //顶点纹理坐标数组
	//osg::ref_ptr < osg::Vec2Array >textureCoord = new osg::Vec2Array;//顶点纹理坐标数组
	osg::UByteArray* texindex = new osg::UByteArray;                 //顶点纹理索引数组
	osg::ref_ptr < osg::Vec3Array >normals = new osg::Vec3Array;     //法线坐标数组
	osg::UByteArray* norindex = new osg::UByteArray;                 //法线索引数组

	vector<int> pointOut;//输出点集
	map<int, int> outMapper;//点集映射
	int pos = 0;
	for (int outCount = 0; outCount < pointSet.count;)//建立点标识与现在点个数之间的索引
	{
		if (pointSet.enabled[pos])//如果是可用点则加入输出点集
		{
			pointOut.push_back(pos);
			outMapper[pos] = outCount;//建立逆映射
			outCount++;
		}
		pos++;
	}

	struct cmp
	{
		bool operator ()(Face f1, Face f2)
		{
			if (f1.p[0] < f2.p[0]) return true;
			if (f1.p[0] > f2.p[0]) return false;
			if (f1.p[1] < f2.p[1]) return true;
			if (f1.p[1] > f2.p[1]) return false;
			if (f1.p[2] < f2.p[2]) return true;
			return false;
		}
	};

	set<Face, cmp> faceOut;
	for (int i = 0; i < (int)pointOut.size(); i++)//遍历每一个有效点
	{
		int insize = (int)pointSet.point.at(pointOut[i]).neighbor.size();
		for (int v1 = 0; v1 < insize; v1++)//遍历每一个有效点的一领域点
		{
			for (int v2 = v1 + 1; v2 < insize; v2++)
			{
				if (pointSet.point.at(pointSet.point.at(pointOut[i]).neighbor[v1]).hasNeighbor(
					pointSet.point.at(pointOut[i]).neighbor[v2]))//判断是否组成一个三角面
				{
					SPoint p = pointSet.point.at(pointOut[i]);
					//面用现在所有有效点的个数表示
					Face toAdd(outMapper[pointOut[i]], outMapper[p.neighbor[v1]], outMapper[p.neighbor[v2]]);
					toAdd.pointSort();
					faceOut.insert(toAdd);//把面加入容器，不重复添加
				}
			}
		}
	}

	set<Face, cmp>::iterator iter;
	//遍历输出面
	double x;
	double y;
	double z;
	double u;
	double v;
	double u_max = 0.0;
	double u_min = 1.0;
	double v_max = 0.0;
	double v_min = 1.0;

	for (iter = faceOut.begin(); iter != faceOut.end(); ++iter)
	{
		int one = (*iter).p[0];
		int two = (*iter).p[1];
		int three = (*iter).p[2];

		double v1[3];//点1
		double v2[3];//点2
		double v3[3];//点3

		//添加顶点
		x = pointSet.point.at(pointOut[one]).Point_Vector[0];
		y = pointSet.point.at(pointOut[one]).Point_Vector[1];
		z = pointSet.point.at(pointOut[one]).Point_Vector[2];
		v1[0] = x;
		v1[1] = y;
		v1[2] = z;
		vertexArray->push_back(osg::Vec3d(x, y, z));

		x = pointSet.point.at(pointOut[two]).Point_Vector[0];
		y = pointSet.point.at(pointOut[two]).Point_Vector[1];
		z = pointSet.point.at(pointOut[two]).Point_Vector[2];
		v2[0] = x;
		v2[1] = y;
		v2[2] = z;
		vertexArray->push_back(osg::Vec3d(x, y, z));

		x = pointSet.point.at(pointOut[three]).Point_Vector[0];
		y = pointSet.point.at(pointOut[three]).Point_Vector[1];
		z = pointSet.point.at(pointOut[three]).Point_Vector[2];
		v3[0] = x;
		v3[1] = y;
		v3[2] = z;
		vertexArray->push_back(osg::Vec3d(x, y, z));

		//计算法向量
		float normal[3];
		double dnormal[3];
		computeNormal(v1, v2, v3, &dnormal[0]);
		normal[0] = dnormal[0];
		normal[1] = dnormal[1];
		normal[2] = dnormal[2];
		normals->push_back(osg::Vec3(normal[0], normal[1], normal[2]));
		normals->push_back(osg::Vec3(normal[0], normal[1], normal[2]));
		normals->push_back(osg::Vec3(normal[0], normal[1], normal[2]));

		//添加纹理点
		bool Effective = false;
		for (set<int>::iterator it1 = pointSet.point.at(pointOut[one]).SPointUV_index.begin(); it1 != pointSet.point.at(pointOut[one]).SPointUV_index.end(); it1++)
		{
			if (Effective == false) {
				for (set<int>::iterator it2 = pointSet.point.at(pointOut[two]).SPointUV_index.begin(); it2 != pointSet.point.at(pointOut[two]).SPointUV_index.end(); it2++)
				{
					if (Effective == false) {
						for (set<int>::iterator it3 = pointSet.point.at(pointOut[three]).SPointUV_index.begin(); it3 != pointSet.point.at(pointOut[three]).SPointUV_index.end(); it3++)
						{
							bool a = pointSet.point_uv.at(*it1).hasNeighbor(*it2);
							bool a1 = pointSet.point_uv.at(*it2).hasNeighbor(*it1);
							bool b = pointSet.point_uv.at(*it2).hasNeighbor(*it3);
							bool b1 = pointSet.point_uv.at(*it3).hasNeighbor(*it2);
							bool c = pointSet.point_uv.at(*it1).hasNeighbor(*it3);
							bool c1 = pointSet.point_uv.at(*it3).hasNeighbor(*it1);
							if (a == true && a1 == true && b == true && b1 == true && c == true && c1 == true) {//构成三角形
								u = pointSet.point_uv.at(*it1).PointUV_Vector[0];
								v = pointSet.point_uv.at(*it1).PointUV_Vector[1];
								if (u < 0)u = 0;
								if (v < 0)v = 0;
								if (u > 1)u = 1;
								if (v > 1)v = 1;
								if (u > u_max)u_max = u;
								if (u < u_min)u_min = u;
								if (v > v_max)v_max = v;
								if (v < v_min)v_min = v;
								textureCoord->push_back(osg::Vec2d(u, v));

								u = pointSet.point_uv.at(*it2).PointUV_Vector[0];
								v = pointSet.point_uv.at(*it2).PointUV_Vector[1];
								if (u < 0)u = 0;
								if (v < 0)v = 0;
								if (u > 1)u = 1;
								if (v > 1)v = 1;
								if (u > u_max)u_max = u;
								if (u < u_min)u_min = u;
								if (v > v_max)v_max = v;
								if (v < v_min)v_min = v;
								textureCoord->push_back(osg::Vec2d(u, v));

								u = pointSet.point_uv.at(*it3).PointUV_Vector[0];
								v = pointSet.point_uv.at(*it3).PointUV_Vector[1];
								if (u < 0)u = 0;
								if (v < 0)v = 0;
								if (u > 1)u = 1;
								if (v > 1)v = 1;
								if (u > u_max)u_max = u;
								if (u < u_min)u_min = u;
								if (v > v_max)v_max = v;
								if (v < v_min)v_min = v;
								textureCoord->push_back(osg::Vec2d(u, v));
								Effective = true;
								break;
							}
						}
					}
				}
			}
		}

		if (Effective == false) {
			/*textureCoord->push_back(osg::Vec2d(1, 1));
			textureCoord->push_back(osg::Vec2d(1, 1));
			textureCoord->push_back(osg::Vec2d(1, 1));*/
			vertexArray->pop_back();
			vertexArray->pop_back();
			vertexArray->pop_back();
			normals->pop_back();
			normals->pop_back();
			normals->pop_back();
		}
	}

	//图像切割
	cv::Mat img;
	cv::Rect m_select;
	img = cv::imread(outputTextureFile);
	if (ratio == 1)img = cv::imread(Tex_absolute_path);
	int i_min = (1.0 - v_max)*img.rows - 1;//i从0到row-1
	int j_min = u_min * img.cols - 1;//j从0到col-1
	if (i_min < 0)i_min = 0;//误差控制
	if (j_min < 0)j_min = 0;//误差控制
	int width = (u_max - u_min)*img.cols;//宽
	if (width > img.cols - j_min)width = img.cols - j_min;//误差控制
	if (width < 2)width = 2;//误差控制
	int height = (v_max - v_min)*img.rows;//高
	if (height > img.rows - i_min)height = img.rows - i_min;//误差控制
	if (height < 2)height = 2;//误差控制

	m_select = cv::Rect(j_min, i_min, width, height);//左上角的列值（col），左上角的行值（row），矩形col方向上的宽，矩形row方向上的宽
	cv::Mat ROI = img(m_select);
	cv::imwrite(outputTextureFile, ROI);

	//纹理坐标转换
	u_min;//新原点
	v_min;
	//遍历纹理坐标数组并修改
	osg::Vec2Array * newtextureCoord = new osg::Vec2Array;      //新的顶点纹理坐标数组
	for (osg::Vec2Array::size_type i = 0; i < textureCoord->size(); ++i)
	{
		double u0 = textureCoord->at(i).x();
		double v0 = textureCoord->at(i).y();
		double u = (u0 - u_min) / (u_max - u_min);
		double v = (v0 - v_min) / (v_max - v_min);
		if (u < 0)u = 0;
		if (u > 1)u = 1;
		if (v < 0)v = 0;
		if (v > 1)v = 1;
		newtextureCoord->push_back(osg::Vec2d(u, v));
	}

	geometry->setVertexArray(vertexArray);
	normals.get()->setBinding(osg::Array::BIND_PER_PRIMITIVE_SET);//2017.10.25add
	geometry->setNormalArray(normals.get());
	geometry->setTexCoordArray(0, newtextureCoord);
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, vertexArray->size()));
	//osgUtil::SmoothingVisitor::smooth(*(geometry.get()));//生成法线
	leaf->addDrawable(geometry);
	return leaf.release();
}