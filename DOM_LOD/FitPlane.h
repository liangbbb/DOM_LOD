#pragma once
#include <osg/Vec3f>
#include <osg/Geometry>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core.hpp" //必须引此头文件    


#include <QString>

using namespace std;

class PlaneProject
{
public:
	PlaneProject();
	PlaneProject(QString polFileName);
	~PlaneProject();

	void cvFitPlane(const CvMat* points, float* plane);//最小二乘法拟合平面
	bool Project2Plane(osg::ref_ptr<osg::Vec3Array> Polygon, float m_plane[4], osg::ref_ptr<osg::Vec3Array>ProjectPolygon);//向趋势面上投影

	bool CoordinateRotation(osg::ref_ptr<osg::Vec3Array> ProjectPolygon, osg::ref_ptr<osg::Vec3Array> RotationPolygon);//三维坐标系旋转求面积

	bool computeXYZ(float v1[3], float v2[3], float normal[3]);//求旋轉後左邊軸的三個標量, 然后求旋转矩阵

	osg::Vec3f GetXYZ(osg::Vec3f& v);//旋转到局部坐标系

	bool CalculateArea(osg::ref_ptr<osg::Vec3Array> RotationPolygon, double &area);//计算多边形面积
	bool Calculateperimeter(osg::ref_ptr<osg::Vec3Array> RotationPolygon, double &perimeter);//计算多边形周长

private:
	QString m_FileName;//polFileName

	osg::Matrix R, R_i;//R为旋转矩阵，R_i为旋转矩阵的逆，A 与仿射变换矩阵【2,3】

	static  void ComputeNormal(float v1[3], float v2[3], float v3[3], double n[3]);//计算法向量
	static  void ComputeNormalDirection(float v1[3], float v2[3], float v3[3], double n[3]);//计算法向量方向

};
