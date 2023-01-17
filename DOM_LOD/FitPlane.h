#pragma once
#include <osg/Vec3f>
#include <osg/Geometry>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core.hpp" //��������ͷ�ļ�    


#include <QString>

using namespace std;

class PlaneProject
{
public:
	PlaneProject();
	PlaneProject(QString polFileName);
	~PlaneProject();

	void cvFitPlane(const CvMat* points, float* plane);//��С���˷����ƽ��
	bool Project2Plane(osg::ref_ptr<osg::Vec3Array> Polygon, float m_plane[4], osg::ref_ptr<osg::Vec3Array>ProjectPolygon);//����������ͶӰ

	bool CoordinateRotation(osg::ref_ptr<osg::Vec3Array> ProjectPolygon, osg::ref_ptr<osg::Vec3Array> RotationPolygon);//��ά����ϵ��ת�����

	bool computeXYZ(float v1[3], float v2[3], float normal[3]);//�����D����߅�S����������, Ȼ������ת����

	osg::Vec3f GetXYZ(osg::Vec3f& v);//��ת���ֲ�����ϵ

	bool CalculateArea(osg::ref_ptr<osg::Vec3Array> RotationPolygon, double &area);//�����������
	bool Calculateperimeter(osg::ref_ptr<osg::Vec3Array> RotationPolygon, double &perimeter);//���������ܳ�

private:
	QString m_FileName;//polFileName

	osg::Matrix R, R_i;//RΪ��ת����R_iΪ��ת������棬A �����任����2,3��

	static  void ComputeNormal(float v1[3], float v2[3], float v3[3], double n[3]);//���㷨����
	static  void ComputeNormalDirection(float v1[3], float v2[3], float v3[3], double n[3]);//���㷨��������

};
