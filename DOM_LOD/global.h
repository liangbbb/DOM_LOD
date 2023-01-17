#pragma once
#include <corecrt_malloc.h>
#include <QtCore/qstring.h>
#include <QtCore/qfileinfo.h>
#include <QColor.h>
#include <osg/Vec4f>
#include <osg/Vec3>
#include <osg/Vec2>

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

class gocColor {//色彩定义
public:
	osg::Vec4f Set(unsigned int rgba);
	void Set(QColor & qc);
	void Set(osg::Vec4f & oc);
	osg::Vec4f Get();
	QColor GetRGB();
	unsigned int GetUInt();
private:
	osg::Vec4f _color;
};

void gocSwapForBigEndian(unsigned char * theArray, int tupleSiz, int numTuple);
bool gocIsFileExist(QString csFile);
QString gocToString(unsigned int nValue);
QString gocToString(int nValue);
QString gocToString(double dValue);
QString gocToString(float fValue);
void gocNoExtension(QString & csIn);
bool gocEqual(float fV1, float fV2);
template <class T>
float gocPointToLine(const T &C, const T &A, const T &B) {
	T ab = B - A;// 直线AB方向向量
	T ac = C - A;
	T n1 = ac ^ ab;//ac叉乘ab，得到平面ABC的法向量
	T n2 = ab ^ n1;//ab叉乘n1，得到平行于平面ABC且垂直AB的向量
	n2.normalize();//单位化
				   //minDistance = ac*n2;//AC在n2方向上的投影
				   //nearestPoint = C - n2*minDistance;
	return ac * n2;
}

float gocdefPoint2Point(const osg::Vec3 & p1, const osg::Vec3 & p2);

float gocDistance(osg::Vec2 p1, osg::Vec2 p2);//两点距离
float gocDistance(osg::Vec3 p1, osg::Vec3 p2);
bool gocInline(osg::Vec3 p1, osg::Vec3 p2, osg::Vec3 p3);//是否三点共线
osg::Vec3 gocCross(osg::Vec3 u, osg::Vec3 v);//计算2点差积:UxV
double gocMode(osg::Vec3 p);//向量大小(模）
osg::Vec3 gocSub(osg::Vec3 u, osg::Vec3 v);//矢量差 U-V
