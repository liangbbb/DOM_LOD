#include "global.h"
#define eps 0.0000001
// Swap an array of (or a single) int, double, when
// loading little-endian / Win-based disk files on big-endian platforms.
void gocSwapForBigEndian(unsigned char * theArray, int tupleSiz, int numTuple)
{
	int      i, j;
	unsigned char * tmpChar0 = theArray;
	unsigned char * tmpChar1 = (unsigned char *)malloc(tupleSiz);
	for (j = 0; j < numTuple; j++, tmpChar0 += tupleSiz)
	{
		for (i = 0; i < tupleSiz; i++)
		{
			tmpChar1[i] = tmpChar0[tupleSiz - 1 - i];
		}
		for (i = 0; i < tupleSiz; i++)
		{
			tmpChar0[i] = tmpChar1[i];
		}
	}
	tmpChar0 = NULL;
	free(tmpChar1);
	tmpChar1 = NULL;
}

bool gocIsFileExist(QString csFile)
{
	QFileInfo info(csFile);
	return info.exists();
}

QString gocToString(unsigned int nValue)
{
	QString csValue;
	csValue.setNum(nValue);
	return csValue;
}

QString gocToString(int nValue)
{
	QString csValue;
	csValue.setNum(nValue);
	return csValue;
}

QString gocToString(double dValue)
{
	QString csValue; //= QString("%0.3f").arg(dValue);
	csValue.setNum(dValue, 'g', 6);
	return csValue;
}

QString gocToString(float fValue) {
	QString csValue; //= QString("%0.3f").arg(dValue);
	csValue.setNum(fValue, 'g', 4);
	return csValue;
}

void gocNoExtension(QString & csIn)
{
	int index = csIn.lastIndexOf(".");
	if (index >= 0)csIn.truncate(index);
}

bool gocEqual(float fV1, float fV2) {
	float fV = fabsf(fV1 - fV2);
	if (fV < 0.000001)
		return true;
	return false;
}

float gocdefPoint2Point(const osg::Vec3 & p1, const osg::Vec3 & p2) {
	return float(sqrt((p1.x() - p2.x())*(p1.x() - p2.x()) + (p1.y() - p2.y())*(p1.y() - p2.y()) + (p1.z() - p2.z())*(p1.z() - p2.z())));
}

float gocDistance(osg::Vec2 p1, osg::Vec2 p2) {
	return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2));
}

//两点距离
float gocDistance(osg::Vec3 p1, osg::Vec3 p2) {
	return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) + pow(p1.z() - p2.z(), 2));
}

bool gocInline(osg::Vec3 p1, osg::Vec3 p2, osg::Vec3 p3) {
	return gocMode(gocCross(gocSub(p1, p2), gocSub(p2, p3))) < eps;
}

//计算2点差积:UxV
osg::Vec3 gocCross(osg::Vec3 u, osg::Vec3 v) {
	osg::Vec3 ret;
	ret.set(u.y()*v.z() - v.y()*u.z(), u.z()*v.x() - u.x()*v.z(), u.x()*v.y() - u.y()*v.x());
	return ret;
}

//向量大小
double gocMode(osg::Vec3 p) {
	return sqrt(p.x()*p.x() + p.y()*p.y() + p.z()*p.z());
}

osg::Vec3 gocSub(osg::Vec3 u, osg::Vec3 v) {
	osg::Vec3 ret;
	ret.set(u.x() - v.x(), u.y() - v.y(), u.z() - v.z());
	return ret;
}

osg::Vec4f gocColor::Set(unsigned int rgba) {
	_color._v[0] = (float)(rgba >> 24) / 255.0f;
	_color._v[1] = (float)((rgba & 0xFF0000) >> 16) / 255.0f;
	_color._v[2] = (float)((rgba & 0xFF00) >> 8) / 255.0f;
	_color._v[3] = (float)(rgba & 0xFF) / 255.0f;
	return _color;
}

void gocColor::Set(QColor & qc) {
	_color._v[0] = (float)qc.red() / 255.0f;
	_color._v[1] = (float)qc.green() / 255.0f;
	_color._v[2] = (float)qc.blue() / 255.0f;
	_color._v[3] = (float)qc.alpha() / 255.0f;
}

void gocColor::Set(osg::Vec4f & oc) {
	_color = oc;
}

osg::Vec4f gocColor::Get() {
	return _color;
}

QColor gocColor::GetRGB() {
	QColor color;
	int d[4];
	for (int i = 0; i < 4; i++) {
		d[i] = _color._v[i] * 256;
		if (d[i] < 0)d[i] = 0; else if (d[i] > 255)d[i] = 255;
	}
	color.setRgb(d[0], d[1], d[2], d[3]);
	return color;
}

unsigned int gocColor::GetUInt() {
	unsigned int d = (unsigned int)(_color._v[3] * 255);
	d += (unsigned int)(_color._v[2] * 256 * 255);
	d += (unsigned int)(_color._v[1] * 256 * 256 * 255);
	d += (unsigned int)(_color._v[0] * 256 * 256 * 256 * 255);
	return d;
}
