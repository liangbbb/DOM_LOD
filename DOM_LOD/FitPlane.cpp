#include "FitPlane.h"
#include <math.h>



PlaneProject::PlaneProject()
{
}

PlaneProject::PlaneProject(QString polFileName) {
	m_FileName = polFileName;
}

PlaneProject::~PlaneProject()
{
}

//我们拟合出来的方程：Ax+By+Cz=D,其中 A = plane12[0], B = plane12[1], C = plane12[2], D = plane12[3].
void PlaneProject::cvFitPlane(const CvMat * points, float * plane)
{
	int nrows = points->rows;
	int ncols = points->cols;
	int type = points->type;
	CvMat* centroid = cvCreateMat(1, ncols, type);
	cvSet(centroid, cvScalar(0));
	for (int c = 0; c < ncols; c++) {
		for (int r = 0; r < nrows; r++)
		{
			centroid->data.fl[c] += points->data.fl[ncols*r + c];
		}
		centroid->data.fl[c] /= nrows;
	}
	// Subtract geometric centroid from each point.
	CvMat* points2 = cvCreateMat(nrows, ncols, type);
	for (int r = 0; r < nrows; r++)
		for (int c = 0; c < ncols; c++)
			points2->data.fl[ncols*r + c] = points->data.fl[ncols*r + c] - centroid->data.fl[c];
	// Evaluate SVD of covariance matrix.
	CvMat* A = cvCreateMat(ncols, ncols, type);
	CvMat* W = cvCreateMat(ncols, ncols, type);
	CvMat* V = cvCreateMat(ncols, ncols, type);

	cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);
	cvSVD(A, W, NULL, V, CV_SVD_V_T);

	// Assign plane coefficients by singular vector corresponding to smallest singular value.
	plane[ncols] = 0;
	for (int c = 0; c < ncols; c++) {
		plane[c] = V->data.fl[ncols*(ncols - 1) + c];
		plane[ncols] += plane[c] * centroid->data.fl[c];
	}
	// Release allocated resources.
	cvReleaseMat(&centroid);
	cvReleaseMat(&points2);
	cvReleaseMat(&A);
	cvReleaseMat(&W);
	cvReleaseMat(&V);
}

//求投影面
//https://blog.csdn.net/soaryy/article/details/82884691
bool PlaneProject::Project2Plane(osg::ref_ptr<osg::Vec3Array> Polygon, float m_plane[4], osg::ref_ptr<osg::Vec3Array>ProjectPolygon) {
	double A = m_plane[0];
	double B = m_plane[1];
	double C = m_plane[2];
	//double D = 0.0;
	double D = -1.0*m_plane[3];

	//double ABC2 = m_plane[0] * m_plane[0] + m_plane[1] * m_plane[1] + m_plane[2] * m_plane[2];
	double ABC2 = A * A + B * B + C * C;

	for (int i = 0; i < Polygon->size(); i++) {
		double x = 0;
		double y = 0;
		double z = 0;

		osg::Vec3 pt;
		pt = Polygon->at(i);
		if (pt.x() == -99999.0)//异常值
		{
			x = -99999.0;
			y = -99999.0;
			z = -99999.0;
		}
		else
		{
			x = ((B * B + C * C) * pt.x() - A * (B * pt.y() + C * pt.z() + D)) / ABC2;
			y = ((A * A + C * C) * pt.y() - B * (A * pt.x() + C * pt.z() + D)) / ABC2;
			z = ((B * B + A * A) * pt.z() - C * (A * pt.x() + B * pt.y() + D)) / ABC2;
			/*Polygon->at(i)[0] = x;
			Polygon->at(i)[1] = y;
			Polygon->at(i)[2] = z;*/
			pt[0] = x;
			pt[1] = y;
			pt[2] = z;
		}
		(*ProjectPolygon)[i].set(pt);
	}
	return true;
}

bool PlaneProject::CoordinateRotation(osg::ref_ptr<osg::Vec3Array> ProjectPolygon, osg::ref_ptr<osg::Vec3Array> RotationPolygon)
{
	float P1[3];//点1
	float P2[3];//点2
	float P3[3];//点3
	P1[0] = ProjectPolygon->at(0)[0];
	P1[1] = ProjectPolygon->at(0)[1];
	P1[2] = ProjectPolygon->at(0)[2];
	P2[0] = ProjectPolygon->at(1)[0];
	P2[1] = ProjectPolygon->at(1)[1];
	P2[2] = ProjectPolygon->at(1)[2];
	P3[0] = ProjectPolygon->at(2)[0];
	P3[1] = ProjectPolygon->at(2)[1];
	P3[2] = ProjectPolygon->at(2)[2];

	//计算平面法向量
	float normal[3];
	double dnormal[3];
	ComputeNormal(P1, P2, P3, &dnormal[0]);
	normal[0] = dnormal[0];
	normal[1] = dnormal[1];
	normal[2] = dnormal[2];
	//计算旋转矩阵
	computeXYZ(P1, P2, normal);
	//坐标旋转
	osg::ref_ptr<osg::Vec3Array> Holes = new osg::Vec3Array(ProjectPolygon->size());//存储旋转后顶点坐标
	for (int i = 0; i < ProjectPolygon->size(); i++) {
		osg::Vec3f pt;
		pt = ProjectPolygon->at(i);
		GetXYZ(pt);//坐标旋转
		(*RotationPolygon)[i].set(pt);//存储
	}
	return true;
}

bool PlaneProject::computeXYZ(float v1[3], float v2[3], float normal[3])
{
	float directX[3];
	float directZ[3];
	double length;
	double dx = (double)v1[0] - v2[0];
	double dy = (double)v1[1] - v2[1];
	double dz = (double)v1[2] - v2[2];
	length = sqrt(dx * dx + dy * dy + dz * dz);
	if (length > 0.0) {
		dx /= length;
		dy /= length;
		dz /= length;
	}
	//X軸
	directX[0] = dx;
	directX[1] = dy;
	directX[2] = dz;
	//Z軸
	directZ[0] = (float)(normal[1] * dz - normal[2] * dy);
	directZ[1] = (float)(normal[2] * dx - normal[0] * dz);
	directZ[2] = (float)(normal[0] * dy - normal[1] * dx);

	/*旋转矩阵	*/
	R.set(directX[0], normal[0], directZ[0], 0,
		directX[1], normal[1], directZ[1], 0,
		directX[2], normal[2], directZ[2], 0,
		0, 0, 0, 1);
	R_i = osg::Matrixd::inverse(R);

	return true;
}

osg::Vec3f PlaneProject::GetXYZ(osg::Vec3f & v)
{
	v = R.preMult(v);
	return v;
}

bool PlaneProject::CalculateArea(osg::ref_ptr<osg::Vec3Array> RotationPolygon, double & area)
{
	int point_num = RotationPolygon->size();

	if (point_num < 3) {
		area = 0;
		return true;
	}

	double s = RotationPolygon->at(0)[2] * (RotationPolygon->at(point_num - 1)[0] - RotationPolygon->at(1)[0]);

	for (int i = 1; i < point_num; ++i) {
		s += RotationPolygon->at(i)[2] * (RotationPolygon->at(i - 1)[0] - RotationPolygon->at((i + 1) % point_num)[0]);
	}
	area = fabs(s / 2.0);
	return true;
}

bool PlaneProject::Calculateperimeter(osg::ref_ptr<osg::Vec3Array> RotationPolygon, double & perimeter)
{
	int point_num = RotationPolygon->size();
	double perimeterSum = 0;
	double perimeterSingle = 0;

	for (int i = 0; i < point_num; i++) {
		double Dx = RotationPolygon->at(i)[0] - RotationPolygon->at(i + 1)[0];
		double Dz = RotationPolygon->at(i)[2] - RotationPolygon->at(i + 1)[2];
		double temp = pow(Dx, 2) + pow(Dz, 2);
		perimeterSingle = pow(temp, 0.5);
		perimeterSum = perimeterSum + perimeterSingle;
		if (i == point_num - 2)break;
	}
	perimeter = perimeterSum;
	return true;
}


void PlaneProject::ComputeNormal(float v1[3], float v2[3], float v3[3], double n[3])
{
	double length;
	n[0] = n[2] = 0; n[1] = 1.0;
	ComputeNormalDirection(v1, v2, v3, n);
	length = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
	if (length > 0.0) {
		n[0] /= length;
		n[1] /= length;
		n[2] /= length;
	}
}

void PlaneProject::ComputeNormalDirection(float v1[3], float v2[3], float v3[3], double n[3])
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

