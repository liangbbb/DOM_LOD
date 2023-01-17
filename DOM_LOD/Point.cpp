#include "Point.h"
#include <iostream>

using namespace std;
SPoint::SPoint()
{
	Point_Vector = cv::Vec3d(0, 0, 0);//��ʼ��������
	Point_rgb = cv::Vec3d(0, 0, 0);//��ʼ��������
	PointUV = cv::Point2d(0, 0);
	PointUV_Vector = cv::Vec2d(0, 0);//��ʼ��������
	error = cv::Mat::zeros(4, 4, CV_64F);//��ʼ�����������
	Texture_error = cv::Mat::zeros(6, 6, CV_64F);//��ʼ��������������
	point_changeable = true;
	//pointPair_changeable = true;
}
//Point::Point(Vector3 _cdt)
//{
//	//cdt = _cdt;
//}

bool SPoint::hasNeighbor(int neiId)
{
	vector<int>::iterator iter;
	for (iter = neighbor.begin(); iter != neighbor.end(); ++iter)
		if (*iter == neiId)
			return true;
	return false;
}

void SPoint::addNeighbor(int neiId)
{
	if (!hasNeighbor(neiId))
		neighbor.push_back(neiId);
}

void SPoint::removeNeighbor(int neiId)
{
	if (hasNeighbor(neiId)) {
		vector<int>::iterator iter;
		for (iter = neighbor.begin(); iter != neighbor.end(); ++iter) {
			if (*iter == neiId) {
				iter = neighbor.erase(iter);
				break;
			}
		}
	}
}

void SPoint::calculateMat(PointSet* set)
{
	int isize = (int)neighbor.size();//һ����������
	error_reset();//��ʼ��������
	for (int i = 0; i < isize; ++i)
	{
		for (int j = i + 1; j < isize; ++j) {
			if (set->point.at(neighbor[i]).hasNeighbor(neighbor[j]))//�ж��Ƿ��ܹ���������
			{
				cv::Vec3d vn1 = set->point.at(neighbor[i]).Point_Vector - Point_Vector;
				cv::Vec3d vn2 = set->point.at(neighbor[j]).Point_Vector - Point_Vector;
				cv::Vec3d vn = vn1.cross(vn2);//��ƽ�淨����
				double a, b, c, d;
				double sum = sqrt(vn[0] * vn[0] + vn[1] * vn[1] + vn[2] * vn[2]);
				if (sum > 1e-10)
				{
					a = vn[0] / sum;
					b = vn[1] / sum;
					c = vn[2] / sum;
				}
				else
				{
					if (abs(vn[0]) > abs(vn[1]) && abs(vn[0]) > abs(vn[2]))
					{
						a = 1; b = 0; c = 0;
					}
					else if (abs(vn[1]) > abs(vn[2]) && abs(vn[1]) > abs(vn[0]))
					{
						a = 0; b = 1; c = 0;
					}
					else
					{
						a = 0; b = 0; c = 1;
					}
				}//��λ������(abc)
				d = -(Point_Vector[0] * a + Point_Vector[1] * b + Point_Vector[2] * c);//��ƽ��㷨ʽ�����е�d
				//cv::Mat V4 = (cv::Mat_<double>(4, 1) << a, b, c, d);
				cv::Mat V4 = cv::Mat::zeros(4, 1, CV_64F);
				V4.at<double>(0, 0) = a;
				V4.at<double>(1, 0) = b;
				V4.at<double>(2, 0) = c;
				V4.at<double>(3, 0) = d;

				//double lll;
				/*lll = V4.at<double>(0, 0);
				lll = V4.at<double>(1, 0);
				lll = V4.at<double>(2, 0);
				lll = V4.at<double>(3, 0);*/
				cv::Mat current_error = cv::Mat::zeros(4, 4, CV_64F);//��ʼ����ǰ���������
				current_error = V4 * V4.t();

				/*for (int p = 0; p < 4; p++) {
					for (int q = 0; q < 4; q++) {
						lll = current_error.at<double>(p, q);
					}
				}*/

				cv::Mat Error = error + current_error;
				/*for (int p = 0; p < 4; p++) {
					for (int q = 0; q < 4; q++) {
						lll = current_error.at<double>(p, q);
					}
				}*/
				error = Error;
			}
		}
	}
}

void SPoint::calculateUVMat(PointSet * set)
{
	Texture_errors_reset();//��ʼ�������󼯺�

	int isize = (int)neighbor.size();//һ����������

	for (auto a : SPointUV_index) {//�����õ��ÿһ�������
		Texture_error_reset();//��ʼ��������
		//����ÿһ���������ص�������
		for (int i = 0; i < isize; ++i) {
			for (int j = i + 1; j < isize; ++j) {
				if (set->point.at(neighbor[i]).hasNeighbor(neighbor[j])) {//�ж��Ƿ��ܹ���������
					SPoint P = set->point.at(neighbor[i]);
					SPoint Q = set->point.at(neighbor[j]);

					cv::Mat p = cv::Mat::zeros(5, 1, CV_64F);//��P����ά����
					cv::Mat q = cv::Mat::zeros(5, 1, CV_64F);//��Q����ά����
					cv::Mat r = cv::Mat::zeros(5, 1, CV_64F);//��R����ά����
					r.at<double>(0, 0) = P.Point_Vector[0];
					r.at<double>(1, 0) = P.Point_Vector[1];
					r.at<double>(2, 0) = P.Point_Vector[2];
					q.at<double>(0, 0) = Q.Point_Vector[0];
					q.at<double>(1, 0) = Q.Point_Vector[1];
					q.at<double>(2, 0) = Q.Point_Vector[2];
					p.at<double>(0, 0) = Point_Vector[0];
					p.at<double>(1, 0) = Point_Vector[1];
					p.at<double>(2, 0) = Point_Vector[2];

					for (auto b : P.SPointUV_index) {//����P���ÿһ�������
						for (auto c : Q.SPointUV_index) {//����Q���ÿһ�������
							SPoint_UV A = set->point_uv.at(a);
							SPoint_UV B = set->point_uv.at(b);
							SPoint_UV C = set->point_uv.at(c);
							if (A.hasNeighbor(b) && A.hasNeighbor(c) && B.hasNeighbor(a) && B.hasNeighbor(c) && C.hasNeighbor(a) && C.hasNeighbor(b)) {//�ж��Ƿ��ܹ�������������
								r.at<double>(3, 0) = B.PointUV_Vector[0];
								r.at<double>(4, 0) = B.PointUV_Vector[1];
								q.at<double>(3, 0) = C.PointUV_Vector[0];
								q.at<double>(4, 0) = C.PointUV_Vector[1];
								p.at<double>(3, 0) = A.PointUV_Vector[0];
								p.at<double>(4, 0) = A.PointUV_Vector[1];

								double E1[5], E2[5], r_minus_p[5];
								//double E1[6], E2[6], r_minus_p[6];
								for (int i = 0; i < 5; i++) {
									if (i < 3) {
										E1[i] = Q.Point_Vector[i] - Point_Vector[i];
										r_minus_p[i] = P.Point_Vector[i] - Point_Vector[i];
									}
									else
									{
										//E1[i] = Q.Point_rgb[i % 3] - Point_rgb[i % 3];
										E1[i] = q.at<double>(i, 0) - p.at<double>(i, 0);
										r_minus_p[i] = r.at<double>(i, 0) - p.at<double>(i, 0);
									}
								}
								double E1_Length = sqrt(E1[0] * E1[0] + E1[1] * E1[1] + E1[2] * E1[2] + E1[3] * E1[3] + E1[4] * E1[4]);
								for (int i = 0; i < 5; i++) {//e1��λ��
									E1[i] = E1[i] / E1_Length;
								}
								double e1rp = 0;
								for (int i = 0; i < 5; i++) {
									e1rp = e1rp + E1[i] * r_minus_p[i];
								}
								for (int i = 0; i < 5; i++) {//e2
									E2[i] = r_minus_p[i] - e1rp * E1[i];
								}
								double E2_Length = sqrt(E2[0] * E2[0] + E2[1] * E2[1] + E2[2] * E2[2] + E2[3] * E2[3] + E2[4] * E2[4]);
								for (int i = 0; i < 5; i++) {//e2��λ��
									E2[i] = E2[i] / E2_Length;
								}

								double Mat_A[5][5];//A
								double Mat_I[5][5];//I
								for (int i = 0; i < 5; i++) {
									for (int j = 0; j < 5; j++) {
										if (i == j) {
											Mat_I[i][j] = 1;
										}
										else
										{
											Mat_I[i][j] = 0;
										}

									}
								}

								for (int i = 0; i < 5; i++) {
									for (int j = 0; j < 5; j++) {
										Mat_A[i][j] = Mat_I[i][j] - E1[i] * E1[j] - E2[i] * E2[j];
									}
								}

								double Vec_b[5], pe1, pe2, pp;//b,p*e1,p*e2,p*p
								pe1 = 0.0;
								pe2 = 0.0;
								pp = 0.0;

								for (int i = 0; i < 5; i++) {
									pe1 = pe1 + p.at<double>(i, 0)*E1[i];
									pe2 = pe2 + p.at<double>(i, 0)*E2[i];
									pp = pp + p.at<double>(i, 0)*p.at<double>(i, 0);
								}
								for (int i = 0; i < 5; i++) {
									Vec_b[i] = pe1 * E1[i] + pe2 * E2[i] - p.at<double>(i, 0);
								}
								double parameter_c;//c
								parameter_c = pp - pe1 * pe1 - pe2 * pe2;

								double M[6][6];//Q
								for (int i = 0; i < 6; i++) {
									for (int j = 0; j < 6; j++) {
										if (i < 5 && j < 5) {
											M[i][j] = Mat_A[i][j];
										}
										if (i == 5 && j < 5) {
											M[i][j] = Vec_b[j];
										}
										if (j == 5 && i < 5) {
											M[i][j] = Vec_b[i];
										}
										if (i == 5 && j == 5) {
											M[i][j] = parameter_c;
										}
									}
								}

								//�������
								for (int i = 0; i < 6; i++) {
									for (int j = 0; j < 6; j++) {
										Texture_error.at<double>(i, j) = Texture_error.at<double>(i, j) + M[i][j];
									}
								}

							}
						}
					}

				}
			}
		}
		//��������뼯��
		Texture_errors.push_back(Texture_error);
	}


	//if (set->point.at(neighbor[i]).hasNeighbor(neighbor[j]))//�ж��Ƿ��ܹ���������
	//{
	//	SPoint P = set->point.at(neighbor[i]);
	//	SPoint Q = set->point.at(neighbor[j]);
	//
	//	cv::Mat p = cv::Mat::zeros(5, 1, CV_64F);//��P����ά����
	//	cv::Mat q = cv::Mat::zeros(5, 1, CV_64F);//��Q����ά����
	//	cv::Mat r = cv::Mat::zeros(5, 1, CV_64F);//��R����ά����
	//	p.at<double>(0, 0) = P.Point_Vector[0];
	//	p.at<double>(1, 0) = P.Point_Vector[1];
	//	p.at<double>(2, 0) = P.Point_Vector[2];
	//	p.at<double>(3, 0) = P.PointUV.x;
	//	p.at<double>(4, 0) = P.PointUV.y;
	//	q.at<double>(0, 0) = Q.Point_Vector[0];
	//	q.at<double>(1, 0) = Q.Point_Vector[1];
	//	q.at<double>(2, 0) = Q.Point_Vector[2];
	//	q.at<double>(3, 0) = Q.PointUV.x;
	//	q.at<double>(4, 0) = Q.PointUV.y;
	//	r.at<double>(0, 0) = Point_Vector[0];
	//	r.at<double>(1, 0) = Point_Vector[1];
	//	r.at<double>(2, 0) = Point_Vector[2];
	//	r.at<double>(3, 0) = PointUV.x;
	//	r.at<double>(4, 0) = PointUV.y;

	//	cv::Mat e1 = q - p;
	//	cv::Mat tem1 = e1.t()*e1;
	//	double L1 = sqrt(tem1.at<double>(0, 0));
	//	if (L1 < 1e-10) {
	//		int uu = 66;
	//	}
	//	cv::Mat Unit_e1 = e1 / L1;

	//	cv::Mat tem2 = Unit_e1.t()*(r - p);
	//	cv::Mat e2 = r - p - tem2.at<double>(0, 0)*Unit_e1;
	//	cv::Mat tem3 = e2.t()*e2;
	//	double L2 = sqrt(tem3.at<double>(0, 0));
	//	if (L2 < 1e-10) {
	//		int uup = 66;
	//	}
	//	cv::Mat Unit_e2 = e2 / L2;

	//	cv::Mat tempp = Unit_e1.t()*Unit_e2;
	//	double pp = tempp.at<double>(0, 0);

	//	cv::Mat A = 1 - Unit_e1 * e1.t() - Unit_e2 * e2.t();

	//	cv::Mat tem4 = p.t()*Unit_e1;
	//	cv::Mat tem5 = p.t()*Unit_e2;
	//	cv::Mat b = tem4.at<double>(0, 0)*Unit_e1 + tem5.at<double>(0, 0)*Unit_e2 - p;

	//	cv::Mat c = p.t()*p - (p.t()*Unit_e1)*(p.t()*Unit_e1) - (p.t()*Unit_e2)*(p.t()*Unit_e2);

	//	cv::Mat current_error = cv::Mat::zeros(6, 6, CV_64F);//��ʼ����ǰ�������������
	//	cv::Mat temp;
	//	cv::hconcat(A, b, temp);
	//	cv::Mat temp1;
	//	cv::hconcat(b.t(), c, temp1);
	//	cv::vconcat(temp, temp1, current_error);

	//	cv::Mat Error = Texture_error + current_error;

	//	/*double lll;
	//	for (int p = 0; p < 6; p++) {
	//		for (int q = 0; q < 6; q++) {
	//			lll = current_error.at<double>(p, q);
	//		}
	//	}*/
	//	Texture_error = Error;
	//}

}

void SPoint::calculateUVMat1(PointSet * set)
{

	int isize = (int)neighbor.size();//һ����������
	Texture_error_reset();//��ʼ��������
	//����ÿһ���������ص�������	
	for (int i = 0; i < isize; ++i) {
		for (int j = i + 1; j < isize; ++j) {

			if (set->point.at(neighbor[i]).hasNeighbor(neighbor[j]))//�ж��Ƿ��ܹ���������
			{
				SPoint P = set->point.at(neighbor[i]);
				SPoint Q = set->point.at(neighbor[j]);

				cv::Mat p = cv::Mat::zeros(5, 1, CV_64F);//��P����ά����
				cv::Mat q = cv::Mat::zeros(5, 1, CV_64F);//��Q����ά����
				cv::Mat r = cv::Mat::zeros(5, 1, CV_64F);//��R����ά����
				p.at<double>(0, 0) = P.Point_Vector[0];
				p.at<double>(1, 0) = P.Point_Vector[1];
				p.at<double>(2, 0) = P.Point_Vector[2];
				p.at<double>(3, 0) = P.PointUV.x;
				p.at<double>(4, 0) = P.PointUV.y;
				q.at<double>(0, 0) = Q.Point_Vector[0];
				q.at<double>(1, 0) = Q.Point_Vector[1];
				q.at<double>(2, 0) = Q.Point_Vector[2];
				q.at<double>(3, 0) = Q.PointUV.x;
				q.at<double>(4, 0) = Q.PointUV.y;
				r.at<double>(0, 0) = Point_Vector[0];
				r.at<double>(1, 0) = Point_Vector[1];
				r.at<double>(2, 0) = Point_Vector[2];
				r.at<double>(3, 0) = PointUV.x;
				r.at<double>(4, 0) = PointUV.y;

				double E1[5], E2[5], r_minus_p[5];
				//double E1[6], E2[6], r_minus_p[6];
				for (int i = 0; i < 5; i++) {
					if (i < 3) {
						E1[i] = Q.Point_Vector[i] - Point_Vector[i];
						r_minus_p[i] = P.Point_Vector[i] - Point_Vector[i];
					}
					else
					{
						//E1[i] = Q.Point_rgb[i % 3] - Point_rgb[i % 3];
						E1[i] = q.at<double>(i, 0) - p.at<double>(i, 0);
						r_minus_p[i] = r.at<double>(i, 0) - p.at<double>(i, 0);
					}
				}
				double E1_Length = sqrt(E1[0] * E1[0] + E1[1] * E1[1] + E1[2] * E1[2] + E1[3] * E1[3] + E1[4] * E1[4]);
				for (int i = 0; i < 5; i++) {//e1��λ��
					E1[i] = E1[i] / E1_Length;
				}
				double e1rp = 0;
				for (int i = 0; i < 5; i++) {
					e1rp = e1rp + E1[i] * r_minus_p[i];
				}
				for (int i = 0; i < 5; i++) {//e2
					E2[i] = r_minus_p[i] - e1rp * E1[i];
				}
				double E2_Length = sqrt(E2[0] * E2[0] + E2[1] * E2[1] + E2[2] * E2[2] + E2[3] * E2[3] + E2[4] * E2[4]);
				for (int i = 0; i < 5; i++) {//e2��λ��
					E2[i] = E2[i] / E2_Length;
				}

				double Mat_A[5][5];//A
				double Mat_I[5][5];//I
				for (int i = 0; i < 5; i++) {
					for (int j = 0; j < 5; j++) {
						if (i == j) {
							Mat_I[i][j] = 1;
						}
						else
						{
							Mat_I[i][j] = 0;
						}
					}
				}

				for (int i = 0; i < 5; i++) {
					for (int j = 0; j < 5; j++) {
						Mat_A[i][j] = Mat_I[i][j] - E1[i] * E1[j] - E2[i] * E2[j];
					}
				}

				double Vec_b[5], pe1, pe2, pp;//b,p*e1,p*e2,p*p
				pe1 = 0.0;
				pe2 = 0.0;
				pp = 0.0;

				for (int i = 0; i < 5; i++) {
					pe1 = pe1 + p.at<double>(i, 0)*E1[i];
					pe2 = pe2 + p.at<double>(i, 0)*E2[i];
					pp = pp + p.at<double>(i, 0)*p.at<double>(i, 0);
				}
				for (int i = 0; i < 5; i++) {
					Vec_b[i] = pe1 * E1[i] + pe2 * E2[i] - p.at<double>(i, 0);
				}
				double parameter_c;//c
				parameter_c = pp - pe1 * pe1 - pe2 * pe2;

				double M[6][6];//Q
				for (int i = 0; i < 6; i++) {
					for (int j = 0; j < 6; j++) {
						if (i < 5 && j < 5) {
							M[i][j] = Mat_A[i][j];
						}
						if (i == 5 && j < 5) {
							M[i][j] = Vec_b[j];
						}
						if (j == 5 && i < 5) {
							M[i][j] = Vec_b[i];
						}
						if (i == 5 && j == 5) {
							M[i][j] = parameter_c;
						}
					}
				}
				//�������
				for (int i = 0; i < 6; i++) {
					for (int j = 0; j < 6; j++) {
						Texture_error.at<double>(i, j) = Texture_error.at<double>(i, j) + M[i][j];
					}
				}

				//cv::Mat e1 = q - p;
				//cv::Mat tem1 = e1.t()*e1;
				//double L1 = sqrt(tem1.at<double>(0, 0));
				//if (L1 < 1e-10) {
				//	int uu = 66;
				//}
				//cv::Mat Unit_e1 = e1 / L1;

				//cv::Mat tem2 = Unit_e1.t()*(r - p);
				//cv::Mat e2 = r - p - tem2.at<double>(0, 0)*Unit_e1;
				//cv::Mat tem3 = e2.t()*e2;
				//double L2 = sqrt(tem3.at<double>(0, 0));
				//if (L2 < 1e-10) {
				//	int uup = 66;
				//}
				//cv::Mat Unit_e2 = e2 / L2;

				//cv::Mat tempp = Unit_e1.t()*Unit_e2;
				//double pp = tempp.at<double>(0, 0);

				//cv::Mat A = 1 - Unit_e1 * e1.t() - Unit_e2 * e2.t();

				//cv::Mat tem4 = p.t()*Unit_e1;
				//cv::Mat tem5 = p.t()*Unit_e2;
				//cv::Mat b = tem4.at<double>(0, 0)*Unit_e1 + tem5.at<double>(0, 0)*Unit_e2 - p;

				//cv::Mat c = p.t()*p - (p.t()*Unit_e1)*(p.t()*Unit_e1) - (p.t()*Unit_e2)*(p.t()*Unit_e2);

				//cv::Mat current_error = cv::Mat::zeros(6, 6, CV_64F);//��ʼ����ǰ�������������
				//cv::Mat temp;
				//cv::hconcat(A, b, temp);
				//cv::Mat temp1;
				//cv::hconcat(b.t(), c, temp1);
				//cv::vconcat(temp, temp1, current_error);

				//cv::Mat Error = Texture_error + current_error;

				///*double lll;
				//for (int p = 0; p < 6; p++) {
				//	for (int q = 0; q < 6; q++) {
				//		lll = current_error.at<double>(p, q);
				//	}
				//}*/
				//Texture_error = Error;
			}
		}
	}


}

void SPoint::calculateQEMMat(PointSet * set)
{
	int isize = (int)neighbor.size();//һ����������
	Texture_error_reset();//��ʼ�����������������

	for (int i = 0; i < isize; ++i) {
		for (int j = i + 1; j < isize; ++j) {
			if (set->point.at(neighbor[i]).hasNeighbor(neighbor[j])) {//�ж��Ƿ��ܹ���������
				SPoint P = set->point.at(neighbor[i]);
				SPoint Q = set->point.at(neighbor[j]);

				cv::Mat p = cv::Mat::zeros(5, 1, CV_64F);//��P����ά����
				cv::Mat q = cv::Mat::zeros(5, 1, CV_64F);//��Q����ά����
				cv::Mat r = cv::Mat::zeros(5, 1, CV_64F);//��R����ά����
				r.at<double>(0, 0) = P.Point_Vector[0];
				r.at<double>(1, 0) = P.Point_Vector[1];
				r.at<double>(2, 0) = P.Point_Vector[2];
				q.at<double>(0, 0) = Q.Point_Vector[0];
				q.at<double>(1, 0) = Q.Point_Vector[1];
				q.at<double>(2, 0) = Q.Point_Vector[2];
				p.at<double>(0, 0) = Point_Vector[0];
				p.at<double>(1, 0) = Point_Vector[1];
				p.at<double>(2, 0) = Point_Vector[2];

				bool cg = false;
				for (auto a : SPointUV_index) {//�����õ��ÿһ�������
					for (auto b : P.SPointUV_index) {//����P���ÿһ�������
						for (auto c : Q.SPointUV_index) {//����Q���ÿһ�������
							SPoint_UV A = set->point_uv.at(a);
							SPoint_UV B = set->point_uv.at(b);
							SPoint_UV C = set->point_uv.at(c);
							if (A.hasNeighbor(b) && A.hasNeighbor(c) && B.hasNeighbor(a) && B.hasNeighbor(c) && C.hasNeighbor(a) && C.hasNeighbor(b)) {//�ж��Ƿ��ܹ�������������
								cg = true;
								r.at<double>(3, 0) = B.PointUV_Vector[0];
								r.at<double>(4, 0) = B.PointUV_Vector[1];
								q.at<double>(3, 0) = C.PointUV_Vector[0];
								q.at<double>(4, 0) = C.PointUV_Vector[1];
								p.at<double>(3, 0) = A.PointUV_Vector[0];
								p.at<double>(4, 0) = A.PointUV_Vector[1];

								double E1[5], E2[5], r_minus_p[5];
								//double E1[6], E2[6], r_minus_p[6];
								for (int i = 0; i < 5; i++) {
									if (i < 3) {
										E1[i] = Q.Point_Vector[i] - Point_Vector[i];
										r_minus_p[i] = P.Point_Vector[i] - Point_Vector[i];
									}
									else
									{
										//E1[i] = Q.Point_rgb[i % 3] - Point_rgb[i % 3];
										E1[i] = q.at<double>(i, 0) - p.at<double>(i, 0);
										r_minus_p[i] = r.at<double>(i, 0) - p.at<double>(i, 0);
									}
								}
								double E1_Length = sqrt(E1[0] * E1[0] + E1[1] * E1[1] + E1[2] * E1[2] + E1[3] * E1[3] + E1[4] * E1[4]);
								for (int i = 0; i < 5; i++) {//e1��λ��
									E1[i] = E1[i] / E1_Length;
								}
								double e1rp = 0;
								for (int i = 0; i < 5; i++) {
									e1rp = e1rp + E1[i] * r_minus_p[i];
								}
								for (int i = 0; i < 5; i++) {//e2
									E2[i] = r_minus_p[i] - e1rp * E1[i];
								}
								double E2_Length = sqrt(E2[0] * E2[0] + E2[1] * E2[1] + E2[2] * E2[2] + E2[3] * E2[3] + E2[4] * E2[4]);
								for (int i = 0; i < 5; i++) {//e2��λ��
									E2[i] = E2[i] / E2_Length;
								}

								double Mat_A[5][5];//A
								double Mat_I[5][5];//I
								for (int i = 0; i < 5; i++) {
									for (int j = 0; j < 5; j++) {
										if (i == j) {
											Mat_I[i][j] = 1;
										}
										else
										{
											Mat_I[i][j] = 0;
										}
									}
								}

								for (int i = 0; i < 5; i++) {
									for (int j = 0; j < 5; j++) {
										Mat_A[i][j] = Mat_I[i][j] - E1[i] * E1[j] - E2[i] * E2[j];
									}
								}

								double Vec_b[5], pe1, pe2, pp;//b,p*e1,p*e2,p*p
								pe1 = 0.0;
								pe2 = 0.0;
								pp = 0.0;

								for (int i = 0; i < 5; i++) {
									pe1 = pe1 + p.at<double>(i, 0)*E1[i];
									pe2 = pe2 + p.at<double>(i, 0)*E2[i];
									pp = pp + p.at<double>(i, 0)*p.at<double>(i, 0);
								}
								for (int i = 0; i < 5; i++) {
									Vec_b[i] = pe1 * E1[i] + pe2 * E2[i] - p.at<double>(i, 0);
								}
								double parameter_c;//c
								parameter_c = pp - pe1 * pe1 - pe2 * pe2;

								double M[6][6];//Q
								for (int i = 0; i < 6; i++) {
									for (int j = 0; j < 6; j++) {
										if (i < 5 && j < 5) {
											M[i][j] = Mat_A[i][j];
										}
										if (i == 5 && j < 5) {
											M[i][j] = Vec_b[j];
										}
										if (j == 5 && i < 5) {
											M[i][j] = Vec_b[i];
										}
										if (i == 5 && j == 5) {
											M[i][j] = parameter_c;
										}
									}
								}
								//�������
								for (int i = 0; i < 6; i++) {
									for (int j = 0; j < 6; j++) {
										Texture_error.at<double>(i, j) = Texture_error.at<double>(i, j) + M[i][j];
									}
								}

							}
						}
					}
				}
				if (cg == false) {
					int ooo = 6;
				}
			}
		}
	}

}

//����ϸ�������������󼯺ϼ���
void SPoint::calculateD_QEMMat(PointSet * set)
{
	int isize = (int)neighbor.size();//һ����������
	Texture_errors_reset();//��ʼ���˼�ϸ�ڵĶ��������󼯺�

	//���㶥��Ķ��������󼯺�
	for (auto a : SPointUV_index) {//�����õ��ÿһ��������ʶ
		Texture_error_reset();//��ʼ������������

		//����ÿһ���������ص�������
		for (int i = 0; i < isize; ++i) {
			for (int j = i + 1; j < isize; ++j) {
				if (set->point.at(neighbor[i]).hasNeighbor(neighbor[j])) {//�ж��Ƿ��ܹ���������
					SPoint P = set->point.at(neighbor[i]);
					SPoint Q = set->point.at(neighbor[j]);

					cv::Mat p = cv::Mat::zeros(5, 1, CV_64F);//��P����ά����
					cv::Mat q = cv::Mat::zeros(5, 1, CV_64F);//��Q����ά����
					cv::Mat r = cv::Mat::zeros(5, 1, CV_64F);//��R����ά����
					r.at<double>(0, 0) = P.Point_Vector[0];
					r.at<double>(1, 0) = P.Point_Vector[1];
					r.at<double>(2, 0) = P.Point_Vector[2];
					q.at<double>(0, 0) = Q.Point_Vector[0];
					q.at<double>(1, 0) = Q.Point_Vector[1];
					q.at<double>(2, 0) = Q.Point_Vector[2];
					p.at<double>(0, 0) = Point_Vector[0];
					p.at<double>(1, 0) = Point_Vector[1];
					p.at<double>(2, 0) = Point_Vector[2];

					for (auto b : P.SPointUV_index) {//����P���ÿһ�������
						for (auto c : Q.SPointUV_index) {//����Q���ÿһ�������
							SPoint_UV A = set->point_uv.at(a);
							SPoint_UV B = set->point_uv.at(b);
							SPoint_UV C = set->point_uv.at(c);
							if (A.hasNeighbor(b) && A.hasNeighbor(c) && B.hasNeighbor(a) && B.hasNeighbor(c) && C.hasNeighbor(a) && C.hasNeighbor(b)) {//�ж��Ƿ��ܹ�������������
								r.at<double>(3, 0) = B.PointUV_Vector[0];
								r.at<double>(4, 0) = B.PointUV_Vector[1];
								q.at<double>(3, 0) = C.PointUV_Vector[0];
								q.at<double>(4, 0) = C.PointUV_Vector[1];
								p.at<double>(3, 0) = A.PointUV_Vector[0];
								p.at<double>(4, 0) = A.PointUV_Vector[1];

								double E1[5], E2[5], r_minus_p[5];
								//double E1[6], E2[6], r_minus_p[6];
								for (int i = 0; i < 5; i++) {
									if (i < 3) {
										E1[i] = Q.Point_Vector[i] - Point_Vector[i];
										r_minus_p[i] = P.Point_Vector[i] - Point_Vector[i];
									}
									else
									{
										//E1[i] = Q.Point_rgb[i % 3] - Point_rgb[i % 3];
										E1[i] = q.at<double>(i, 0) - p.at<double>(i, 0);
										r_minus_p[i] = r.at<double>(i, 0) - p.at<double>(i, 0);
									}
								}
								double E1_Length = sqrt(E1[0] * E1[0] + E1[1] * E1[1] + E1[2] * E1[2] + E1[3] * E1[3] + E1[4] * E1[4]);
								for (int i = 0; i < 5; i++) {//e1��λ��
									E1[i] = E1[i] / E1_Length;
								}
								double e1rp = 0;
								for (int i = 0; i < 5; i++) {
									e1rp = e1rp + E1[i] * r_minus_p[i];
								}
								for (int i = 0; i < 5; i++) {//e2
									E2[i] = r_minus_p[i] - e1rp * E1[i];
								}
								double E2_Length = sqrt(E2[0] * E2[0] + E2[1] * E2[1] + E2[2] * E2[2] + E2[3] * E2[3] + E2[4] * E2[4]);
								for (int i = 0; i < 5; i++) {//e2��λ��
									E2[i] = E2[i] / E2_Length;
								}

								double Mat_A[5][5];//A
								double Mat_I[5][5];//I
								for (int i = 0; i < 5; i++) {
									for (int j = 0; j < 5; j++) {
										if (i == j) {
											Mat_I[i][j] = 1;
										}
										else
										{
											Mat_I[i][j] = 0;
										}

									}
								}

								for (int i = 0; i < 5; i++) {
									for (int j = 0; j < 5; j++) {
										Mat_A[i][j] = Mat_I[i][j] - E1[i] * E1[j] - E2[i] * E2[j];
									}
								}

								double Vec_b[5], pe1, pe2, pp;//b,p*e1,p*e2,p*p
								pe1 = 0.0;
								pe2 = 0.0;
								pp = 0.0;

								for (int i = 0; i < 5; i++) {
									pe1 = pe1 + p.at<double>(i, 0)*E1[i];
									pe2 = pe2 + p.at<double>(i, 0)*E2[i];
									pp = pp + p.at<double>(i, 0)*p.at<double>(i, 0);
								}
								for (int i = 0; i < 5; i++) {
									Vec_b[i] = pe1 * E1[i] + pe2 * E2[i] - p.at<double>(i, 0);
								}
								double parameter_c;//c
								parameter_c = pp - pe1 * pe1 - pe2 * pe2;

								double M[6][6];//Q
								for (int i = 0; i < 6; i++) {
									for (int j = 0; j < 6; j++) {
										if (i < 5 && j < 5) {
											M[i][j] = Mat_A[i][j];
										}
										if (i == 5 && j < 5) {
											M[i][j] = Vec_b[j];
										}
										if (j == 5 && i < 5) {
											M[i][j] = Vec_b[i];
										}
										if (i == 5 && j == 5) {
											M[i][j] = parameter_c;
										}
									}
								}

								//�������
								for (int i = 0; i < 6; i++) {
									for (int j = 0; j < 6; j++) {
										Texture_error.at<double>(i, j) = Texture_error.at<double>(i, j) + M[i][j];
									}
								}

							}
						}
					}

				}
			}
		}
		//��������뼯��
		Texture_errors.push_back(Texture_error);
	}

	//����һ������ӷ�Ƕ����
	int N_SPoint_UV = SPointUV_index.size();//����������������
	double SAE = 1;
	//SAE = SeamAngleError(set, N_SPoint_UV);//����Ƕ����

	//�����������������Ӷ�


	//�����������㶥������
}


void SPoint::error_reset()
{
	error = cv::Mat::zeros(4, 4, CV_64F);
}

void SPoint::Texture_error_reset()
{
	//Texture_error = cv::Mat::zeros(7, 7, CV_64F);
	Texture_error = cv::Mat::zeros(6, 6, CV_64F);
}

void SPoint::Texture_errors_reset()
{
	Texture_errors.clear();
}


//����ӷ�Ƕ����
double SPoint::SeamAngleError(PointSet* set, int num_SPoint_UV)
{
	double E;
	SPoint p1, p2;//�����ڽӷ��
	int n = 0;
	int isize = (int)neighbor.size();//һ����������

	switch (num_SPoint_UV)
	{
	case 1:
		E = 1;
		break;
	case 2:
		for (int i = 0; i < isize; ++i) {
			if (set->point.at(neighbor[i]).SPointUV_index.size() > 1 && n <= 1) {//�ҵ��ӷ��
				if (n == 0) {
					p1 = set->point.at(neighbor[i]);//�ҵ���һ��
				}
				else
				{
					p2 = set->point.at(neighbor[i]);//�ҵ��ڶ���
				}
				n++;
			}
		}
		if (n == 2) {//�ҵ������ӷ��
			for (auto a : SPointUV_index) {//�����õ��ÿһ�������
				for (auto b : p1.SPointUV_index) {//����P1���ÿһ�������
					for (auto c : p2.SPointUV_index) {//����p2���ÿһ�������
						SPoint_UV A = set->point_uv.at(a);
						SPoint_UV B = set->point_uv.at(b);
						SPoint_UV C = set->point_uv.at(c);
						if (A.hasNeighbor(b) && B.hasNeighbor(a) && A.hasNeighbor(c) && C.hasNeighbor(a)) {
							cv::Vec3d pp1_xyz = p1.Point_Vector - Point_Vector;
							cv::Vec2d pp1_uv = B.PointUV_Vector - A.PointUV_Vector;
							cv::Mat pp1 = cv::Mat::zeros(1, 5, CV_64F);
							pp1.at<double>(0, 0) = pp1_xyz[0];
							pp1.at<double>(0, 1) = pp1_xyz[1];
							pp1.at<double>(0, 2) = pp1_xyz[2];
							pp1.at<double>(0, 3) = pp1_uv[0];
							pp1.at<double>(0, 4) = pp1_uv[1];

							cv::Vec3d pp2_xyz = p2.Point_Vector - Point_Vector;
							cv::Vec2d pp2_uv = C.PointUV_Vector - A.PointUV_Vector;
							cv::Mat pp2 = cv::Mat::zeros(5, 1, CV_64F);
							pp2.at<double>(0, 0) = pp2_xyz[0];
							pp2.at<double>(1, 0) = pp2_xyz[1];
							pp2.at<double>(2, 0) = pp2_xyz[2];
							pp2.at<double>(3, 0) = pp2_uv[0];
							pp2.at<double>(4, 0) = pp2_uv[1];

							cv::Mat pp1_pp2 = pp1 * pp2;
							double ss = pp1_pp2.at<double>(0, 0);
							//��pp1��ģ��
							cv::Mat m1 = pp1 * pp1.t();
							double s_pp1 = sqrt(m1.at<double>(0, 0));
							//��pp2��ģ��
							cv::Mat m2 = pp2.t() * pp2;
							double s_pp2 = sqrt(m2.at<double>(0, 0));
							//�н�����ֵcos
							double cos_angle = ss / (s_pp1*s_pp2);
							//�нǷ����һ���ֵ
							double angle = acos(cos_angle);
							//����Ƕ����
							//E = E + 1 + 3.14 - angle;
							E = E + 1 + 1 + cos_angle;
						}
					}
				}
			}
			//ƽ���ӷ�Ƕ����
			E = 0.5*E;
		}
		else
		{
			E = 10000;
		}
		break;
	default:
		E = 10000;
		break;
	}

	return E;
}


//void Point::accelerateFinding(PointSet* set)
//{
//	int nsize = neighbor.size();
//	for (int i = 0; i < nsize - 1; ++i)
//	{
//		int j;
//		for (j = i + 1; j < nsize; ++j)
//			if (set->point[neighbor[i]].hasNeighbor(neighbor[j]))
//			{
//				int tmp = neighbor[i + 1];
//				neighbor[i + 1] = neighbor[j];
//				neighbor[j] = tmp;
//				break;
//			}
//	}
//}

//Point Point::operator=(Point& _p)
//{
//	cdt = _p.cdt;
//	error = _p.error;//might have a bug?
//	neighbor = _p.neighbor;
//	return *this;
//}

SPoint_UV::SPoint_UV()
{
	PointUV_Vector = cv::Vec2d(0, 0);//��ʼ��������
}

bool SPoint_UV::hasNeighbor(int neiId)
{
	vector<int>::iterator iter;
	for (iter = neighbor.begin(); iter != neighbor.end(); ++iter)
		if (*iter == neiId)
			return true;
	return false;
}

void SPoint_UV::addNeighbor(int neiId)
{
	if (!hasNeighbor(neiId))
		neighbor.push_back(neiId);
}

