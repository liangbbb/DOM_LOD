#pragma once
#include "CU.h"

CU::CU(cv::String _address)
{
	src = cv::imread(_address);
	if (src.empty()) {
		return;
	}
	if (src.channels() > 1) cv::cvtColor(src, src, CV_RGB2GRAY);//ת�Ҷ�ͼ��src 
	Edge_Canny(src, Canny, 50, 100);

}
CU::~CU()
{
}

float CU::mul(TPoint p1, TPoint p2, TPoint p0)
{
	return ((p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y));
}

/*����������������ĵ�����ʸ����ע�ⷽ�򣩣�Ȼ����������������ʸ���γ�ƽ�棬
 * �������ƽ���ʣ�µ�ʸ����ˣ��ó�һ����ʸ����������������������⣬��֮�����档
 */
int CU::inside(TPoint tr[], TPoint p)
{
	int i;
	for (i = 0; i < 3; i++)
		if (mul(p, tr[i], tr[(i + 1) % 3]) * mul(p, tr[(i + 2) % 3], tr[(i + 1) % 3]) > 0)
			return 0;
	return 1;
}

//sobel���� /
//�׳�
int CU::factorial(int n) {
	int fac = 1;
	//0�Ľ׳�
	if (n == 0)
		return fac;
	for (int i = 1; i <= n; ++i) {
		fac *= i;
	}
	return fac;
}

//���Sobelƽ������
cv::Mat CU::getSobelSmoooth(int wsize) {
	int n = wsize - 1;
	cv::Mat SobelSmooothoper = cv::Mat::zeros(cv::Size(wsize, 1), CV_32FC1);
	for (int k = 0; k <= n; k++) {
		float *pt = SobelSmooothoper.ptr<float>(0);
		pt[k] = factorial(n) / (factorial(k)*factorial(n - k));
	}
	return SobelSmooothoper;
}
//���Sobel�������
cv::Mat CU::getSobeldiff(int wsize) {
	cv::Mat Sobeldiffoper = cv::Mat::zeros(cv::Size(wsize, 1), CV_32FC1);
	cv::Mat SobelSmoooth = getSobelSmoooth(wsize - 1);
	for (int k = 0; k < wsize; k++) {
		if (k == 0)
			Sobeldiffoper.at<float>(0, k) = 1;
		else if (k == wsize - 1)
			Sobeldiffoper.at<float>(0, k) = -1;
		else
			Sobeldiffoper.at<float>(0, k) = SobelSmoooth.at<float>(0, k) - SobelSmoooth.at<float>(0, k - 1);
	}
	return Sobeldiffoper;
}


//���ʵ��
void CU::conv2D(cv::Mat& src, cv::Mat& dst, cv::Mat kernel, int ddepth, cv::Point anchor, int delta, int borderType) {
	cv::Mat  kernelFlip;
	cv::flip(kernel, kernelFlip, -1);
	cv::filter2D(src, dst, ddepth, kernelFlip, anchor, delta, borderType);
}

//�ɷ������������ȴ�ֱ����������ˮƽ������
void CU::sepConv2D_Y_X(cv::Mat& src, cv::Mat& dst, cv::Mat kernel_Y, cv::Mat kernel_X, int ddepth, cv::Point anchor, int delta, int borderType) {
	cv::Mat dst_kernel_Y;
	conv2D(src, dst_kernel_Y, kernel_Y, ddepth, anchor, delta, borderType); //��ֱ������
	conv2D(dst_kernel_Y, dst, kernel_X, ddepth, anchor, delta, borderType); //ˮƽ������
}

//�ɷ�������������ˮƽ����������ֱ������
void CU::sepConv2D_X_Y(cv::Mat& src, cv::Mat& dst, cv::Mat kernel_X, cv::Mat kernel_Y, int ddepth, cv::Point anchor, int delta, int borderType) {
	cv::Mat dst_kernel_X;
	conv2D(src, dst_kernel_X, kernel_X, ddepth, anchor, delta, borderType); //ˮƽ������
	conv2D(dst_kernel_X, dst, kernel_Y, ddepth, anchor, delta, borderType); //��ֱ������
}

//Sobel���ӱ�Ե���
//dst_X ��ֱ����
//dst_Y ˮƽ����
void CU::Sobel(cv::Mat& src, cv::Mat& dst_X, cv::Mat& dst_Y, cv::Mat& dst, int wsize, int ddepth, cv::Point anchor, int delta, int borderType) {

	cv::Mat SobelSmooothoper = getSobelSmoooth(wsize); //ƽ��ϵ��
	cv::Mat Sobeldiffoper = getSobeldiff(wsize); //���ϵ��

	//�ɷ������������ȴ�ֱ����ƽ������ˮƽ�����֡����õ���ֱ��Ե
	sepConv2D_Y_X(src, dst_X, SobelSmooothoper.t(), Sobeldiffoper, ddepth);

	//�ɷ�������������ˮƽ����ƽ������ֱ�����֡����õ�ˮƽ��Ե
	sepConv2D_X_Y(src, dst_Y, SobelSmooothoper, Sobeldiffoper.t(), ddepth);

	//��Եǿ�ȣ����ƣ�
	dst = abs(dst_X) + abs(dst_Y);
	cv::convertScaleAbs(dst, dst); //�����ֵ��תΪ�޷���8λͼ
}

//ȷ��һ����������Ƿ���ͼ����
bool CU::checkInRang(int r, int c, int rows, int cols) {
	if (r >= 0 && r < rows && c >= 0 && c < cols)
		return true;
	else
		return false;
}

//��ȷ����Ե��������ӳ���Ե
void CU::trace(cv::Mat &edgeMag_noMaxsup, cv::Mat &edge, float TL, int r, int c, int rows, int cols) {
	if (edge.at<uchar>(r, c) == 0)
	{
		edge.at<uchar>(r, c) = 255;
		for (int i = -1; i <= 1; ++i) {
			for (int j = -1; j <= 1; ++j) {
				float mag = edgeMag_noMaxsup.at<float>(r + i, c + j);
				if (checkInRang(r + i, c + j, rows, cols) && mag >= TL)
					trace(edgeMag_noMaxsup, edge, TL, r + i, c + j, rows, cols);
			}
		}
	}
}

//Matת����
template<typename _Tp>
std::vector<_Tp> CU::convertMat2Vector(const cv::Mat &mat) {
	return (std::vector<_Tp>)(mat.reshape(1, 1));//ͨ�������䣬����תΪһ��
}

//vectorתMat
template<typename _Tp>
cv::Mat CU::convertVector2Mat(std::vector<_Tp> v, int channels, int rows) {
	cv::Mat mat = cv::Mat(v);//��vector��ɵ��е�mat
	cv::Mat dest = mat.reshape(channels, rows).clone();//PS������clone()һ�ݣ����򷵻س���
	return dest;
}

void CU::Edge_Canny(cv::Mat &src, cv::Mat &edge, float TL, float TH, int wsize, bool L2graydient) {
	int rows = src.rows;
	int cols = src.cols;

	//��˹�˲�
	cv::GaussianBlur(src, src, cv::Size(5, 5), 0.8);
	//sobel����
	cv::Mat dx, dy, sobel_dst;
	Sobel(src, dx, dy, sobel_dst, wsize, CV_32FC1);

	//�����ݶȷ�ֵ
	cv::Mat edgeMag;
	if (L2graydient)
		cv::magnitude(dx, dy, edgeMag); //��ƽ��
	else
		edgeMag = abs(dx) + abs(dy); //����ֵ֮�ͽ���

	//�����ݶȷ��� �Լ� �Ǽ���ֵ����
	cv::Mat edgeMag_noMaxsup = cv::Mat::zeros(rows, cols, CV_32FC1);// ����һ��CV_32FC1���͵�����
	for (int r = 1; r < rows - 1; ++r) {
		//�б���
		for (int c = 1; c < cols - 1; ++c) {
			//�б���
			float x = dx.at<float>(r, c);
			float y = dy.at<float>(r, c);
			float angle = std::atan2f(y, x) / CV_PI * 180; //��ǰλ���ݶȷ���
			float mag = edgeMag.at<float>(r, c);  //��ǰλ���ݶȷ�ֵ

			//�Ǽ���ֵ����
			//��ֱ��Ե--�ݶȷ���Ϊˮƽ����-3*3���������ҷ���Ƚ�
			if (abs(angle) < 22.5 || abs(angle) > 157.5) {
				float left = edgeMag.at<float>(r, c - 1);
				float right = edgeMag.at<float>(r, c + 1);
				if (mag >= left && mag >= right)
					edgeMag_noMaxsup.at<float>(r, c) = mag;
			}

			//ˮƽ��Ե--�ݶȷ���Ϊ��ֱ����-3*3���������·���Ƚ�
			if ((angle >= 67.5 && angle <= 112.5) || (angle >= -112.5 && angle <= -67.5)) {
				float top = edgeMag.at<float>(r - 1, c);
				float down = edgeMag.at<float>(r + 1, c);
				if (mag >= top && mag >= down)
					edgeMag_noMaxsup.at<float>(r, c) = mag;
			}

			//+45���Ե--�ݶȷ���Ϊ����������-3*3�������������·���Ƚ�
			if ((angle > 112.5 && angle <= 157.5) || (angle > -67.5 && angle <= -22.5)) {
				float right_top = edgeMag.at<float>(r - 1, c + 1);
				float left_down = edgeMag.at<float>(r + 1, c - 1);
				if (mag >= right_top && mag >= left_down)
					edgeMag_noMaxsup.at<float>(r, c) = mag;
			}


			//+135���Ե--�ݶȷ���Ϊ����������-3*3�������������Ϸ���Ƚ�
			if ((angle >= 22.5 && angle < 67.5) || (angle >= -157.5 && angle < -112.5)) {
				float left_top = edgeMag.at<float>(r - 1, c - 1);
				float right_down = edgeMag.at<float>(r + 1, c + 1);
				if (mag >= left_top && mag >= right_down)
					edgeMag_noMaxsup.at<float>(r, c) = mag;
			}
		}
	}

	std::vector<float> v = convertMat2Vector<float>(edgeMag_noMaxsup);//����ת���鷽��ʹ�ù�һ������

	cv::normalize(v, v, 1, 0, cv::NORM_MINMAX);//��һ������

	Result = convertVector2Mat<float>(v, 1, edgeMag_noMaxsup.rows);//����ת�������
}

float CU::cuVariance(const std::vector<TPoint> &coor, float sumsrc, const cv::Mat &src) {//��׼�����
	TPoint temp;
	float sum = 0;
	float avesrc = sumsrc / coor.size();//���ؾ�ֵ
	for (int i = 0; i < coor.size(); i++) {
		temp = coor[i];//��i�����ص������
		sum += pow(src.at<uchar>(temp.x, temp.y) - avesrc, 2);
	}
	return sqrt(sum / coor.size());
}

void CU::calculation(int v, PointSet * Set, float &Variance, float &aveCanny) {

	int rows = src.rows;
	int cols = src.cols;

	float sumCanny = 0;
	float sumsrc = 0;
	std::vector<TPoint> coor;//��������
	TPoint temp;//������ĳ��������
	int xx = 0, yy = 0;//������ĳ�����x,y

	/*****************�����λ�ȡ********************/
	int min_i = 999999999;
	int max_i = -999999999;
	int min_j = 999999999;
	int max_j = -999999999;

	int size = 0;//��������Ŀ
	std::vector<TPoint> triangles;

	int isize = Set->point.at(v).neighbor.size();//���һ����������
	for (int i = 0; i < isize; ++i) {
		for (int j = i + 1; j < isize; ++j) {
			int p_index = Set->point.at(v).neighbor[i];
			int p_index1 = Set->point.at(v).neighbor[j];
			if (Set->point.at(p_index).hasNeighbor(p_index1)) {//�ж��Ƿ��ܹ���������
				SPoint P = Set->point.at(p_index);
				SPoint Q = Set->point.at(p_index1);

				for (auto a : Set->point.at(v).SPointUV_index) {//�����õ��ÿһ�������
					for (auto b : P.SPointUV_index) {//����P���ÿһ�������
						for (auto c : Q.SPointUV_index) {//����Q���ÿһ�������
							SPoint_UV A = Set->point_uv.at(a);
							SPoint_UV B = Set->point_uv.at(b);
							SPoint_UV C = Set->point_uv.at(c);
							if (A.hasNeighbor(b) && A.hasNeighbor(c) && B.hasNeighbor(a) && B.hasNeighbor(c) && C.hasNeighbor(a) && C.hasNeighbor(b)) {//�ж��Ƿ��ܹ�������������
								size++;
								int it;
								int jt;
								uvtoij(A.PointUV_Vector[0], A.PointUV_Vector[1], it, jt);
								temp.x = it;
								temp.y = jt;
								triangles.push_back(temp);
								if (it > max_i) {
									max_i = it;
								}
								if (it < min_i) {
									min_i = it;
								}
								if (jt > max_j) {
									max_j = jt;
								}
								if (jt < min_j) {
									min_j = jt;
								}

								uvtoij(B.PointUV_Vector[0], B.PointUV_Vector[1], it, jt);
								temp.x = it;
								temp.y = jt;
								triangles.push_back(temp);
								if (it > max_i) {
									max_i = it;
								}
								if (it < min_i) {
									min_i = it;
								}
								if (jt > max_j) {
									max_j = jt;
								}
								if (jt < min_j) {
									min_j = jt;
								}

								uvtoij(C.PointUV_Vector[0], C.PointUV_Vector[1], it, jt);
								temp.x = it;
								temp.y = jt;
								triangles.push_back(temp);
								if (it > max_i) {
									max_i = it;
								}
								if (it < min_i) {
									min_i = it;
								}
								if (jt > max_j) {
									max_j = jt;
								}
								if (jt < min_j) {
									min_j = jt;
								}
							}
						}
					}
				}

			}
		}
	}
	/*****************�����λ�ȡ********************/

	for (int no = 0; no < size; no++) {
		triangle[0] = triangles[no * 3];
		triangle[1] = triangles[no * 3 + 1];
		triangle[2] = triangles[no * 3 + 2];

		for (int i = 0; i < rows; i++) {//��
			for (int j = 0; j < cols; j++) {//��
				P = { i ,j };
				if (i< max_i && i > min_i && j< max_j && j >min_j) {
					bool isintri = inside(triangle, P);//P���Ƿ����������ڲ�
					if (isintri) {
						sumCanny += Result.at<float>(i, j);//canny�ͼ���
						sumsrc += src.at<uchar>(i, j);//����ֵ�ͼ���
						coor.push_back(P);
					}
				}
			}
		}
	}
	Variance = cuVariance(coor, sumsrc, src);//��׼�����
	aveCanny = sumCanny / coor.size();//canny��ֵ����
}
//uvת��
void CU::uvtoij(float u, float v, int &i, int &j) {
	i = int(src.rows*(1 - v));
	j = int(src.cols*(u));
}
