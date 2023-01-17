#pragma once
#include "CU.h"

CU::CU(cv::String _address)
{
	src = cv::imread(_address);
	if (src.empty()) {
		return;
	}
	if (src.channels() > 1) cv::cvtColor(src, src, CV_RGB2GRAY);//转灰度图像src 
	Edge_Canny(src, Canny, 50, 100);

}
CU::~CU()
{
}

float CU::mul(TPoint p1, TPoint p2, TPoint p0)
{
	return ((p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y));
}

/*由三个顶点向所求的点引出矢量（注意方向），然后任意用其中两个矢量形成平面，
 * 再用这个平面和剩下的矢量叉乘，得出一个新矢量，方向向里，则在三角形外，反之在里面。
 */
int CU::inside(TPoint tr[], TPoint p)
{
	int i;
	for (i = 0; i < 3; i++)
		if (mul(p, tr[i], tr[(i + 1) % 3]) * mul(p, tr[(i + 2) % 3], tr[(i + 1) % 3]) > 0)
			return 0;
	return 1;
}

//sobel算子 /
//阶乘
int CU::factorial(int n) {
	int fac = 1;
	//0的阶乘
	if (n == 0)
		return fac;
	for (int i = 1; i <= n; ++i) {
		fac *= i;
	}
	return fac;
}

//获得Sobel平滑算子
cv::Mat CU::getSobelSmoooth(int wsize) {
	int n = wsize - 1;
	cv::Mat SobelSmooothoper = cv::Mat::zeros(cv::Size(wsize, 1), CV_32FC1);
	for (int k = 0; k <= n; k++) {
		float *pt = SobelSmooothoper.ptr<float>(0);
		pt[k] = factorial(n) / (factorial(k)*factorial(n - k));
	}
	return SobelSmooothoper;
}
//获得Sobel差分算子
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


//卷积实现
void CU::conv2D(cv::Mat& src, cv::Mat& dst, cv::Mat kernel, int ddepth, cv::Point anchor, int delta, int borderType) {
	cv::Mat  kernelFlip;
	cv::flip(kernel, kernelFlip, -1);
	cv::filter2D(src, dst, ddepth, kernelFlip, anchor, delta, borderType);
}

//可分离卷积———先垂直方向卷积，后水平方向卷积
void CU::sepConv2D_Y_X(cv::Mat& src, cv::Mat& dst, cv::Mat kernel_Y, cv::Mat kernel_X, int ddepth, cv::Point anchor, int delta, int borderType) {
	cv::Mat dst_kernel_Y;
	conv2D(src, dst_kernel_Y, kernel_Y, ddepth, anchor, delta, borderType); //垂直方向卷积
	conv2D(dst_kernel_Y, dst, kernel_X, ddepth, anchor, delta, borderType); //水平方向卷积
}

//可分离卷积———先水平方向卷积，后垂直方向卷积
void CU::sepConv2D_X_Y(cv::Mat& src, cv::Mat& dst, cv::Mat kernel_X, cv::Mat kernel_Y, int ddepth, cv::Point anchor, int delta, int borderType) {
	cv::Mat dst_kernel_X;
	conv2D(src, dst_kernel_X, kernel_X, ddepth, anchor, delta, borderType); //水平方向卷积
	conv2D(dst_kernel_X, dst, kernel_Y, ddepth, anchor, delta, borderType); //垂直方向卷积
}

//Sobel算子边缘检测
//dst_X 垂直方向
//dst_Y 水平方向
void CU::Sobel(cv::Mat& src, cv::Mat& dst_X, cv::Mat& dst_Y, cv::Mat& dst, int wsize, int ddepth, cv::Point anchor, int delta, int borderType) {

	cv::Mat SobelSmooothoper = getSobelSmoooth(wsize); //平滑系数
	cv::Mat Sobeldiffoper = getSobeldiff(wsize); //差分系数

	//可分离卷积———先垂直方向平滑，后水平方向差分——得到垂直边缘
	sepConv2D_Y_X(src, dst_X, SobelSmooothoper.t(), Sobeldiffoper, ddepth);

	//可分离卷积———先水平方向平滑，后垂直方向差分——得到水平边缘
	sepConv2D_X_Y(src, dst_Y, SobelSmooothoper, Sobeldiffoper.t(), ddepth);

	//边缘强度（近似）
	dst = abs(dst_X) + abs(dst_Y);
	cv::convertScaleAbs(dst, dst); //求绝对值并转为无符号8位图
}

//确定一个点的坐标是否在图像内
bool CU::checkInRang(int r, int c, int rows, int cols) {
	if (r >= 0 && r < rows && c >= 0 && c < cols)
		return true;
	else
		return false;
}

//从确定边缘点出发，延长边缘
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

//Mat转数组
template<typename _Tp>
std::vector<_Tp> CU::convertMat2Vector(const cv::Mat &mat) {
	return (std::vector<_Tp>)(mat.reshape(1, 1));//通道数不变，按行转为一行
}

//vector转Mat
template<typename _Tp>
cv::Mat CU::convertVector2Mat(std::vector<_Tp> v, int channels, int rows) {
	cv::Mat mat = cv::Mat(v);//将vector变成单列的mat
	cv::Mat dest = mat.reshape(channels, rows).clone();//PS：必须clone()一份，否则返回出错
	return dest;
}

void CU::Edge_Canny(cv::Mat &src, cv::Mat &edge, float TL, float TH, int wsize, bool L2graydient) {
	int rows = src.rows;
	int cols = src.cols;

	//高斯滤波
	cv::GaussianBlur(src, src, cv::Size(5, 5), 0.8);
	//sobel算子
	cv::Mat dx, dy, sobel_dst;
	Sobel(src, dx, dy, sobel_dst, wsize, CV_32FC1);

	//计算梯度幅值
	cv::Mat edgeMag;
	if (L2graydient)
		cv::magnitude(dx, dy, edgeMag); //开平方
	else
		edgeMag = abs(dx) + abs(dy); //绝对值之和近似

	//计算梯度方向 以及 非极大值抑制
	cv::Mat edgeMag_noMaxsup = cv::Mat::zeros(rows, cols, CV_32FC1);// 创建一个CV_32FC1类型的数组
	for (int r = 1; r < rows - 1; ++r) {
		//行遍历
		for (int c = 1; c < cols - 1; ++c) {
			//列遍历
			float x = dx.at<float>(r, c);
			float y = dy.at<float>(r, c);
			float angle = std::atan2f(y, x) / CV_PI * 180; //当前位置梯度方向
			float mag = edgeMag.at<float>(r, c);  //当前位置梯度幅值

			//非极大值抑制
			//垂直边缘--梯度方向为水平方向-3*3邻域内左右方向比较
			if (abs(angle) < 22.5 || abs(angle) > 157.5) {
				float left = edgeMag.at<float>(r, c - 1);
				float right = edgeMag.at<float>(r, c + 1);
				if (mag >= left && mag >= right)
					edgeMag_noMaxsup.at<float>(r, c) = mag;
			}

			//水平边缘--梯度方向为垂直方向-3*3邻域内上下方向比较
			if ((angle >= 67.5 && angle <= 112.5) || (angle >= -112.5 && angle <= -67.5)) {
				float top = edgeMag.at<float>(r - 1, c);
				float down = edgeMag.at<float>(r + 1, c);
				if (mag >= top && mag >= down)
					edgeMag_noMaxsup.at<float>(r, c) = mag;
			}

			//+45°边缘--梯度方向为其正交方向-3*3邻域内右上左下方向比较
			if ((angle > 112.5 && angle <= 157.5) || (angle > -67.5 && angle <= -22.5)) {
				float right_top = edgeMag.at<float>(r - 1, c + 1);
				float left_down = edgeMag.at<float>(r + 1, c - 1);
				if (mag >= right_top && mag >= left_down)
					edgeMag_noMaxsup.at<float>(r, c) = mag;
			}


			//+135°边缘--梯度方向为其正交方向-3*3邻域内右下左上方向比较
			if ((angle >= 22.5 && angle < 67.5) || (angle >= -157.5 && angle < -112.5)) {
				float left_top = edgeMag.at<float>(r - 1, c - 1);
				float right_down = edgeMag.at<float>(r + 1, c + 1);
				if (mag >= left_top && mag >= right_down)
					edgeMag_noMaxsup.at<float>(r, c) = mag;
			}
		}
	}

	std::vector<float> v = convertMat2Vector<float>(edgeMag_noMaxsup);//矩阵转数组方便使用归一化函数

	cv::normalize(v, v, 1, 0, cv::NORM_MINMAX);//归一化函数

	Result = convertVector2Mat<float>(v, 1, edgeMag_noMaxsup.rows);//数组转矩阵输出
}

float CU::cuVariance(const std::vector<TPoint> &coor, float sumsrc, const cv::Mat &src) {//标准差计算
	TPoint temp;
	float sum = 0;
	float avesrc = sumsrc / coor.size();//像素均值
	for (int i = 0; i < coor.size(); i++) {
		temp = coor[i];//第i个像素点的坐标
		sum += pow(src.at<uchar>(temp.x, temp.y) - avesrc, 2);
	}
	return sqrt(sum / coor.size());
}

void CU::calculation(int v, PointSet * Set, float &Variance, float &aveCanny) {

	int rows = src.rows;
	int cols = src.cols;

	float sumCanny = 0;
	float sumsrc = 0;
	std::vector<TPoint> coor;//像素坐标
	TPoint temp;//三角形某个点坐标
	int xx = 0, yy = 0;//三角形某个点的x,y

	/*****************三角形获取********************/
	int min_i = 999999999;
	int max_i = -999999999;
	int min_j = 999999999;
	int max_j = -999999999;

	int size = 0;//三角形数目
	std::vector<TPoint> triangles;

	int isize = Set->point.at(v).neighbor.size();//点的一环领域点个数
	for (int i = 0; i < isize; ++i) {
		for (int j = i + 1; j < isize; ++j) {
			int p_index = Set->point.at(v).neighbor[i];
			int p_index1 = Set->point.at(v).neighbor[j];
			if (Set->point.at(p_index).hasNeighbor(p_index1)) {//判断是否能构成三角面
				SPoint P = Set->point.at(p_index);
				SPoint Q = Set->point.at(p_index1);

				for (auto a : Set->point.at(v).SPointUV_index) {//遍历该点的每一个纹理点
					for (auto b : P.SPointUV_index) {//遍历P点的每一个纹理点
						for (auto c : Q.SPointUV_index) {//遍历Q点的每一个纹理点
							SPoint_UV A = Set->point_uv.at(a);
							SPoint_UV B = Set->point_uv.at(b);
							SPoint_UV C = Set->point_uv.at(c);
							if (A.hasNeighbor(b) && A.hasNeighbor(c) && B.hasNeighbor(a) && B.hasNeighbor(c) && C.hasNeighbor(a) && C.hasNeighbor(b)) {//判断是否能构成纹理三角面
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
	/*****************三角形获取********************/

	for (int no = 0; no < size; no++) {
		triangle[0] = triangles[no * 3];
		triangle[1] = triangles[no * 3 + 1];
		triangle[2] = triangles[no * 3 + 2];

		for (int i = 0; i < rows; i++) {//行
			for (int j = 0; j < cols; j++) {//列
				P = { i ,j };
				if (i< max_i && i > min_i && j< max_j && j >min_j) {
					bool isintri = inside(triangle, P);//P点是否在三角形内部
					if (isintri) {
						sumCanny += Result.at<float>(i, j);//canny和计算
						sumsrc += src.at<uchar>(i, j);//像素值和计算
						coor.push_back(P);
					}
				}
			}
		}
	}
	Variance = cuVariance(coor, sumsrc, src);//标准差计算
	aveCanny = sumCanny / coor.size();//canny均值计算
}
//uv转换
void CU::uvtoij(float u, float v, int &i, int &j) {
	i = int(src.rows*(1 - v));
	j = int(src.cols*(u));
}
