#pragma once

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <vector>
#include "PointSet.h"


struct TPoint {
	int x;
	int y;
};


class CU {

public:
	CU(cv::String _address);
	~CU();
	void calculation(int v, PointSet * Set, float &Variance, float &aveCanny);
	void Edge_Canny(cv::Mat &src, cv::Mat &edge, float TL, float TH, int wsize = 3, bool L2graydient = false);
	struct TPoint triangle[3], P;
private:

	cv::Mat Result;//canny归一化后的矩阵
	cv::Mat src;//原图矩阵
	cv::Mat Canny;//canny矩阵

	float mul(struct TPoint p1, struct TPoint p2, struct TPoint p0);
	int inside(struct TPoint tr[], struct TPoint p);//是否在三角形内
	int factorial(int n);//sobel算子 /
	cv::Mat getSobelSmoooth(int wsize);
	cv::Mat getSobeldiff(int wsize);
	void conv2D(cv::Mat& src, cv::Mat& dst, cv::Mat kernel, int ddepth, cv::Point anchor = cv::Point(-1, -1), int delta = 0, int borderType = cv::BORDER_DEFAULT);
	void sepConv2D_Y_X(cv::Mat& src, cv::Mat& dst, cv::Mat kernel_Y, cv::Mat kernel_X, int ddepth, cv::Point anchor = cv::Point(-1, -1), int delta = 0, int borderType = cv::BORDER_DEFAULT);
	void sepConv2D_X_Y(cv::Mat& src, cv::Mat& dst, cv::Mat kernel_X, cv::Mat kernel_Y, int ddepth, cv::Point anchor = cv::Point(-1, -1), int delta = 0, int borderType = cv::BORDER_DEFAULT);
	void Sobel(cv::Mat& src, cv::Mat& dst_X, cv::Mat& dst_Y, cv::Mat& dst, int wsize, int ddepth, cv::Point anchor = cv::Point(-1, -1), int delta = 0, int borderType = cv::BORDER_DEFAULT);
	bool checkInRang(int r, int c, int rows, int cols);
	void trace(cv::Mat &edgeMag_noMaxsup, cv::Mat &edge, float TL, int r, int c, int rows, int cols);
	template<typename _Tp>
	std::vector<_Tp> convertMat2Vector(const cv::Mat &mat);
	template<typename _Tp>
	cv::Mat convertVector2Mat(std::vector<_Tp> v, int channels, int rows);
	float cuVariance(const std::vector<TPoint> &coor, float sumsrc, const cv::Mat &src);
	void uvtoij(float u, float v, int &i, int &j);
};


