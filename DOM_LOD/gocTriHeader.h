#pragma once
//定义网格文件格式
//头区
//三角形记录
//组

typedef struct TRI_HEADER_t
{
	unsigned long long nTriangleNum;  //三角形数
	enum Type {
		triangle = 0,
		triangle_index,
		triangle_with_texture,
		triangle_index_with_texture
	};
	unsigned int nType;
	unsigned int nMeanLen;  //三角形边长的均值*1000
	unsigned int nGroupNum;
}TRI_HEADER;

//一三角形顶点集
typedef struct isTriangleEqual {
	float  x1[3];
	float  x2[3];
	float  x3[3];
	float  normal[3];
}Triangle_Record;

//index of vertex , the data is in an extern PTS file
typedef struct _Triangle_Index_Record_t {
	unsigned int x1_index;
	unsigned int x2_index;
	unsigned int x3_index;
	unsigned int normal[3];
}Triangle_Index_Record;

//带纹理坐标的三角形顶点
typedef struct _Triangle_Record_Index_WithTexture_t {
	unsigned int x1_index;
	unsigned int x2_index;
	unsigned int x3_index;
	float x1[3];
	float x2[3];
	float x3[3];
	float normal[3];
	float groupNum;  //落入的组号 
	float tex_x1[2];
	float tex_x2[2];
	float tex_x3[2];
}Triangle_Record_With_Index_Texture;

//顶点索引，数据存放在文件中
typedef struct _Triangle_Index_Record_WithTexture_t {
	unsigned int x1_index;
	unsigned int x2_index;
	unsigned int x3_index;
	float	normal[3];
	unsigned int groupNum;    //_texture_file_t矩阵中的索引
	float tex_x1[2];
	float tex_x2[2];
	float tex_x3[2];
}Triangle_Index_Record_With_Texture;

//带纹理帶三角形顶点编号 三角形顶点
typedef struct _Triangle_Record_WithTexture_t {
	float x1[3];
	float x2[3];
	float x3[3];
	float normal[3];
	float groupNum;  //落入的组号 
	float tex_x1[2];
	float tex_x2[2];
	float tex_x3[2];
}Triangle_Record_With_Texture;

typedef struct _Triangle_Index_Record_WithTexture_index {
	unsigned int x1_index;
	unsigned int x2_index;
	unsigned int x3_index;
	unsigned int groupNum;    //_texture_file_t矩阵中的索引
	unsigned int x1t_index;
	unsigned int x2t_index;
	unsigned int x3t_index;
	unsigned int x1n_index;
	unsigned int x2n_index;
	unsigned int x3n_index;
}TriangleIndex_Record_With_TextureIndex;


