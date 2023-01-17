// DOM_LOD.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。


#include <iostream>
#include <QDialog>
#include "DOMlod.h"

int main()
{
	QString FileOriPath = "E://M4//SIM//DOM.obj"; //DOM file path
	QString FileSavePath = "E://LOD";//LOD model output path

	DOMLod lod(FileOriPath.toLocal8Bit().toStdString(), FileSavePath.toLocal8Bit().toStdString());
	lod.buildDOMLod_Quadtree();//program execution
    std::cout << "success!\n";
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

