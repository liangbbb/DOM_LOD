# DOM_LOD
A High-Efficiency LOD Reconstruction and Visualization Method for LiDAR-derived 3D Digital Outcrop Model

1. Introduction

This program uses a quadtree-based LOD organization strategy and a new edge collapse simplification algorithm to build the digital outcrop tile pyramid model. Using a regular grid + pseudo-quadtree, we build a tile spatial index. Additionally, we combine the node evaluation function and PagedLOD technology to dynamically load model tiles. The multi-scale and fast visualization of large-scale DOM in the OSG platform is made possible by creating a digital outcrop LOD model.

![image](https://user-images.githubusercontent.com/100745194/213050723-a0a22aec-c244-4391-b6c9-a5b6a89a9d63.png)

Figure: Rendering effect of model 1 (LOD) at different sight distance levels. (a) Level 10; (b) Level 8; (c) Level 6; (d) Level 4; (e) Level 2.

2. Install

Prerequisites:

   OSG 3.6
   
   QT 5.9.1
   
   OpenCV (The latest version can support this project.)
   
   Visual studio 2017
   
note: This project is a C++ console application based on VS platform. The project has been fully uploaded to https://github.com/liangbbb/DOM_LOD.git.

Project directory structure:

DOM_LOD

├── DOM_LOD

│   ├── DOM_LOD.vcxproj

│   ├── DOM_LOD.vcxproj.filters

│   ├── DOM_LOD. vcxproj.user

│   ├── opencv_world320d.dll

│   ├── zlibd.dll

│   ├── CU.h

│   ├── CU.cpp

│   ├── DOM_LOD.h

│   ├── DOM_LOD.cpp

│   ├── Face.h

│   ├── Face.cpp

│   ├── FitPlane.h

│   ├── FitPlane.cpp

│   ├── global.h

│   ├── global.cpp

│   ├── gocReadDom.h

│   ├── gocReadDom.cpp

│   ├── gocTriHeader.h

│   ├── MyManipulator.h

│   ├── MyManipulator.cpp

│   ├── Pair.h

│   ├── Pair.cpp

│   ├── PairHeap.h

│   ├── PairHeap.cpp

│   ├── Point.h

│   ├── Point.cpp

│   ├── PointSet.h

│   ├── PointSet.cpp

│   ├── QuadDOMNode.h

│   ├── QuadDOMTree.h

│   ├── QuadDOMTree.cpp

│   ├── Simplification.h

│   ├── Simplification.cpp

├── x64

│   ├── Debug/

├── DOM_LOD.sln


3. Datasets

The project supports OBJ format digital outcrop monomer model.

4. Run

The relevant parameters in the LOD reconstruction method are set as follows. 

QString FileOriPath= "E://…//DOM.obj";  //DOM file path

QString FileSavePath = "E://LOD";       //LOD model output path

The tile data threshold Volumemax=1MB; 

Mesh simplification ratio θ= 0.8; 

Texture thinning ratio α=0.2; 

The number of inserted smooth layers j=3.

5. Acknowledgement

The authors would like to express their gratitude to the PetroChina Research Institute of Petroleum Exploration & Development for providing the resources required to collect the samples used in this study. This research was funded by the National Natural Science Foundation of China (No. 41701537,
No. 42172172).


