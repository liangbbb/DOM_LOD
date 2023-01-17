#pragma once
#include <osg/Viewport>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>

//ÂþÓÎ¿ØÖÆÆ÷
class CMyManipulator : public osgGA::TrackballManipulator {
public:
	CMyManipulator(int flags = DEFAULT_SETTINGS);
	CMyManipulator(const TrackballManipulator& tm,
		const osg::CopyOp& copyOp = osg::CopyOp::SHALLOW_COPY);
	~CMyManipulator() {}
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	void lock();
	bool isLock();
	void setStep(double dStep);
private:
	bool   _lock;
	double _dStep;
};
