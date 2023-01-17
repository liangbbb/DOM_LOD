#include "MyManipulator.h"

void CMyManipulator::lock() {
	_lock = !_lock;
}

bool CMyManipulator::isLock() {
	return _lock;
}

void CMyManipulator::setStep(double dStep) {
	_dStep = dStep;
}

CMyManipulator::CMyManipulator(int flags) :TrackballManipulator(flags) {
	_lock = false;
	_dStep = 1.0;
}

CMyManipulator::CMyManipulator(const TrackballManipulator& tm,
	const osg::CopyOp& copyOp) : TrackballManipulator(tm, copyOp) {
	_lock = false;
	_dStep = 1.0;
}

bool CMyManipulator::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) {
	if (_lock) {
		switch (ea.getEventType()) {
		case osgGA::GUIEventAdapter::SCROLL:
			break;
		case (osgGA::GUIEventAdapter::KEYDOWN): {
			switch (ea.getKey()) {
			case osgGA::GUIEventAdapter::KEY_Up:
			{
				osg::Vec3d pt = _center;
				pt._v[2] -= _dStep;
				_center.set(pt);
			}
			break;
			case osgGA::GUIEventAdapter::KEY_Down:
			{
				osg::Vec3d pt = _center;
				pt._v[2] += _dStep;
				_center.set(pt);
			}
			break;
			case osgGA::GUIEventAdapter::KEY_Left:
			{
				osg::Vec3d pt = _center;
				pt._v[0] += _dStep;
				_center.set(pt);
			}
			break;
			case osgGA::GUIEventAdapter::KEY_Right:
			{
				osg::Vec3d pt = _center;
				pt._v[0] -= _dStep;
				_center.set(pt);
			}
			break;
			}
		}//KEYDOWN
		default:
			return true;
		}
	}//_lock

	return osgGA::TrackballManipulator::handle(ea, aa);
}
