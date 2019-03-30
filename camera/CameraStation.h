#pragma once

#include "cdl/cStation.h"
#include "cbl/cMathOP.h"
#include "cil/cImgConvertFunctor.h"
#include "cil/cMinMaxFunctor.h"
#include "cil/cImgMirrorFunctor.h"
#include "cvw/cMainWindow.h"
#include "cvw/cViscom.h"
#include "cvw/cDeleteViscom.h"
#include "veFramework/Types/vePose.h"
#include <QPainter>
#include <QPaintEngine>

#include <System.h>

class CameraStation : public cStation
{
	STATIONDECL(CameraStation);
	CameraStation();
	virtual ~CameraStation();

	void mapCallback(cBufferT<cImg>* i_buffer);
	void dataCallback(cBufferT<cImg>* i_buffer);

	// utils
	cv::Mat toMat(cImg* img);

	// variables
	double time{ 0.0 };
	ORB_SLAM2::System SLAM;

	// pose only uses <x, y, yaw> attributes & yaw.metric = radians
	vePose m_pose{ 0.0, 0.0, -1.0, -1.0, 0.0, 0.0 };
	// x = x, y = y, z = yaw
	c3DPoint<double> m_startPose;

	PORT(cImg)					 m_ipMap;
	PORTGROUP(cImg)              m_ipData;
	cOPort<cImg>				 m_opMap;
	cOPort<vePose>				 m_opPose;

	cImg m_map;
};