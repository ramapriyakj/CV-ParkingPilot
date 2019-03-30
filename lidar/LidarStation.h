#pragma once

#include "MapMatch.h"

class LidarStation : public cStation
{
	STATIONDECL(LidarStation);
	LidarStation();
	virtual ~LidarStation();
	int size_factor;
	int mapsize_factor;
	c2DPoint<sint32> init_position_factor;
	float init_orientation_factor;
	float dAngle_factor;


	void stop(StopReasons reasons) override;

	void mapCallback(cBufferT<cImg>* buffer);

	/*
	* Call back for lidar video and ego motion
	*/
	void dataCallback();
	/*
	* Call back for lidar raw data
	*/
	void initialize();
	vePose calculate_egoPose(veEgomotion egomotion);
	vePose calculate_curPose(QVector<c2DPoint<double>> maxPose2d, QVector<c3DPoint<double>> maxPose);

	void update_egoPose(veEgomotion egomotion, vePose &inputPose);

private:
	uint32 frameCount = 0;
	uint8 SampleRate = 5;   //Every "SampleRate" frame get processed
	uint8 SampleCount = 0;
	uint8 mapCropMode = 0;
	uint8 init_Flag = 1;
	uint8 EGO_COMPENSATOR = 3;   //Compensator for dropping ego motion data
	float PIXELPERMETER = 5.0;

	veEgomotion lastEgomotion;
	vePose pose{ -1.0, -1.0 , 0.0, 0.0, 0.0, 0.0 };
	vePose PositionfromEgo{ -1.0, -1.0 , 0.0, 0.0, 0.0, 0.0 };  //Calculated global position from egomotion
	vePose Position_map{ -1.0, -1.0 , 0.0, 0.0, 0.0, 0.0 };; //Final output position on map
	QVector<c2DPoint<double>> Path;

	void ManualSize();

	PORT(cImg)						m_ipMap;
	PORTGROUP(cImg, veEgomotion)	m_ipData;

	cOPort<cImg>					m_opMap;
	cOPort<vePose>					m_opPose;
	cOPort<vePose>					m_opPosetest;
	cOPort<cImg>                    m_optestImg;  //Output test image
	cOPort<QVector<c2DPoint<double>>> m_opPath;
	cOPort < QVector<c2DPoint<double>>> m_opMaxPose;
	cImg														m_map;
	cImg                                                        testImg;
};