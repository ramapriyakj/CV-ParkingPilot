#pragma once

class RadarStation : public cStation
{
	STATIONDECL(RadarStation);
	RadarStation();
	virtual ~RadarStation();

	void mapCallback(cBufferT<cImg>* i_buffer);
	void dataCallback();
	bool flag = 0;
	
	PORT(cImg)					 m_ipMap;
	PORT(cImg)					 m_ipData;
	cOPort<cImg>				 m_opMap;
	cOPort<vePose>				 m_opPose;

	cImg m_map;
};