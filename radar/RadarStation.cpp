#include "radarPCH.h"
#include "RadarStation.h"
#include "RadarLib.h"
#include "ConvertImgFormat.h"

#define ROTATIONS 36
#define FREQUENCY 10
#define BEST_POS_COUNT 1

//-------------------------------------------------------------------------------------------------
RadarStation::RadarStation()
	: cStation("")
	, m_ipMap(this, &RadarStation::mapCallback, "map")
	, m_ipData(this, &RadarStation::dataCallback, "gridmap")
	, m_opMap(this, "map")
	, m_opPose(this, "pose")
{
	m_ipMap.getParams<0>().required = true;
	m_ipMap.registerOutPorts(QList<cOPortBase*>());
	m_ipData.getParams<0>().required = true;
	//m_ipData.getParams<1>().required = true;
	m_ipData.registerOutPorts(QList<cOPortBase*>({ &m_opMap, &m_opPose }));

}
//-------------------------------------------------------------------------------------------------
RadarStation::~RadarStation()
{
}
void RadarStation::mapCallback(cBufferT<cImg>* i_buffer)
{
	m_map = i_buffer->getData();
}
//-------------------------------------------------------------------------------------------------
void RadarStation::dataCallback()
{
	ConvertImgFormat convertImg;
	if (m_map.isValid()) {
		cImg gridMap = m_ipData.getValue<0>();
		Mat radarFrame, groundTruthMap;
		radarFrame = convertImg.convertToCv(&gridMap);
		if (flag == 0)
		{
			flag = 1;
			 string bestRotatedFramesFile = "";
			// Call this test function to create the file to store the data of possible locations ( BEST_POS_COUNT = 1)
			//string bestRotatedFramesFile = Radar::createBestRotatedframesFile("F:/TU_Berlin/study/AutonomousDriving/Data/output/");
			
			groundTruthMap = convertImg.convertToCv(&m_map);
			cvtColor(groundTruthMap, groundTruthMap, CV_RGB2GRAY);
			cvtColor(groundTruthMap, groundTruthMap, CV_GRAY2RGB);
			
			Radar::setStaticMembers(groundTruthMap, ROTATIONS, FREQUENCY, BEST_POS_COUNT, bestRotatedFramesFile);
		}
		Radar radarLibObj(radarFrame,0);

		vePose position;
		Point carPosition;
		float angle;
		
		cMatrix<float64> covMatPos(3, 3);
		covMatPos(0, 0) = 100;
		covMatPos(0, 1) = 0;
		covMatPos(0, 2) = 0;
		covMatPos(1, 0) = 0;
		covMatPos(1, 1) = 100;
		covMatPos(1, 2) = 0;
		covMatPos(2, 0) = 0;
		covMatPos(2, 1) = 0;
		covMatPos(2, 2) = 1;

		cMatrix<float64> covMatOri(3, 3);
		covMatOri(0, 0) = 1;
		covMatOri(0, 1) = 0;
		covMatOri(0, 2) = 0;
		covMatOri(1, 0) = 0;
		covMatOri(1, 1) = 1;
		covMatOri(1, 2) = 0;
		covMatOri(2, 0) = 0;
		covMatOri(2, 1) = 0;
		covMatOri(2, 2) = 1;
			
		// get one best position. We can also get an array of BEST_POS_COUNT best positions.
		carPosition = radarLibObj.getCarPosition();
		angle = radarLibObj.getOrientation();

		position.setX(carPosition.x);
		position.setY(carPosition.y);
		position.setYawAngle(angle);
		position.setCovPosition(covMatPos);
		position.setCovOrientation(covMatOri);


		m_opMap.send(m_map, m_ipData.getStamp());
		m_opPose.send(position, m_ipData.getStamp());
	
	}
}




//-------------------------------------------------------------------------------------------------

STATION(RadarStation, "semesterprojekt/radar/Radar");