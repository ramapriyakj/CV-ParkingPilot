#include "lidarPCH.h"
#include "testStation.h"
//-------------------------------------------------------------------------------------------------
testStation::testStation() :
	cStation(""),
	m_ipData(this, &testStation::callback, "egoMotion"),
	m_opData(this, "Pose"),
	m_opData2(this, "CalculatedPose"),
	m_dtOut(this, "dt"),
	m_outPose(this, "pose"),
	m_opPath(this, "Path"),
	lastPosition(0, 0),
	magnitute_factor(1),
	startPoint_factor(350, 537),
	magnitute_factor2(1),
	startPoint_factor2(0,0)
	{
		m_param["startPoint"].setRefData(startPoint_factor);
		m_param["startPoint"]->connectChangedSignal(this, &testStation::startPointChange, false);
		m_param["magnitute"].setRefData(magnitute_factor);
		m_param["magnitute"]->connectChangedSignal(this, &testStation::magnituteChange, false);

		m_param["cal_startPoint"].setRefData(startPoint_factor2);
		m_param["cal_startPoint"]->connectChangedSignal(this, &testStation::m_startPointChange, false);
		m_param["cal_magnitute"].setRefData(magnitute_factor2);
		m_param["cal_magnitute"]->connectChangedSignal(this, &testStation::m_magnituteChange, false);
	}
//-------------------------------------------------------------------------------------------------
testStation::~testStation()
{
}
//-------------------------------------------------------------------------------------------------
void testStation::callback(cBufferT<veEgomotion> *i_buffer)
{
	
	veEgomotion egoIn = i_buffer->getData();
	vePose pose = egoIn.toPose();
	cVector<float64> poseOut = egoIn.getMovement3D();
	c2DPoint<double> position;
	c2DPoint<double> dPosition;
	c2DPoint<double> realPosition;

	dPosition.setX((double)poseOut.getX()*magnitute_factor2);
	dPosition.setY((double)poseOut.getY()*magnitute_factor2);

	position.setX((double)poseOut.getX()*magnitute_factor +startPoint_factor.getX());
	position.setY ((double)poseOut.getY()*magnitute_factor + startPoint_factor.getY());

	realPosition.setX((double)dPosition.getX() + lastPosition.getX());				// Related position of starting point
	realPosition.setY((double)dPosition.getY() + lastPosition.getY());
	lastPosition = realPosition;

	realPosition.setX((double)realPosition.getX() + startPoint_factor2.getX());      // Add starting point to output (absoute position)
	realPosition.setY((double)realPosition.getY() + startPoint_factor2.getY());

	points.insert(points.size(), position);
	m_points.insert(m_points.size(), realPosition);

    m_opData.send(points, i_buffer->getStamp());
	m_opData.send(m_points, i_buffer->getStamp());
	dT = i_buffer->getStamp() - lastTimeStamp;
	lastTimeStamp = i_buffer->getStamp();
	m_dtOut.send(dT, i_buffer->getStamp());

	cTracer::cout << "Orientation: " << pose.getOrientation() << " Pose" << pose.get2DPose() << TREND;
	m_outPose.send(pose, i_buffer->getStamp());

	vePose poseOnMap = calculate_egoPose(egoIn);

	m_opPath.send(Path, i_buffer->getStamp());
	
}


vePose testStation::calculate_egoPose(veEgomotion egomotion) {
	vePose Position_map; //Final output position on map

	cVector<float64> updatePoseOrientation   // Add current pose to last pose, and orientation as well
	{ 0.0,0.0,PositionfromEgo.getYawAngle() - egomotion.getDeltaTime() * egomotion.getRotationRates()[2] };
	//cVector<float64> updatePosePosition
	//{ PositionfromEgo.getX() - egomotion.getMovement3D().getX()*5.0 * sin(updatePoseOrientation[2]),
	//	PositionfromEgo.getY() + egomotion.getMovement3D().getX() * 5.0*cos(updatePoseOrientation[2]), 0 };  //Map: 0.2m/pixel
	cVector<float64> updatePosePosition
	{ PositionfromEgo.getX() + egomotion.getMovement3D().getX()*5.0 * cos(updatePoseOrientation[2]),
		PositionfromEgo.getY() + egomotion.getMovement3D().getX() * 5.0*sin(updatePoseOrientation[2]), 0 };  //Map: 0.2m/pixel

																											 //Pure testing
																											 //cVector<float64> updatePosePosition
																											 //{ PositionfromEgo.getX() + egomotion.getMovement3D().getX()*5.0 ,
																											 //	PositionfromEgo.getY() + egomotion.getMovement3D().getX() * 5.0, 0 };  //Map: 0.2m/pixel
																											 //

	PositionfromEgo.setOrientation(updatePoseOrientation); // Relative Position to starting point
	PositionfromEgo.setPosition(updatePosePosition);

	Position_map.setOrientation({ 0.0, 0.0, PositionfromEgo.getYawAngle()  });
	Position_map.setPosition({ PositionfromEgo.getX() + startPoint_factor.getX(),
		PositionfromEgo.getY() + startPoint_factor.getY(), 0.0 });
	Path.append(c2DPoint<double>{ Position_map.getX(), Position_map.getY() });
	return Position_map;
}

void testStation::startPointChange() {
	cTracer::cout << "The starting point of existing pose has been changed to: " << startPoint_factor << TREND;
}

void testStation::magnituteChange() {
	cTracer::cout << "The magnitute of existing pose has been changed to: " << magnitute_factor << TREND;
}

void testStation::m_startPointChange() {
	cTracer::cout << "The starting point of calculated pose has been changed to: " << startPoint_factor2 << TREND;
}

void testStation::m_magnituteChange() {
	cTracer::cout << "The magnitute of calculated pose has been changed to: " << magnitute_factor2 << TREND;
}
//-----------------------------------------------------------------------------------------
STATION(testStation, "semesterprojekt/egoMotion2Pose");
