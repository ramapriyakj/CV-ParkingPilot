#include "lidarPCH.h"
#include "LidarStation.h"

using namespace cv;
//-------------------------------------------------------------------------------------------------
LidarStation::LidarStation()
	: cStation("")
	, m_ipMap(this, &LidarStation::mapCallback, "map")
	, m_ipData(this, &LidarStation::dataCallback, "gridmap", "egomotion")
	, m_optestImg(this, "test")
	, m_opMap(this, "map")
	, m_opPose(this, "pose")
	, m_opPosetest(this, "poseTest")
	, m_opPath(this, "Path")
	,m_opMaxPose(this,"maxPose")
	, size_factor(400)   //factor of size for lidar video frame size input
	, mapsize_factor(400)
	, init_position_factor({ 366, 509 })
	, init_orientation_factor(5.0/6.0*CV_PI)
	, dAngle_factor(1.0/6.0*CV_PI)
{
	m_ipMap.getParams<0>().required = true;
	m_ipMap.registerOutPorts(QList<cOPortBase*>());

	m_ipData.getParams<0>().required = true;
	m_ipData.getParams<1>().required = true;
	m_ipData.registerOutPorts(QList<cOPortBase*>({ &m_opMap, &m_opPose, &m_optestImg, &m_opPosetest, &m_opPath,&m_opMaxPose }));

	m_param["size"].setRefData(size_factor);
	m_param["size"]->connectChangedSignal(this, &LidarStation::ManualSize, false);
	m_param["map_size"].setRefData(mapsize_factor);
	m_param["map_size"]->connectChangedSignal(this, &LidarStation::ManualSize, false);
	m_param["Init_Position"].setRefData(init_position_factor);
	m_param["Init_Position"]->connectChangedSignal(this, &LidarStation::ManualSize, false);
	m_param["init_Orientation"].setRefData(init_orientation_factor);
	m_param["init_Orientation"]->connectChangedSignal(this, &LidarStation::ManualSize, false);
	m_param["Search_Angle_Resolution"].setRefData(dAngle_factor);
	m_param["Search_Angle_Resolution"]->connectChangedSignal(this, &LidarStation::ManualSize, false);
}
//-------------------------------------------------------------------------------------------------
LidarStation::~LidarStation()
{
}
//-------------------------------------------------------------------------------------------------
void LidarStation::stop(StopReasons reasons) 
{
	if (!(reasons & StopReason::Pause))
	{
		// TODO: Do everything here to reset your station, since it was not paused but was stopped or has a discontinuity
		//       in time (e.g. someone moved the slider or someone set the player back to the beginning)
	}
}
//-------------------------------------------------------------------------------------------------
void LidarStation::mapCallback(cBufferT<cImg>* buffer) 
{
	m_map = buffer->getData();
}
//-------------------------------------------------------------------------------------------------
void LidarStation::dataCallback()
{
	if (m_map.isValid()) {
		frameCount++;
		if (frameCount == 1) {
			initialize();  //Initialize: initial orientation
		}

		cImg gridMap = m_ipData.getValue<0>();
		veEgomotion egomotion = m_ipData.getValue<1>();
		cBufferStamp stamp = m_ipData.getStamp();

		if (!(frameCount% SampleRate) && frameCount > 20)   // Only process part of the frames for speeding up
		{
			//cTracer::cout << "Frame: " << frameCount << TREND;
			cImg imgTemp(cImgType::ImageTypes::UInt8, size_factor, size_factor);

			c2DPoint<sint32> grid_topLeftPoint, map_topLeftPoint, targetPosition{ 0,0 };
			cRect<sint32> grid_rect, map_rect;
			grid_topLeftPoint.setX(1000 - 0.5*size_factor);  // Set the topleftpoint of the rect
			grid_topLeftPoint.setY(1000 - 0.5*size_factor);
			grid_rect.setTopLeft(grid_topLeftPoint);
			grid_rect.setHeight(size_factor);
			grid_rect.setWidth(size_factor);

			c2DPoint<sint32> mapcropBase{ 0,0 };
			if (mapCropMode) {
				if (Position_map.getX() > 10) {
					mapcropBase.setX(Position_map.getX());
					mapcropBase.setY(Position_map.getY());
				}
				else {
					mapcropBase.setX(init_position_factor.getX());
					mapcropBase.setY(init_position_factor.getY());
				}
				if ((mapcropBase.getX() - 0.5*mapsize_factor) < 0)  map_topLeftPoint.setX(0);
				else map_topLeftPoint.setX(mapcropBase.getX() - 0.5*mapsize_factor);
				if ((mapcropBase.getY() - 0.5*mapsize_factor) < 0)  map_topLeftPoint.setY(0);
				else map_topLeftPoint.setY(mapcropBase.getY() - 0.5*mapsize_factor);

				map_rect = { map_topLeftPoint, QSize{ mapsize_factor, mapsize_factor } };
			}
			else map_rect = cRect<sint32>{ c2DPoint<sint32>{0,0}, QSize{ m_map.getSize().width() + 1, m_map.getSize().height() + 1 } };

			imgTemp.copyFrom(gridMap, grid_rect, targetPosition);

			cImg mapTemp(cImgType::ImageTypes::UInt8, map_rect.getWidth(), map_rect.getHeight());
			mapTemp.copyFrom(m_map, map_rect, targetPosition);
			//	m_optestImg.send(imgTemp, stamp);
		
			float lastYaw = Position_map.getOrientation()[2];
			float angleRange[2]{1.5*CV_PI - lastYaw - dAngle_factor , 1.5*CV_PI - lastYaw + dAngle_factor };

			MapMatch matching(&mapTemp, &imgTemp, angleRange);
			matching.Template_match();

			QVector<c3DPoint<double>> maxPose = matching.getMaxPose();

			QVector<c2DPoint<double>> maxPose2d(5);
			for (int i = 0; i < maxPose.size(); i++) {

				maxPose2d[i].setX(maxPose[i].getX());
				maxPose2d[i].setY(maxPose[i].getY());
				c2DPoint<double> delta_Pose{ maxPose2d[i].getX() - map_rect.getWidth() / 2.0, maxPose2d[i].getY() - map_rect.getHeight() / 2.0 };  //Relative pose on cropped map
				maxPose2d[i].setX(delta_Pose.getX() + mapcropBase.getX());
				maxPose2d[i].setY(delta_Pose.getY() + mapcropBase.getY());
			}

			pose = calculate_curPose(maxPose2d,maxPose);



			//pose.setX(delta_Pose.getX() + mapcropBase.getX());
			//pose.setY(delta_Pose.getY() + mapcropBase.getY());
			//cTracer::cout << " Real position on map" << pose << TREND;
			cImg testImg = matching.getTestImage();
			//m_optestImg.send(testImg, stamp);
			//m_optestImg.send(m_map, stamp);
			m_opMaxPose.send(maxPose2d, stamp);
			m_opPose.send(pose, stamp);
		}
		else {
			update_egoPose(egomotion, pose);
		//	cTracer::cout << " In between sampled frames position on map" << pose << TREND;
			vePose poseOnMap = calculate_egoPose(egomotion);
			
			//Send data
			pose.setZ(-1.0);		//Always set pose to invalid before sending invalid data!
			pose.setRollAngle(-1.0);  
			m_opPose.send(pose, stamp);  //Send pose from updating using egomotion
			//m_opPosetest.send(poseOnMap, stamp);
			m_opPath.send(Path, stamp);
		}
		m_opMap.send(m_map, stamp);
		m_opPath.send(Path, stamp);  // Draw real path by ego motion on map
	}
}
//-------------------------------------------------------------------------------------------------
void LidarStation::initialize() {
	Position_map.setYawAngle(init_orientation_factor);
	pose.setX(init_position_factor.getX());
	pose.setY(init_position_factor.getY());
	pose.setYawAngle(init_orientation_factor);

}

vePose LidarStation::calculate_curPose(QVector<c2DPoint<double>> maxPose2d, QVector<c3DPoint<double>> maxPose) {
	vePose position;
	

	//pose = matching.getPose();
	//c2DPoint<float64> delta_Pose{ pose.getX() - map_rect.getWidth()/2.0, pose.getY() - map_rect.getHeight()/2.0 };  //Relative pose on cropped map


	int maxNumber = maxPose2d.size();
	//Mat posX = Mat::zeros(1, maxNumber, CV_64FC1);
	//Mat posY = Mat::zeros(1, maxNumber, CV_64FC1);


	Mat pos = Mat::zeros(2, maxNumber, CV_64FC1);
	Mat orien = Mat::zeros(1, maxNumber, CV_64FC1);

	for (int i = 0; i < maxNumber; i++) {
		pos.at<double>(0, i) = maxPose2d.at(i).getX();
		pos.at <double>(1, i) = maxPose2d.at(i).getY();
		orien.at<double>(0, i) = maxPose.at(i).getZ();
		//cTracer::cout << pos.at<double>(0, i) << " " << pos.at<double>(1, i) << TREND;
	}


	//vconcat(posX, posY, pos);
	Mat_<double> covar, mean;
	calcCovarMatrix(pos, covar, mean, CV_COVAR_COLS | CV_COVAR_NORMAL);
	covar /= 2;

	//Mat orien = Mat::zeros(1, maxNumber, CV_64FC1);
	Mat ori_m, ori_sd;
	meanStdDev(orien, ori_m, ori_sd);

	double ori_mean, ori_sd1;
	ori_mean = ori_m.at<double>(0, 0);
	ori_sd1 = ori_sd.at<double>(0, 0);


	cMatrix<float64> posCov(3, 3);
	posCov(0, 2) = 0;
	posCov(1, 2) = 0;
	posCov(2, 2) = 0;
	posCov(2, 0) = 0;
	posCov(2, 1) = 0;
	posCov(0, 0) = covar.at<double>(0, 0);
	posCov(0, 1) = covar.at<double>(0, 1);
	posCov(1, 0) = covar.at<double>(1, 0);
	posCov(1, 1) = covar.at<double>(1, 1);

	cMatrix<float64> oriCov(3, 3);
	oriCov(0, 0) = 0;
	oriCov(0, 1) = 0;
	oriCov(0, 2) = 0;
	oriCov(1, 0) = 0;
	oriCov(1, 1) = 0;
	oriCov(1, 2) = 0;
	oriCov(2, 0) = 0;
	oriCov(2, 1) = 0;
	oriCov(2, 2) = ori_sd1;


	position.setCovOrientation(oriCov);
	position.setX(mean.at<double>(0, 0));
	position.setY(mean.at<double>(0, 1));
	position.setZ(0);
	position.setYawAngle(ori_mean);
	position.setCovPosition(posCov);



	return position;
}

vePose LidarStation::calculate_egoPose(veEgomotion egomotion) {

	cVector<float64> updatePoseOrientation   // Add current pose to last pose, and orientation to globle position
	{ 0.0,0.0,Position_map.getYawAngle() - egomotion.getDeltaTime()*EGO_COMPENSATOR * egomotion.getRotationRates()[2] };

	cVector<float64> updatePosePosition
	{ PositionfromEgo.getX() + egomotion.getMovement3D().getX()*PIXELPERMETER * EGO_COMPENSATOR* cos(updatePoseOrientation[2]),
		PositionfromEgo.getY() + egomotion.getMovement3D().getX() * PIXELPERMETER*EGO_COMPENSATOR *sin(updatePoseOrientation[2]), 0 };  

	PositionfromEgo.setOrientation(updatePoseOrientation); //Obsolute angle
	PositionfromEgo.setPosition(updatePosePosition); // Relative Position to starting point

	Position_map.setOrientation(updatePoseOrientation);
	Position_map.setPosition({ PositionfromEgo.getX() + init_position_factor.getX(),
		PositionfromEgo.getY() + init_position_factor.getY(), 0.0 });
	Path.append(c2DPoint<double>{ Position_map.getX(), Position_map.getY() });
	return Position_map;
}

void LidarStation::update_egoPose(veEgomotion egomotion, vePose &inputPose) {
	cVector<float64> updatePoseOrientation  
	{ 0.0,0.0,inputPose.getYawAngle() - egomotion.getDeltaTime()*EGO_COMPENSATOR * egomotion.getRotationRates()[2] };

	cVector<float64> updatePosePosition
	{ inputPose.getX() + egomotion.getMovement3D().getX()*PIXELPERMETER * EGO_COMPENSATOR* cos(updatePoseOrientation[2]),
		inputPose.getY() + egomotion.getMovement3D().getX() * PIXELPERMETER*EGO_COMPENSATOR *sin(updatePoseOrientation[2]), 0 };
	inputPose.setPosition(updatePosePosition);
	inputPose.setOrientation(updatePoseOrientation);

}

void LidarStation::ManualSize() {

	if (mapsize_factor<0) mapCropMode = 0;  //Do not crop map if mapsize set to -1
	else mapCropMode = 1;
	cTracer::cout << "Video size: " << size_factor << " Map size" << mapsize_factor << " Initial Position" << init_position_factor.getX() << " " << init_position_factor.getY() << " Initial Orientation" << init_orientation_factor <<" Angle Searching range: " <<dAngle_factor/CV_PI*180.0<< " Crop Map?: " << mapCropMode << TREND;
}
//-------------------------------------------------------------------------------------------------

STATION(LidarStation, "semesterprojekt/lidar/Lidar");