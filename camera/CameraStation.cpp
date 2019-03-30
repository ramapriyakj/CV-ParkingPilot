#include "cameraPCH.h"
#include "CameraStation.h"

cv::Mat CameraStation::toMat(cImg* img)
{
	auto dest = cv::Mat3b(img->getHeight(), img->getWidth());
	uchar* pptr;
	for (uint32 y = 0; y < img->getHeight(); y++)
	{
		for (uint32 x = 0; x < img->getWidth(); x++)
		{
			pptr = img->getPixel<uchar>(x, y);
			dest.at<cv::Vec3b>(y, x) = cv::Vec3b(*pptr, *(pptr + 1), *(pptr + 2));
		}
	}
	return dest;
}

//-------------------------------------------------------------------------------------------------
CameraStation::CameraStation()
	: cStation("")
	, m_ipMap(this, &CameraStation::mapCallback, "map")
	, m_ipData(this, &CameraStation::dataCallback, "image")
	, m_opMap(this, "map")
	, m_opPose(this, "pose")
	, m_startPose(0, 0, 0)
	, SLAM("ORBvoc.txt", "hella.yaml", ORB_SLAM2::System::MONOCULAR, true)
{
	m_ipMap.getParams<0>().required = true;
	m_ipMap.registerOutPorts(QList<cOPortBase*>());
	m_ipData.getParams<0>().required = true;
	m_param["Start Pose"].setRefData(m_startPose);
	m_ipData.registerOutPorts(QList<cOPortBase*>({ &m_opMap, &m_opPose }));
}
//-------------------------------------------------------------------------------------------------
CameraStation::~CameraStation()
{
}
//-------------------------------------------------------------------------------------------------
void CameraStation::mapCallback(cBufferT<cImg>* i_buffer)
{
	m_map = i_buffer->getData();
}
//-------------------------------------------------------------------------------------------------
void CameraStation::dataCallback(cBufferT<cImg>* i_buffer)
{
	cImg image = i_buffer->getData();
	//veEgomotion egomotion = m_ipData.getValue<1>();


	//cTracer::cout << TREND;
	//cTracer::cout << egomotion.getDeltaTime() << TREND;
	//cTracer::cout << egomotion.getVelocity() << TREND;

	// Process image with monocular SLAM
	cv::Mat Tcw = SLAM.TrackMonocular(toMat(&image), time);
	time += 0.033;

	cDblMatrix covPosition{ 3, 3,{ 200, 0, 0, 0, 200, 0, 0, 0, 1 } };
	cDblMatrix covOrientation{ 3, 3, {1.0, 0, 0, 0, 1.0, 0, 0, 0, 1000.0} };

	// extract pose only if SLAM is in localization mode
	if (SLAM.GetTrackingState() == 2)
	{
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);
		float scale = 180;
		float px = twc.at<float>(2) * scale;
		float py = twc.at<float>(0) * scale * 2.5;
		float r = m_startPose.getZ();
		float x = px * cos(r) - py * sin(r);
		float y = px * sin(r) + py * cos(r);

		m_pose.setX(m_startPose.getX() + x);
		m_pose.setY(m_startPose.getY() + y);

		// todo : get yaw from twc matrix
		m_pose.setYawAngle(m_startPose.getZ());

		m_pose.setCovPosition(covPosition);
		m_pose.setCovOrientation(covOrientation);
	}

	// send map + pose at same time to enable viscom render
	cTracer::cout << "Cam pose: " << m_pose.getX() << ", " << m_pose.getY() << " | " << m_pose.getYawAngle() << TREND;
	m_opPose.send(m_pose, m_ipData.getStamp());
	m_opMap.send(m_map, m_ipData.getStamp());

	// TODO : encapsulate in safe call
	if (m_map.isValid() && m_opMap.requestsData())
	{

	}
}
//-------------------------------------------------------------------------------------------------



STATION(CameraStation, "semesterprojekt/camera/Camera");