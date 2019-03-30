#include "commonPCH.h"
#include "Pose2ViscomStation.h"
#include "cvw/cAllViscoms.h"
#include "opencv2/core.hpp"
#include "Helper.h"
//-------------------------------------------------------------------------------------------------
unsigned long long Pose2ViscomStation::m_currentId = 0;
//-------------------------------------------------------------------------------------------------
Pose2ViscomStation::Pose2ViscomStation()
	: cStation("")
	, m_ipPose(this, &Pose2ViscomStation::callback, "pose")
	, m_opViscom(this, "viscom")
	, m_color(255, 0, 0)
	, m_id(m_currentId++)
	, m_drawOrientation(true)
	, m_drawCovPosition(true)
{
	m_param["color"].setRefData(m_color);
	m_param["drawCovPosition"].setRefData(m_drawCovPosition);
	m_param["drawOrientation"].setRefData(m_drawOrientation);
}
//-------------------------------------------------------------------------------------------------
Pose2ViscomStation::~Pose2ViscomStation() {}
//-------------------------------------------------------------------------------------------------
void Pose2ViscomStation::callback(cBufferT<vePose>* i_buffer)
{
	if (m_opViscom.requestsData())
	{
		// Draw the position itself
		auto pose = i_buffer->getData();
		auto twodPose = pose.get2DPose();
		std::string name = "poseViscomPosition" + std::to_string(m_id);
		m_opViscom.send(cViscomPtr(new cDeleteViscom(name.c_str())), i_buffer->getStamp());
		m_opViscom.send(cViscomPtr(new cCircleViscom<sint32>(cCircle<sint32>(twodPose[0], twodPose[1], 2), name.c_str(),
			QPen(Qt::transparent), m_color)),
			i_buffer->getStamp());

		// Draw orientation as arrow
		std::string nameOrientation = "poseViscomOrientation" + std::to_string(m_id);
		m_opViscom.send(cViscomPtr(new cDeleteViscom(nameOrientation.c_str())), i_buffer->getStamp());
		if(m_drawOrientation)
		{
			cRotation rotation{ c3DDblPoint{ 0.0, 0.0, 1.0 }, twodPose.getZ() };
			c3DDblPoint orientationDir;
			rotation.transform(c3DDblPoint{ 1.0, 0.0, 0.0 }, orientationDir);
			c2DDblPoint twodOrientationEnd = c2DDblPoint{ twodPose } + c2DDblPoint{ 10 * orientationDir.getX(), 10 * orientationDir.getY() };
			m_opViscom.send(cViscomPtr(new cArrowViscom(nameOrientation.c_str(),
				QPointF{ twodPose[0], twodPose[1] },
				QPointF{ twodOrientationEnd.getX(), twodOrientationEnd.getY() },
				cArrowViscom::ArrowPos::ArrowEnd, QPen(m_color))), i_buffer->getStamp());
		}

		std::string name2 = "poseViscomCovariance" + std::to_string(m_id);
		m_opViscom.send(cViscomPtr(new cDeleteViscom(name2.c_str())), i_buffer->getStamp());
		if (m_drawCovPosition)
		{
			// Draw the covariance matrix as 95% confidence ellipsis
			cv::Mat covMat = toCv(pose.getCovPosition());

			// Calculate the ellipsis' rotation angle
			cv::Mat eigenvalues;
			cv::Mat eigenvectors;
			eigen(covMat, eigenvalues, eigenvectors);
			cv::Mat v = eigenvectors.row(0);
			double angle = atan2f(v.at<double>(0, 1), v.at<double>(0, 0)) * 180.0 / CV_PI;

			// Calculate the ellipsis' axes for 95% confidence interval
			double lambda1 = eigenvalues.at<double>(0);
			double lambda2 = eigenvalues.at<double>(1);
			double x_axis = sqrt(5.991 * lambda1);
			double y_axis = sqrt(5.991 * ((lambda2 == 0.0) ? lambda1 : lambda2));

			m_opViscom.send(cViscomPtr(new cEllipseViscom<sint32>(cEllipse<sint32>(twodPose[0], twodPose[1], x_axis, y_axis, angle), name2.c_str(),
				QPen(m_color))),
				i_buffer->getStamp());
		}
	}
}
//-------------------------------------------------------------------------------------------------

STATION(Pose2ViscomStation, "semesterprojekt/common/Pose2Viscom");
