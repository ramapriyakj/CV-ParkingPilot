#include "fusionPCH.h"
#include "KalmanFusion.h"
#include "../common/Helper.h"

using namespace cv;

KalmanFusion::KalmanFusion()
	: m_orientationFilter((Mat_<float64>{ 2, 1 } << 0.0, 1.0),
	(Mat_<float64>{ 2, 2 } << 1000.0, 0.0, 0.0, 0.0), // Initially extremly uncertain about the orientation itself.
						  (Mat_<float64>{ 1, 2 } << 1.0, 0.0))
	, m_positionFilter((Mat_<float64>{ 3, 1 } << 0.0, 0.0, 1.0), 
	(Mat_<float64>{ 3, 3 } << 1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 0.0),
					   (Mat_<float64>{ 2, 3 } << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0))
{
}

void KalmanFusion::reset()
{
	// Reset the filters
	m_positionFilter.reset();
	m_orientationFilter.reset();
}

vePose KalmanFusion::fuse(const vePose & cameraPose, const vePose & lidarPose, const vePose & radarPose, const veEgomotion & egomotion)
{
	vePose pose;

	fuseOrientation(cameraPose, lidarPose, radarPose, egomotion, pose);
	fusePosition(cameraPose, lidarPose, radarPose, egomotion, pose);

	return pose;
}

void KalmanFusion::fuseOrientation(const vePose & cameraPose, const vePose & lidarPose, const vePose & radarPose, const veEgomotion & egomotion, vePose & pose)
{
	// Current movement model
	if (egomotion.getDeltaTime() > 0.0)
	{
		//cTracer::cout << -egomotion.getRotationRates()[2] << TREND;
		float64 dt = egomotion.getDeltaTime();
		Mat Fk = (Mat_<double>(2, 2) << 1.0, dt * -egomotion.getRotationRates()[2], 0.0, 1.0);

		// Somewhat uncertain about the prediction of the angle .. use covariance from egomotion?
		Mat Qk = (Mat_<double>(2, 2) << 0.01, 0.0, 0.0, 0.0);

		m_orientationFilter.predict(Fk, Qk);
	}

	// Updates
	updateOrientation(cameraPose);
	updateOrientation(lidarPose);
	updateOrientation(radarPose);

	// Set the orientation and covariance
	Mat x = m_orientationFilter.getX();
	cDblVector _x{ 0.0, 0.0, x.at<double>(0) };
	pose.setOrientation(_x);

	Mat P = m_orientationFilter.getP();
	cDblMatrix _P{ 3, 3 };
	_P(2, 2) = P.at<double>(0, 0);
	pose.setCovOrientation(_P);
}

void KalmanFusion::updateOrientation(const vePose & measuredPose)
{
	// Skip invalid orientation measurements
	if (hasInvalidOrientation(measuredPose))
	{
		return;
	}

	Mat zk = (Mat_<double>(1, 1) << measuredPose.getOrientation()[2]);
	Mat Rk = (Mat_<double>(1, 1) << measuredPose.getCovOrientation()(2, 2));

	m_orientationFilter.update(zk, Rk);
}

void KalmanFusion::fusePosition(const vePose & cameraPose, const vePose & lidarPose, const vePose & radarPose, const veEgomotion & egomotion, vePose & pose)
{
	// Prediction
	if (egomotion.getDeltaTime() > 0.0)
	{
		float64 phi = pose.getOrientation().getZ();

		cDblMatrix R{ 3, 3,
		{ cos(phi) , -sin(phi) , 0.0,
		  sin(phi), cos(phi), 0.0,
		  0.0, 0.0, 1.0 } };

		auto v = R * egomotion.getVelocity3D() * PIXEL_PER_METER;

		// Current movement model
		float64 dt = egomotion.getDeltaTime();
		Mat Fk = (Mat_<double>(3, 3) << 1.0, 0.0, dt * v[0],
				  0.0, 1.0, dt * v[1],
				  0.0, 0.0, 1.0);

		// Somewhat uncertain about the prediction of the position .. use covariance from egomotion?
		Mat Qk = (Mat_<double>(3, 3) << 0.5, 0.0, 0.0,
				  0.0, 0.5, 0.0,
				  0.0, 0.0, 0.0);

		m_positionFilter.predict(Fk, Qk);
	}

	updatePosition(cameraPose);
	updatePosition(lidarPose);
	updatePosition(radarPose);

	// Set the position and covariance
	Mat x = m_positionFilter.getX();
	cDblVector _x{ x.at<double>(0), x.at<double>(1), 0.0 };
	pose.setPosition(_x);

	Mat P = m_positionFilter.getP();
	cDblMatrix _P{ 3, 3 };
	_P(0, 0) = P.at<double>(0, 0);
	_P(0, 1) = P.at<double>(0, 1);
	_P(1, 0) = P.at<double>(1, 0);
	_P(1, 1) = P.at<double>(1, 1);
	pose.setCovPosition(_P);
}

void KalmanFusion::updatePosition(const vePose & measuredPose)
{
	// Skip invalid position measurements
	if (hasInvalidPosition(measuredPose))
	{
		return;
	}

	auto p = measuredPose.getPosition();
	auto cov = measuredPose.getCovPosition();

	Mat zk = (Mat_<double>(2, 1) << p[0], p[1]);
	Mat Rk = (Mat_<double>(2, 2) << cov(0, 0), cov(0, 1), cov(1, 0), cov(1, 1));

	m_positionFilter.update(zk, Rk);
}


