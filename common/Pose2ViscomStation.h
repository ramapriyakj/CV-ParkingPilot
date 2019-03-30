#pragma once

class Pose2ViscomStation : public cStation {
public:
	STATIONDECL(Pose2ViscomStation);
	Pose2ViscomStation();
	virtual ~Pose2ViscomStation();

	void callback(cBufferT<vePose>* i_buffer);

	PORT(vePose)		m_ipPose;
	cOPort<cViscomPtr>	m_opViscom;

private:
	static unsigned long long m_currentId;
	unsigned long long        m_id;
	QColor				      m_color;
	bool				      m_drawCovPosition;
	bool				      m_drawOrientation;
};