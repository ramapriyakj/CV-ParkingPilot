#include "commonPCH.h"
#include "Egomotion2Viscom.h"

#include "cvw/cAllViscoms.h"
//-------------------------------------------------------------------------------------------------
unsigned long long Egomotion2ViscomStation::m_currentId = 0;
//-------------------------------------------------------------------------------------------------
Egomotion2ViscomStation::Egomotion2ViscomStation()
	: cStation("")
	, m_ipEgomotion(this, &Egomotion2ViscomStation::callback, "egomotion")
	, m_opViscom(this, "viscom")
	, m_id(m_currentId++)
{
}
//-------------------------------------------------------------------------------------------------
Egomotion2ViscomStation::~Egomotion2ViscomStation() {}
//-------------------------------------------------------------------------------------------------
void Egomotion2ViscomStation::callback(cBufferT<veEgomotion>* i_buffer)
{
	if (m_opViscom.requestsData())
	{
		const veEgomotion& egom = i_buffer->getData();

		// Draw the velocity text
		m_opViscom.send(cViscomPtr(new cDeleteViscom(name("velocity"))), i_buffer->getStamp());
		m_opViscom.send(cViscomPtr(new cTextViscom(QVariant(("Velocity: " + std::to_string(egom.getVelocity()) + " m/s").c_str()), name("velocity"),
			c2DDblPoint{ 20.0f, 20.0f }, 
			QPen{ Qt::white })), i_buffer->getStamp());

		// Draw the acceleration text
		m_opViscom.send(cViscomPtr(new cDeleteViscom(name("acceleration"))), i_buffer->getStamp());
		m_opViscom.send(cViscomPtr(new cTextViscom(QVariant(("Acceleration: " + std::to_string(egom.getAcceleration()) + " m/s^2").c_str()), name("acceleration"),
			c2DDblPoint{ 20.0f, 30.0f },
			QPen{ Qt::white })), i_buffer->getStamp());

		// Draw the angular velocity text
		m_opViscom.send(cViscomPtr(new cDeleteViscom(name("rotationrate"))), i_buffer->getStamp());
		m_opViscom.send(cViscomPtr(new cTextViscom(QVariant(("RotRate: " + std::to_string(egom.getRotationRates().getZ()) + " rad/s").c_str()), name("rotationrate"),
			c2DDblPoint{ 20.0f, 40.0f },
			QPen{ Qt::white })), i_buffer->getStamp());
	}
}
//-------------------------------------------------------------------------------------------------

STATION(Egomotion2ViscomStation, "semesterprojekt/common/Egomotion2Viscom");

