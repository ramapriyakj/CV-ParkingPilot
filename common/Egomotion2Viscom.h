#pragma once

class Egomotion2ViscomStation : public cStation {
public:
	STATIONDECL(Egomotion2ViscomStation);
	Egomotion2ViscomStation();
	virtual ~Egomotion2ViscomStation();

	void callback(cBufferT<veEgomotion>* i_buffer);

	PORT(veEgomotion)	m_ipEgomotion;
	cOPort<cViscomPtr>	m_opViscom;

private:
	inline QString name(const std::string& name)
	{
		return ("Egomotion2Viscom" + std::to_string(m_id) + name).c_str();
	}

	static unsigned long long m_currentId;
	unsigned long long        m_id;
};