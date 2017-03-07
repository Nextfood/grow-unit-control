#ifndef __Tsl2591Sensor_h__
#define __Tsl2591Sensor_h__

#include <map>
#include "InterfaceSerialConnection.h"

class Tsl2591Sensor {
public:
	Tsl2591Sensor(InterfaceSerialConnection& isc);
	void verifyDeviceId();
	void update();

	double getFull() const;
	double getIr() const;
	double getLux() const;

    std::string getInfo(const std::string& key) const;
private:
	InterfaceSerialConnection& m_isc;
	double m_full,m_ir,m_lux;
	std::map<std::string, std::string> m_info;
};

#endif 

