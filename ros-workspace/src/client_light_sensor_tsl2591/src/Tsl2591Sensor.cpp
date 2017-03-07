
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "Tsl2591Sensor.h"
#include "JsonTools.h"

Tsl2591Sensor::Tsl2591Sensor(InterfaceSerialConnection& isc) : m_isc(isc),m_full(0),m_ir(0),m_lux(0) {
	verifyDeviceId();
}

void Tsl2591Sensor::verifyDeviceId() {
	m_info.clear();

	boost::property_tree::ptree pt;
	pt.put("cmd", "info");
	auto respPt = JsonTools::jsonToPtree(m_isc.query(JsonTools::ptreeToJson(pt)));

	for (auto it: respPt) {
		m_info[it.first] = it.second.data();
	}
	
	if (m_info.find("id") == m_info.end())
		throw std::runtime_error("Unable to verify the device ID of the interface. Was expecting 'TSL2591' but received nothing.");

	if(m_info["id"].compare("TSL2591") != 0)
		throw std::runtime_error(std::string("Unable to verify the device ID of the interface. Was expecting 'TSL2591' but got '") + m_info["id"]+"'.");


	std::cout << "Found device: " << m_info["id"] << " (ID: " << m_info["unique_id"] << ")" << std::endl;
}

void Tsl2591Sensor::update() {
	try {
		boost::property_tree::ptree pt;
		pt.put("cmd", "data");
		auto respPt = JsonTools::jsonToPtree(m_isc.query(JsonTools::ptreeToJson(pt)));
		m_full = respPt.get<double>("full");
		m_ir = respPt.get<double>("ir");
		m_lux = respPt.get<double>("lux");
	} catch (const boost::property_tree::ptree_error& e) {
		throw std::runtime_error("Exception occurred JSON during parsing: " + std::string(e.what()));
	}
}

double Tsl2591Sensor::getFull() const {
	return m_full;
}
double Tsl2591Sensor::getIr() const{
	return m_ir;
}
double Tsl2591Sensor::getLux() const{
	return m_lux;
}



std::string Tsl2591Sensor::getInfo(const std::string& key) const {
	auto itr = m_info.find(key);
	if(itr == m_info.end()) return "";
	return (*itr).second;
}

