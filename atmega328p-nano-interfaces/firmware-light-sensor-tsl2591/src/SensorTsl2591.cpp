#include "SensorTsl2591.h"
#include "JsonBuilder.h"

SensorTsl2591::SensorTsl2591() {
}

void SensorTsl2591::setup() {
    if (!m_tsl.begin()) {
        Serial.write("No TSL2591 sensor found. Check wiring.");
        while (1);
    }
    setGain(TSL2591_GAIN_MED);
    setTiming(TSL2591_INTEGRATIONTIME_600MS);
}

void SensorTsl2591::initiateUpdateData() {
    m_tsl.getFullLuminosityNonBlocking();
}

bool SensorTsl2591::isUpdatedDataReady() {
    return m_tsl.isFullLuminosityReady();
}

String SensorTsl2591::updateData(String& json) {
    uint32_t lum = m_tsl.getFullLuminosityNonBlockingData();
    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    uint32_t lux = m_tsl.calculateLux(full, ir);
    JsonBuilder s(json);
    JsonObject& datanode = s.get().createNestedObject("sensors").createNestedObject("tsl2591");
    datanode["full"] = full;
    datanode["ir"] = ir;
    datanode["lux"] = lux;
    return s.serialize();
}

bool SensorTsl2591::configure(String& json) {
    JsonBuilder s(json);
    JsonObject& root = s.get();
    bool configured = false;
    if (root.containsKey("tsl2591") && root["tsl2591"].is<JsonObject&>()) {
        JsonObject& sensor_root = root["tsl2591"].as<JsonObject&>();
        if (sensor_root.containsKey("gain")) {
            setGain(sensor_root["gain"].as<char*>());
            configured = true;
        }
        if (sensor_root.containsKey("timing")) {
            setTiming(sensor_root["timing"].as<char*>());
            configured = true;
        }
    }
    return configured;
}

void SensorTsl2591::setGain(tsl2591Gain_t gain) {
    m_tsl.setGain(gain);
}

void SensorTsl2591::setGain(const String& str) {
    while (!m_tsl.isFullLuminosityReady()); // Unable to carry out more than one operation on the device at one time
    if (str.compareTo("1") == 0) setGain(TSL2591_GAIN_LOW);
    else if (str.compareTo("25") == 0) setGain(TSL2591_GAIN_MED);
    else if (str.compareTo("428") == 0) setGain(TSL2591_GAIN_HIGH);
    else if (str.compareTo("9876") == 0) setGain(TSL2591_GAIN_MAX);

}

void SensorTsl2591::setTiming(tsl2591IntegrationTime_t timing) {
    m_tsl.setTiming(timing);
}

void SensorTsl2591::setTiming(const String& str) {
    while (!m_tsl.isFullLuminosityReady()); // Unable to carry out more than one operation on the device at one time
    if (str.compareTo("100") == 0) setTiming(TSL2591_INTEGRATIONTIME_100MS);
    else if (str.compareTo("200") == 0) setTiming(TSL2591_INTEGRATIONTIME_200MS);
    else if (str.compareTo("300") == 0) setTiming(TSL2591_INTEGRATIONTIME_300MS);
    else if (str.compareTo("400") == 0) setTiming(TSL2591_INTEGRATIONTIME_400MS);
    else if (str.compareTo("500") == 0) setTiming(TSL2591_INTEGRATIONTIME_500MS);
    else if (str.compareTo("600") == 0) setTiming(TSL2591_INTEGRATIONTIME_600MS);
}

String SensorTsl2591::gainToString(tsl2591Gain_t gain) {
    switch (gain) {
        case TSL2591_GAIN_LOW: return "1";
        case TSL2591_GAIN_MED: return "25";
        case TSL2591_GAIN_HIGH: return "428";
        case TSL2591_GAIN_MAX: return "9876";
        default: return "";
    };
}

String SensorTsl2591::timingToString(tsl2591IntegrationTime_t timing) {
    switch (timing) {
        case TSL2591_INTEGRATIONTIME_100MS: return "100";
        case TSL2591_INTEGRATIONTIME_200MS: return "200";
        case TSL2591_INTEGRATIONTIME_300MS: return "300";
        case TSL2591_INTEGRATIONTIME_400MS: return "400";
        case TSL2591_INTEGRATIONTIME_500MS: return "500";
        case TSL2591_INTEGRATIONTIME_600MS: return "600";
        default: return "";
    }
}

String SensorTsl2591::updateInfo(String& json) {
    while (!m_tsl.isFullLuminosityReady()); // Unable to carry out more than one operation on the device at one time
    sensor_t sensor;
    m_tsl.getSensor(&sensor);
    JsonBuilder s(json);
    JsonObject& datanode = s.get().createNestedObject("sensors").createNestedObject("tsl2591");
    datanode["sensor"] = sensor.name;
    datanode["driver_version"] = sensor.version;
    datanode["min_value"] = sensor.min_value;
    datanode["max_value"] = sensor.max_value;
    datanode["resolution"] = sensor.resolution;
    datanode["gain"] = gainToString(m_tsl.getGain());
    datanode["timing"] = timingToString(m_tsl.getTiming());
    return s.serialize();
}
