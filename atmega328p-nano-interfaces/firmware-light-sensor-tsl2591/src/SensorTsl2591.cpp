#include "SensorTsl2591.h"
#include "JsonBuilder.h"

SensorTsl2591::SensorTsl2591() {
}

void SensorTsl2591::setup() {
    if (!m_tsl.begin()) {
        Serial.println(F("No TSL2591 sensor found. Check wiring."));
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

String& SensorTsl2591::updateData(String& json) {
    uint32_t lum = m_tsl.getFullLuminosityNonBlockingData();
    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    uint32_t lux = m_tsl.calculateLux(full, ir);
    JsonBuilder s(json);
    JsonObject& datanode = s.get().createNestedObject(F("sensors")).createNestedObject(F("tsl2591"));
    datanode["full"] = full;
    datanode["ir"] = ir;
    datanode["lux"] = lux;
    return s.serialize(json);
}

bool SensorTsl2591::configure(String& json) {
    JsonBuilder s(json);
    JsonObject& root = s.get();
    bool configured = false;
    if (root.containsKey(F("tsl2591")) && root[F("tsl2591")].is<JsonObject&>()) {
        JsonObject& sensor_root = root[F("tsl2591")].as<JsonObject&>();
        if (sensor_root.containsKey(F("gain"))) {
            setGain((tsl2591Gain_t) sensor_root[F("gain")].as<int>());
            configured = true;
        }
        if (sensor_root.containsKey(F("timing"))) {
            setTiming((tsl2591IntegrationTime_t) (sensor_root[F("timing")].as<int>() - 1) / 100);
            configured = true;
        }
    }
    return configured;
}

void SensorTsl2591::setGain(tsl2591Gain_t gain) {
    m_tsl.setGain(gain);
}

void SensorTsl2591::setTiming(tsl2591IntegrationTime_t timing) {
    m_tsl.setTiming(timing);
}

String& SensorTsl2591::updateInfo(String& json) {
    while (!m_tsl.isFullLuminosityReady()); // Unable to carry out more than one operation on the device at one time
    sensor_t sensor;
    m_tsl.getSensor(&sensor);
    JsonBuilder s(json);
    JsonObject& datanode = s.get().createNestedObject(F("sensors")).createNestedObject(F("tsl2591"));
    datanode[F("sensor")] = sensor.name;
    datanode[F("gain")] = (int) m_tsl.getGain();
    datanode[F("timing")] = ((int) m_tsl.getTiming() + 1)*100;
    return s.serialize(json);
}
