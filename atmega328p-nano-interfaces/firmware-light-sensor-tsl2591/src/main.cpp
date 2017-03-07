#include <SoftwareSerial.h>
#include "Arduino.h"
#include "Adafruit_TSL2591.h"
#include <ArduinoJson.h>



Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

void setGain(tsl2591Gain_t gain) {
    tsl.setGain(gain);
}

void setGain(const String& str) {
    if (str.compareTo("1") == 0) setGain(TSL2591_GAIN_LOW);
    else if (str.compareTo("25") == 0) setGain(TSL2591_GAIN_MED);
    else if (str.compareTo("428") == 0) setGain(TSL2591_GAIN_HIGH);
    else if (str.compareTo("9876") == 0) setGain(TSL2591_GAIN_MAX);

}

void setTiming(tsl2591IntegrationTime_t timing) {
    tsl.setTiming(timing);
}

void setTiming(const String& str) {
    if (str.compareTo("100") == 0) setTiming(TSL2591_INTEGRATIONTIME_100MS);
    else if (str.compareTo("200") == 0) setTiming(TSL2591_INTEGRATIONTIME_200MS);
    else if (str.compareTo("300") == 0) setTiming(TSL2591_INTEGRATIONTIME_300MS);
    else if (str.compareTo("400") == 0) setTiming(TSL2591_INTEGRATIONTIME_400MS);
    else if (str.compareTo("500") == 0) setTiming(TSL2591_INTEGRATIONTIME_500MS);
    else if (str.compareTo("600") == 0) setTiming(TSL2591_INTEGRATIONTIME_600MS);
}

void readSensor(uint16_t& full, uint16_t& ir, uint32_t& lux) {
    uint32_t lum = tsl.getFullLuminosity();
    ir = lum >> 16;
    full = lum & 0xFFFF;
    lux = tsl.calculateLux(full, ir);
}

String gainToString(tsl2591Gain_t gain) {
    switch (gain) {
        case TSL2591_GAIN_LOW: return "1";
        case TSL2591_GAIN_MED: return "25";
        case TSL2591_GAIN_HIGH: return "428";
        case TSL2591_GAIN_MAX: return "9876";
        default: return "";
    };
}

String timingToString(tsl2591IntegrationTime_t timing) {
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

void sendSensorInfo() {
    sensor_t sensor;
    tsl.getSensor(&sensor);
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["id"] = "TSL2591";
    root["sensor"] = sensor.name;
    root["driver_version"] = sensor.version;
    root["unique_id"] = sensor.sensor_id;
    root["min_value"] = sensor.min_value;
    root["max_value"] = sensor.max_value;
    root["resolution"] = sensor.resolution;
    root["gain"] = gainToString(tsl.getGain());
    root["timing"] = timingToString(tsl.getTiming());
    root.printTo(Serial);
}

void sendSensorData() {
    uint16_t full, ir;
    uint32_t lux;
    readSensor(full, ir, lux);
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["full"] = full;
    root["ir"] = ir;
    root["lux"] = lux;

    root.printTo(Serial);
}

void parseJson(const String& json) {
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(json);
    if (root.containsKey("gain"))
        setGain(root["gain"].as<char*>());

    if (root.containsKey("timing"))
        setTiming(root["timing"].as<char*>());

    if (root.containsKey("cmd")) {
        if (String(root["cmd"].as<char*>()).compareTo("info") == 0)
            sendSensorInfo();
        else if (String(root["cmd"].as<char*>()).compareTo("data") == 0)
            sendSensorData();
    }
}

void stringWriteCallback(const String& data) {
    parseJson(data);
}

void setup() {
    Serial.setTimeout(100);
    Serial.begin(57600);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }


    if (!tsl.begin()) {
        Serial.write("No TSL2591 sensor found. Check wiring.");
        while (1);
    }
    setGain(TSL2591_GAIN_MED);
    setTiming(TSL2591_INTEGRATIONTIME_600MS);
}

void loop() {


    while (Serial.available() > 0) {
        String data = Serial.readString();
        stringWriteCallback(data);
    }
}



