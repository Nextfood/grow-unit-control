#include <Arduino.h>
#include <ArduinoJson.h>
#include "DevicePwmPowerDriver.h"
#include "Id.h"
#include "SerialUtils.h"



#define BAUDRATE 57600
#define SERIALPORT_TIMEOUT_MS 1000
#define READ_BUFFER_SIZE 150

DataPwm g_data;
InfoPwm g_info;
DevicePwmPowerDriver<6> g_device;
Id g_id;



#define RETURN_SUCCESS() \
{ \
    StaticJsonBuffer<300> jsonBuffer; \
    JsonObject& root = jsonBuffer.createObject(); \
    root[F("id")] = g_id.id; \
    root[F("result")] = F("success"); \
    String jsonStr; \
    root.printTo(jsonStr); \
    SerialUtils::write(jsonStr); \
}

#define RETURN_FAILURE(errorMsg) \
{ \
    StaticJsonBuffer<300> jsonBuffer; \
    JsonObject& root = jsonBuffer.createObject(); \
    root[F("id")] = g_id.id; \
    root[F("result")] = F("failure"); \
    root[F("error_message")] = errorMsg; \
    String jsonStr; \
    root.printTo(jsonStr); \
    SerialUtils::write(jsonStr); \
}

void sendInfo() {
    g_info.serializeJson();
}

void sendData() {
    g_data.serializeJson();
}

void parseCommand(String& json) {
    if (g_data.deserializeJson(json)) {
        g_data.serializeJson();
        return;
    } else if (g_info.deserializeJson(json)) {
        g_info.serializeJson();
        return;
    } else if (g_id.deserializeJson(json)) {
        RETURN_SUCCESS();
        return;
    }

    ConfigurePwm conf;
    if (conf.deserializeJson(json)) {
        g_device.configure(conf);
        RETURN_SUCCESS();
        return;
    }

    Serial.println(json);
    RETURN_FAILURE(F("Unknown Command"));
}

void setup() {
    g_id.readIdFromEeprom();

    if (g_id.id[0] == 0) {
        g_id.setIdWithoutEeprom(F("<unknown location>"));
    }

    g_data.id = g_id.id;
    g_info.id = g_id.id;


    Serial.setTimeout(SERIALPORT_TIMEOUT_MS);
    Serial.begin(BAUDRATE);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    // Setup devices
    SetupPwm s;
    s.id = g_id.id;
    g_device.setup(s);
    g_device.initiateUpdateData();
}

String readJson() {
    char buf[READ_BUFFER_SIZE];
    buf[0] = '\0';
    Serial.readBytesUntil('\n', buf, READ_BUFFER_SIZE);
    return String(buf);
}

void loop() {

    if (g_device.isUpdatedDataReady()) {
        g_device.updateData(g_data);
        g_device.initiateUpdateData();
    }

    String data = readJson();
    if (data.length()) {
        parseCommand(data);
    }

}
