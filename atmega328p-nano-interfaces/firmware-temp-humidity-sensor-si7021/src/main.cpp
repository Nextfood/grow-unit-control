#include <Arduino.h>
#include <ArduinoJson.h>
#include "DeviceSi7021.h"
#include "Id.h"
#include "SerialUtils.h"



#define BAUDRATE 57600
#define SERIALPORT_TIMEOUT_MS 1000
#define READ_BUFFER_SIZE 150

DataSi7021 g_data;
InfoSi7021 g_info;
DeviceSi7021 g_device;
Id g_id;



#define RETURN_SUCCESS() \
{ \
    StaticJsonBuffer<320> jsonBuffer; \
    JsonObject& root = jsonBuffer.createObject(); \
    root[F("id")] = g_id.id; \
    root[F("result")] = F("success"); \
    String jsonStr; \
    root.printTo(jsonStr); \
    SerialUtils::write(jsonStr); \
    return; \
}

#define RETURN_FAILURE(errorMsg) \
{ \
    StaticJsonBuffer<320> jsonBuffer; \
    JsonObject& root = jsonBuffer.createObject(); \
    root[F("id")] = g_id.id; \
    root[F("result")] = F("failure"); \
    root[F("error_message")] = errorMsg; \
    String jsonStr; \
    root.printTo(jsonStr); \
    SerialUtils::write(jsonStr); \
    return; \
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
    }

    ConfigureSi7021 conf;
    if (conf.deserializeJson(json)) {
        g_device.configure(conf);
        RETURN_SUCCESS();
    }

    RETURN_FAILURE(F("Unknown Command"));
}

void setup() {
    g_id.readIdFromEeprom();

    if (g_id.id[0] == 0 || g_id.id[0] == 0xff) {
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
    SetupSi7021 s;
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
