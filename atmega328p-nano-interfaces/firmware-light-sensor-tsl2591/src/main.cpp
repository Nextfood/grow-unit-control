#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include "Arduino.h"
#include "SensorTsl2591.h"
#include "JsonBuilder.h"


#define BAUDRATE 57600
#define SERIALPORT_TIMEOUT_MS 100


String g_id;
String g_info;
String g_data;


#define NUMBER_OF_DEVICES 1



Device* deviceArray[NUMBER_OF_DEVICES] = {
    new SensorTsl2591()
    // Add more devices here
};

static inline void returnSuccess() {
    JsonBuilder result;
    result.get()[F("id")] = g_id;
    result.get()[F("result")] = F("success");
    result.serialize(Serial);
}

static inline void returnFailure(String errorMsg) {
    JsonBuilder result;
    result.get()[F("id")] = g_id;
    result.get()["result"] = F("failure");
    result.get()[F("error_message")] = errorMsg;
    result.serialize(Serial);
}

void setId(const String& str) {
    g_id = str;
    {
        JsonBuilder s(g_info);
        s.get()["id"] = str;
        s.serialize(g_info);
    }
    {
        JsonBuilder s(g_data);
        s.get()["id"] = str;
        s.serialize(g_data);
    }

    returnSuccess();
}

void sendInfo() {
    for (int i = 0; i < NUMBER_OF_DEVICES; ++i)
        deviceArray[i]->updateInfo(g_info);
    Serial.println(g_info);
}

void sendData() {
    Serial.println(g_data);
}

void parseJson(String& json) {
    String cmd;
    String id;
    {
        JsonBuilder s(json);
        JsonObject& root = s.get();
        if (!root.success()) // JSON parsing failed
            return;

        if (root.containsKey("cmd")) {
            cmd = String(root["cmd"].as<char*>());
        } else if (root.containsKey("id")) {
            id = String(root["id"].as<char*>());
        }
    } // deallocate json parser


    if (cmd.compareTo("info") == 0)
        return sendInfo();

    if (cmd.compareTo("data") == 0)
        return sendData();

    if (id.length() != 0) {
        return setId(id);
    }

    bool configured = false;
    for (int i = 0; i < NUMBER_OF_DEVICES; ++i) {
        if (deviceArray[i]->configure(json)) configured = true;
    }

    if (configured)
        returnSuccess();
    else
        returnFailure(F("Unknown command."));
}

void setup() {
    g_id = F("<unknown location>");

    {
        JsonBuilder s;
        s.get()["id"] = g_id;
        g_info.reserve(300);
        s.serialize(g_info);
    }
    {
        JsonBuilder s;
        s.get()["id"] = g_id;
        g_info.reserve(300);
        s.serialize(g_data);
    }

    Serial.setTimeout(SERIALPORT_TIMEOUT_MS);
    Serial.begin(BAUDRATE);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }


    // Setup devices
    for (int i = 0; i < NUMBER_OF_DEVICES; ++i) {
        deviceArray[i]->setup();
        deviceArray[i]->initiateUpdateData();
    }
}

void loop() {

    for (int i = 0; i < NUMBER_OF_DEVICES; ++i) {
        if (deviceArray[i]->isUpdatedDataReady()) {
            deviceArray[i]->updateData(g_data);
            deviceArray[i]->initiateUpdateData();
        }
    }

    while (Serial.available() > 0) {
        String data = Serial.readString();
        parseJson(data);
    }


}



