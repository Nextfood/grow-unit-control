#ifndef _InfoPressureSensor1_2MPa_H_
#define _InfoPressureSensor1_2MPa_H_

#include "Arduino.h"
#include <ArduinoJson.h>
#include "SerialUtils.h"


class InfoPressureSensor1_2MPa {
public:
    char* id;

    void serializeJson() {
        StaticJsonBuffer<320> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        root[F("id")] = id;
		JsonArray& devicesRoot = root.createNestedArray("devices");
		devicesRoot.add(F("Pressure Sensor 1.2 MPa"));
        String jsonStr;
        root.printTo(jsonStr);
        SerialUtils::write(jsonStr);
    }

    bool deserializeJson(String& json) {
        StaticJsonBuffer<300> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(json);

        if (root.success()) {

            if (root[F("cmd")].is<char*>()) {
                String cmd = F("info");
                return (cmd.compareTo(root[F("cmd")].as<char*>())) == 0;
            }
        }
        return false;
    }
};


#endif
