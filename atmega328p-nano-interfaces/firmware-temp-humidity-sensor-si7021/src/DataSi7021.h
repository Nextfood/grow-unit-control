#ifndef _DataSi7021_H_
#define _DataSi7021_H_

#include "Arduino.h"
#include <ArduinoJson.h>
#include "SerialUtils.h"

class DataSi7021 {
public:
    char* id;
    float temperature, humidity;

    void serializeJson() {
        StaticJsonBuffer<320> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        root[F("id")] = id;
        JsonObject& deviceObj = root.createNestedObject(F("devices"));
        JsonObject& si7021Obj = deviceObj.createNestedObject(F("Temp Humidity Sensor Si7021"));
        si7021Obj[F("temperature")] = temperature;
        si7021Obj[F("humidity")] = humidity;
        String jsonStr;
        root.printTo(jsonStr);
        SerialUtils::write(jsonStr);
    }

    bool deserializeJson(String& json) {
        StaticJsonBuffer<300> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(json);
        if (root.success()) {
            if (root[F("cmd")].is<char*>()) {
                String cmd = F("data");
                return (cmd.compareTo(root[F("cmd")].as<char*>())) == 0;
            }
        }
        return false;
    }
};


#endif
