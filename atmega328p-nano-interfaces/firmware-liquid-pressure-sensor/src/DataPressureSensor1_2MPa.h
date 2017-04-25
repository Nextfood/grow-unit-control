#ifndef _DataPressureSensor1_2MPa_H_
#define _DataPressureSensor1_2MPa_H_

#include "Arduino.h"
#include <ArduinoJson.h>
#include "SerialUtils.h"

class DataPressureSensor1_2MPa {
public:
    char* id;
    float pressure, pressure_adc;

    void serializeJson() {
        StaticJsonBuffer<320> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        root[F("id")] = id;
        JsonObject& deviceObj = root.createNestedObject(F("devices"));
        JsonObject& si7021Obj = deviceObj.createNestedObject(F("Pressure Sensor 1.2 MPa"));
        si7021Obj[F("pressure")] = double_with_n_digits(pressure, 3);
        si7021Obj[F("pressure_adc")] = pressure_adc;
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
