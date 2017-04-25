#ifndef _DataRelay_H_
#define _DataRelay_H_

#include "Arduino.h"
#include <ArduinoJson.h>
#include "SerialUtils.h"

class DataRelay {
public:
    char* id;
    static const int channels = 1;
    bool relay[1];

    void serializeJson() {
        StaticJsonBuffer<320> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        root[F("id")] = id;
        JsonObject& deviceObj = root.createNestedObject(F("devices"));
        JsonArray& array = deviceObj.createNestedArray(F("relay"));
        for (int i = 0; i < channels; ++i) {
            array.add(relay[i]);
        }

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
