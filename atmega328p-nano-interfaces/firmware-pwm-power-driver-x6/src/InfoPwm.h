#ifndef _InfoPwm_H_
#define _InfoPwm_H_

#include "Arduino.h"
#include <ArduinoJson.h>
#include "SerialUtils.h"


class InfoPwm {
public:
    char* id;

    void serializeJson() {
        StaticJsonBuffer<320> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        root[F("id")] = id;
        root[F("device")] = F("PWM Power Driver x6");
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
