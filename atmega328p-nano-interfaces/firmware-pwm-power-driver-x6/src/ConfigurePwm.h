#ifndef _ConfigurePwm_H_
#define _ConfigurePwm_H_

#include "Arduino.h"
#include <ArduinoJson.h>

class ConfigurePwm {
public:
    static const int channels = 6;
    float pwm[6];

    bool deserializeJson(String& json) {
        StaticJsonBuffer<300> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(json);

        if (root.success()) {
            if (root[F("devices")].is<JsonObject&>()) {
                JsonObject& devicesRoot = root[F("devices")].as<JsonObject&>();
                if (devicesRoot[F("PWM Power Driver x6")].is<JsonObject&>()) {
                    JsonObject& pwmX6Root = devicesRoot[F("PWM Power Driver x6")].as<JsonObject&>();
                    if (pwmX6Root[F("pwm")].is<JsonArray&>()) {
                        JsonArray& pwmArray = pwmX6Root[F("pwm")].as<JsonArray&>();
                        if (pwmArray.size() == channels) {
                            for (unsigned int i = 0; i < pwmArray.size(); ++i) {
                                pwm[i] = pwmArray.get<float>(i);
                            }
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }
};


#endif
