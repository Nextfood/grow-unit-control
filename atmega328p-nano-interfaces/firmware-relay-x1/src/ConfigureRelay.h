#ifndef _ConfigureRelay_H_
#define _ConfigureRelay_H_

#include "Arduino.h"
#include <ArduinoJson.h>

class ConfigureRelay {
public:
    static const int channels = 1;
    bool relay[1];

    bool deserializeJson(String& json) {
        StaticJsonBuffer<300> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(json);

        if (root.success()) {
            if (root[F("devices")].is<JsonObject&>()) {
                JsonObject& devicesRoot = root[F("devices")].as<JsonObject&>();
                if (devicesRoot[F("relay")].is<JsonArray&>()) {
                    JsonArray& relays = devicesRoot[F("relay")].as<JsonArray&>();
                    if (relays.size() == channels) {
                        for (unsigned int i = 0; i < relays.size(); ++i) {
                            relays[i] = relays.get<bool>(i);
                        }
                        return true;
                    }
                }

            }
        }
        return false;
    }
};


#endif
