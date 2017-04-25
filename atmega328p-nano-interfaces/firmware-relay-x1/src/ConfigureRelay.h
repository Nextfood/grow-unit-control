#ifndef _ConfigureRelay_H_
#define _ConfigureRelay_H_

#include "Arduino.h"
#include <ArduinoJson.h>

class ConfigureRelay {
public:
    static const int channels = 1;
    bool relay[1];

    bool deserializeJson(String& json) {
        StaticJsonBuffer<320> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(json);

        if (root.success()) {
            if (root[F("devices")].is<JsonObject&>()) {
                JsonObject& devicesRoot = root[F("devices")].as<JsonObject&>();
                if (devicesRoot[F("Relay x1")].is<JsonObject&>()) {
                    JsonObject& relayX1Root = devicesRoot[F("Relay x1")].as<JsonObject&>();
                    if (relayX1Root[F("state")].is<JsonArray&>()) {
                        JsonArray& relayArray = relayX1Root[F("state")].as<JsonArray&>();
                        if (relayArray.size() == channels) {
                            for (unsigned int i = 0; i < relayArray.size(); ++i) {
                                relay[i] = relayArray.get<bool>(i);
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
