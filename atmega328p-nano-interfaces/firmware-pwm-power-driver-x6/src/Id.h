#ifndef _Id_H_
#define _Id_H_

#include <ArduinoJson.h>
#include <EEPROM.h>
#include "Arduino.h"

#define SIZE_OF_ID 25

class Id {
public:
    char id[25];

    Id() {
        id[0] = 0;
    }
    
    void readIdFromEeprom() {
        for (int i = 0; i < SIZE_OF_ID; ++i) { // Acquire ID from the EEPROM
            id[i] = EEPROM.read(i);
            if (id[i] == 0 || id[i] == 0xff) break;
        }
        id[SIZE_OF_ID - 1] = 0;
    }

    void writeIdToEeprom() {
        for (int i = 0; i < SIZE_OF_ID - 1; ++i) {
            EEPROM.write(i, id[i]);
        }
        EEPROM.write(SIZE_OF_ID - 1, 0);
    }

    void setId(const String& str) {
        str.toCharArray(id, SIZE_OF_ID);
        writeIdToEeprom();
    }

    void setIdWithoutEeprom(const String& str) {
        str.toCharArray(id, SIZE_OF_ID);
    }

    bool deserializeJson(String& json) {
        StaticJsonBuffer<300> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(json);

        if (root.success()) {

            if (root[F("id")].is<char*>()) {
                setId(String(root[F("id")].as<char*>()));
                return true;
            }
        }
        return false;
    }

};


#endif
