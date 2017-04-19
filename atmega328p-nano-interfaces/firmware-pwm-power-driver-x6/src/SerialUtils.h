#ifndef _SerialUtils_H_
#define _SerialUtils_H_

#include "Arduino.h"

class SerialUtils {
public:

    static void write(const String& s) {

        const char* ptr = s.c_str();
        unsigned int count = 0;
        while (count < s.length()) {
            count += Serial.write(&ptr[count], s.length() - count);
        }
        Serial.write('\n');
        Serial.flush();
    }
};


#endif
