#ifndef _Device_H_
#define _Device_H_

#include <ArduinoJson.h>

class Device {
public:
    virtual void setup() = 0;
    virtual void initiateUpdateData() = 0;
    virtual bool isUpdatedDataReady() = 0;
    virtual String updateData(String& json) = 0;
    virtual String updateInfo(String& json) = 0;
    virtual bool configure(String& json) = 0;
};


#endif
