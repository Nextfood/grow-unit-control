#ifndef _SensorTsl2591_H_
#define _SensorTsl2591_H_

#include "Adafruit_TSL2591.h"
#include "Device.h"

class SensorTsl2591 : public Device {
public:

    SensorTsl2591();
    virtual void setup();
    virtual void initiateUpdateData();
    virtual bool isUpdatedDataReady();
    virtual String& updateData(String& json);
    virtual String& updateInfo(String& json);
    virtual bool configure(String& json);

private:
    void setGain(tsl2591Gain_t gain);
    void setTiming(tsl2591IntegrationTime_t timing);

    Adafruit_TSL2591 m_tsl = Adafruit_TSL2591(0);
};



#endif
