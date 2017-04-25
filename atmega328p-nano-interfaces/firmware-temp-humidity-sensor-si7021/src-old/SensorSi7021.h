#ifndef _SensorSi7021_H_
#define _SensorSi7021_H_

#include "Adafruit_Si7021.h"
#include "Device.h"
#include "JsonBuilder.h"

class SensorSi7021 : public Device {
public:

    SensorSi7021() {
    }

    virtual void setup() {
        m_si7021.begin();
    }

    virtual void initiateUpdateData() {
    }

    virtual bool isUpdatedDataReady() {
        return true;
    }

    virtual String& updateData(String& json) {
        JsonBuilder s(json);
        JsonObject& datanode = s.get().createNestedObject("sensors").createNestedObject("si7021");
        datanode["temperature"] = m_si7021.readTemperature();
        datanode["humidity"] = m_si7021.readHumidity();
        return s.serialize(json);
    }

    virtual String& updateInfo(String& json) {
        JsonBuilder s(json);
        JsonObject& datanode = s.get().createNestedObject("sensors").createNestedObject("si7021");
        datanode["sensor"] = "Si7021";
        return s.serialize(json);
    }

    virtual bool configure(String& json) {
        return false;
    }
    ;

private:
    Adafruit_Si7021 m_si7021 = Adafruit_Si7021();
};



#endif
