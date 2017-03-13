#ifndef _DevicePressureSensor1200kPa_H_
#define _DevicePressureSensor1200kPa_H_

#include "Device.h"
#include "JsonBuilder.h"

class DevicePressureSensor1200kPa : public Device {
public:

    DevicePressureSensor1200kPa(int pin)
    : m_pin(pin) {
    }

    virtual void setup() {
    }

    virtual void initiateUpdateData() {
    }

    virtual bool isUpdatedDataReady() {
        return true;
    }

    virtual String& updateData(String& json) {
        JsonBuilder s(json);
        if (!s.get().containsKey(F("device"))) {
            s.get().createNestedObject(F("device"));
        }
        if (!s.get()[F("device")].as<JsonObject&>().containsKey(F("liquid_pressure"))) {
            s.get()[F("device")].as<JsonObject&>().createNestedObject(F("liquid_pressure"));
        }
        JsonObject& datanode = s.get()[F("device")][F("liquid_pressure")];

        datanode[F("pressure")] = getPressure();
        datanode[F("pressure_adc")] = analogRead(m_pin);
        return s.serialize(json);
    }

    virtual String& updateInfo(String& json) {
        JsonBuilder s(json);
        if (!s.get().containsKey(F("device"))) {
            s.get().createNestedObject(F("device"));
        }
        if (!s.get()[F("device")].as<JsonObject&>().containsKey(F("liquid_pressure"))) {
            s.get()[F("device")].as<JsonObject&>().createNestedObject(F("liquid_pressure"));
        }
        JsonObject& datanode = s.get()[F("device")][F("liquid_pressure")];
        datanode["device"] = F("Liquid Pressure Sensor 0-1.2 MPa");
        return s.serialize(json);
    }

    virtual bool configure(String& json) {
        return false;
    }


private:

    float getPressure() {
        // ADC is referenced from 0-5V (0-1023)
        // Pressure sensor is presumably linear from 0.5-4.5V
        // corresponding to 0-1.2 MPa.
        int adc = analogRead(m_pin);

        if (adc < 102) {
            return 0.0;
        } else if (adc > 921) {
            return 1.2;
        } else {
            return 0.001465201465*(float)adc - 0.149450549; // basic linear equation
        }
    }

    int m_pin;
};



#endif
