#ifndef _DevicePressureSensor1_2MPa_H_
#define _DevicePressureSensor1_2MPa_H_

#include "Device.h"
#include "SetupPressureSensor1_2MPa.h"
#include "DataPressureSensor1_2MPa.h"
#include "InfoPressureSensor1_2MPa.h"
#include "ConfigurePressureSensor1_2MPa.h"

class DevicePressureSensor1_2MPa : public Device<SetupPressureSensor1_2MPa,
DataPressureSensor1_2MPa, InfoPressureSensor1_2MPa, ConfigurePressureSensor1_2MPa> {
public:

    virtual void setup(SetupPressureSensor1_2MPa& setup) {
        m_pin = setup.pin;
    }

    virtual void initiateUpdateData() {
    }

    virtual bool isUpdatedDataReady() {
        return true;
    }

    virtual DataPressureSensor1_2MPa& updateData(DataPressureSensor1_2MPa& data) {
        data.pressure = _getPressure();
        data.pressure_adc = analogRead(m_pin);
        return data;
    }

    virtual InfoPressureSensor1_2MPa& updateInfo(InfoPressureSensor1_2MPa& info) {
        return info;
    }

    virtual bool configure(ConfigurePressureSensor1_2MPa& conf) {
        return false;
    }

private:

    float _getPressure() {
        // ADC is referenced from 0-5V (0-1023)
        // Pressure sensor is presumably linear from 0.5-4.5V
        // corresponding to 0-1.2 MPa.
        int adc = analogRead(m_pin);

        if (adc < 102) {
            return 0.0;
        } else
            if (adc > 921) {
            return 1.2;
        } else {
            return 0.001465201465 * (float) adc - 0.149450549; // basic linear equation
        }
    }

    int m_pin;
};



#endif
