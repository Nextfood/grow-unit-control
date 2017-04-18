#ifndef _DevicePwmPowerDriver_H_
#define _DevicePwmPowerDriver_H_

#include "Device.h"
#include "SetupPwm.h"
#include "DataPwm.h"
#include "InfoPwm.h"
#include "ConfigurePwm.h"

template<int CHANNELS>
class DevicePwmPowerDriver : public Device<SetupPwm, DataPwm, InfoPwm, ConfigurePwm> {
public:

    virtual void setup(SetupPwm& setup) {
        for (int i = 0; i < CHANNELS; ++i) {
            m_pin[i] = setup.pin[i];
            m_pwm[i] = setup.pwm[i];
            pinMode(m_pin[i], OUTPUT);
            digitalWrite(m_pin[i], LOW);
        }
    }

    virtual void initiateUpdateData() {
    }

    virtual bool isUpdatedDataReady() {
        return true;
    }

    virtual DataPwm& updateData(DataPwm& data) {
        for (int i = 0; i < CHANNELS; ++i) {
            data.pwm[i] = m_pwm[i];
        }
        return data;
    }

    virtual InfoPwm& updateInfo(InfoPwm& info) {
        return info;
    }

    virtual bool configure(ConfigurePwm& conf) {
        for (int i = 0; i < CHANNELS; ++i) {
            if (conf.pwm[i] <= 0.0) {
                digitalWrite(m_pin[i], LOW);
                m_pwm[i] = 0.0;
            } else if (conf.pwm[i] >= 100.0) {
                digitalWrite(m_pin[i], HIGH);
                m_pwm[i] = 100.0;
            } else {
                int v = (conf.pwm[i] * 255) / 100;
                analogWrite(m_pin[i], v);
                m_pwm[i] = conf.pwm[i];
            }
        }
        return true;
    }


private:
    float m_pwm[CHANNELS];
    int m_pin[CHANNELS];
};



#endif
