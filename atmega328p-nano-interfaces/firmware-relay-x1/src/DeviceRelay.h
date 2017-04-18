#ifndef _DeviceRelay_H_
#define _DeviceRelay_H_

#include "Device.h"
#include "SetupRelay.h"
#include "DataRelay.h"
#include "InfoRelay.h"
#include "ConfigureRelay.h"

template<int CHANNELS>
class DeviceRelay : public Device<SetupRelay, DataRelay, InfoRelay, ConfigureRelay> {
public:

    virtual void setup(SetupRelay& setup) {
        for (int i = 0; i < CHANNELS; ++i) {
            m_pin[i] = setup.pin[i];
            m_relay[i] = setup.initState[i];
            m_invert[i] = setup.invert[i];
            pinMode(m_pin[i], OUTPUT);
            (m_relay[i] ^ m_invert[i]) ? digitalWrite(m_pin[i], HIGH) : digitalWrite(m_pin[i], LOW);
        }
    }

    virtual void initiateUpdateData() {
    }

    virtual bool isUpdatedDataReady() {
        return true;
    }

    virtual DataRelay& updateData(DataRelay& data) {
        for (int i = 0; i < CHANNELS; ++i) {
            data.relay[i] = m_relay[i];
        }
        return data;
    }

    virtual InfoRelay& updateInfo(InfoRelay& info) {
        return info;
    }

    virtual bool configure(ConfigureRelay& conf) {
        for (int i = 0; i < CHANNELS; ++i) {
            (conf.relay[i] ^ m_invert[i]) ? digitalWrite(m_pin[i], HIGH) : digitalWrite(m_pin[i], LOW);
            m_relay[i] = conf.relay[i];
        }
        return true;
    }
private:
    int m_pin[CHANNELS];
    bool m_relay[CHANNELS];
    bool m_invert[CHANNELS];
};



#endif
