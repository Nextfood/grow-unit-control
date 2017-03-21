#ifndef _DeviceRelay_H_
#define _DeviceRelay_H_

#include "Device.h"
#include "JsonBuilder.h"

class DeviceRelay : public Device {
public:

    DeviceRelay(int pin, bool initialState, bool invert = false)
    : m_pin(pin), m_pinState(initialState), m_invert(invert) {
    }

    virtual void setup() {
        pinMode(m_pin, OUTPUT);
        (m_pinState ^ m_invert) ? digitalWrite(m_pin, HIGH) : digitalWrite(m_pin, LOW);
    }

    virtual void initiateUpdateData() {
    }

    virtual bool isUpdatedDataReady() {
        return true;
    }

    virtual String& updateData(String& json) {
        JsonBuilder s(json);
        JsonObject& datanode = s.get().createNestedObject(F("devices")).createNestedObject(F("relay"));
        datanode["state"] = m_pinState;
        return s.serialize(json);
    }

    virtual String& updateInfo(String& json) {
        JsonBuilder s(json);
        JsonObject& datanode = s.get().createNestedObject(F("devices")).createNestedObject(F("relay"));
        datanode["devices"] = F("SONGLE x1");
        return s.serialize(json);
    }

    virtual bool configure(String& json) {
        JsonBuilder s(json);
        JsonObject& root = s.get();
        if (root.containsKey(F("relay")) && root[F("relay")].is<JsonObject&>()) {
            JsonObject& dev_root = root[F("relay")].as<JsonObject&>();
            if (dev_root.containsKey(F("state"))) {
                m_pinState = dev_root[F("state")].as<bool>();
                (m_pinState ^ m_invert) ? digitalWrite(m_pin, HIGH) : digitalWrite(m_pin, LOW);
                return true;
            }
        }
        return false;
    }


private:

    int m_pin;
    bool m_pinState, m_invert;

};



#endif
