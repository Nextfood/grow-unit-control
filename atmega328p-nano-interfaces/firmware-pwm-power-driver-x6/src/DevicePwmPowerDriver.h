#ifndef _DevicePwmPowerDriver_H_
#define _DevicePwmPowerDriver_H_

#include "Device.h"
#include "JsonBuilder.h"

class DevicePwmPowerDriver : public Device {
public:

    DevicePwmPowerDriver(int pin, String channelName)
    : m_pin(pin), m_pinLevel(0.0), m_channelName(channelName) {
    }

    virtual void setup() {
        pinMode(m_pin, OUTPUT);
        digitalWrite(m_pin, LOW);
    }

    virtual void initiateUpdateData() {
    }

    virtual bool isUpdatedDataReady() {
        return true;
    }

    virtual String& updateData(String& json) {
        JsonBuilder s(json);
        if (!s.get().containsKey(F("devices"))) {
            s.get().createNestedObject(F("devices"));
        }
        if (!s.get()[F("devices")].as<JsonObject&>().containsKey(F("pwm_power_driver"))) {
            s.get()[F("devices")].as<JsonObject&>().createNestedObject(F("pwm_power_driver"));
        }
        JsonObject& datanode = s.get()[F("devices")][F("pwm_power_driver")];

        datanode[m_channelName] = m_pinLevel;
        return s.serialize(json);
    }

    virtual String& updateInfo(String& json) {
        JsonBuilder s(json);
        if (!s.get().containsKey(F("devices"))) {
            s.get().createNestedObject(F("devices"));
        }
        if (!s.get()[F("devices")].as<JsonObject&>().containsKey(F("pwm_power_driver"))) {
            s.get()[F("devices")].as<JsonObject&>().createNestedObject(F("pwm_power_driver"));
        }
        JsonObject& datanode = s.get()[F("devices")][F("pwm_power_driver")];
        datanode["devices"] = F("PWM Power Driver");
        return s.serialize(json);
    }

    virtual bool configure(String& json) {
        JsonBuilder s(json);
        JsonObject& root = s.get();
        if (root.containsKey(F("pwm_power_driver")) && root[F("pwm_power_driver")].is<JsonObject&>()) {
            JsonObject& dev_root = root[F("pwm_power_driver")].as<JsonObject&>();
            if (dev_root.containsKey(m_channelName)) {
                m_pinLevel = dev_root[m_channelName].as<float>();
                if (m_pinLevel <= 0.0) {
                    digitalWrite(m_pin, LOW);
                    m_pinLevel = 0.0;
                } else if (m_pinLevel >= 100.0) {
                    digitalWrite(m_pin, HIGH);
                    m_pinLevel = 100.0;
                } else {
                    analogWrite(m_pin, (int) (m_pinLevel * 255) / 100);
                }
                return true;
            }
        }
        return false;
    }


private:

    int m_pin;
    float m_pinLevel;
    String m_channelName;


};



#endif
