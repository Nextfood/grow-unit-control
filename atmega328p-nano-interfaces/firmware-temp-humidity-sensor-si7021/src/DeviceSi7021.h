#ifndef _DeviceSi7021_H_
#define _DeviceSi7021_H_

#include "Adafruit_Si7021.h"
#include "Device.h"
#include "SetupSi7021.h"
#include "DataSi7021.h"
#include "InfoSi7021.h"
#include "ConfigureSi7021.h"

class DeviceSi7021 : public Device<SetupSi7021, DataSi7021, InfoSi7021, ConfigureSi7021> {
public:

    virtual void setup(SetupSi7021& setup) {
        m_si7021.begin();
    }

    virtual void initiateUpdateData() {
    }

    virtual bool isUpdatedDataReady() {
        return true;
    }

    virtual DataSi7021& updateData(DataSi7021& data) {
        data.temperature = m_si7021.readTemperature();
        data.humidity = m_si7021.readHumidity();
        return data;
    }

    virtual InfoSi7021& updateInfo(InfoSi7021& info) {
        return info;
    }

    virtual bool configure(ConfigureSi7021& conf) {
        return false;
    }
private:
    Adafruit_Si7021 m_si7021 = Adafruit_Si7021();
};



#endif
