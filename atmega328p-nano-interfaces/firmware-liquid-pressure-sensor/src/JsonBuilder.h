#ifndef _JsonBuilder_H_
#define _JsonBuilder_H_

#include <ArduinoJson.h>

class JsonBuilder {
private:
    StaticJsonBuffer<300> m_jsonBuffer;
    JsonObject* m_obj;
public:

    JsonBuilder() : m_obj(&m_jsonBuffer.createObject()) {
    }

    JsonBuilder(String& json) : m_obj(&m_jsonBuffer.parseObject(json)) {
    }

    JsonObject& get() {
        return *m_obj;
    }

    String& serialize(String& json) {
        json = "";
        m_obj->prettyPrintTo(json);
        return json;
    }

    void serialize(Print& json) {
        m_obj->prettyPrintTo(json);
    }

};


#endif
