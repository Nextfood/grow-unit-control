#ifndef __InterfaceSerialConnection_h__
#define __InterfaceSerialConnection_h__

#include <string>
#include <serial/serial.h>


class InterfaceSerialConnection {
public:

    InterfaceSerialConnection(const std::string& portname);
    std::string query(const std::string& sendStr);
    
private:
    serial::Serial m_ser;
};

#endif 

