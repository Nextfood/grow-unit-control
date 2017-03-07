#include <stdexcept>
#include <thread>
#include "InterfaceSerialConnection.h"




InterfaceSerialConnection::InterfaceSerialConnection(const std::string& portname) {
    try {
        
        m_ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(2000);
        m_ser.setTimeout(to);
        m_ser.setPort(portname);
        m_ser.open();
        if (!m_ser.isOpen())
            throw std::runtime_error("Unable to open port.");
        
    } catch (const serial::SerialException& e) {
        throw std::runtime_error("Exception occurred with the serial port: " + std::string(e.what()));
    }
}

std::string InterfaceSerialConnection::query(const std::string& sendStr) {
    try {
        if (sendStr.empty()) return "";

        int retries = 4;
        while (retries--) {
            size_t bytes_wrote = m_ser.write(sendStr);
            if (sendStr.size() != bytes_wrote)
                throw std::runtime_error("Unable to send bytes for query to serial port.");
            m_ser.flush();

            std::string received = m_ser.read(65536);
            if (!received.empty())
                return received;

            std::this_thread::sleep_for(std::chrono::seconds(1)); // In case of retry
        }
    } catch (const serial::SerialException& e) {
        throw std::runtime_error("Exception occurred with the serial port: " + std::string(e.what()));
    }
}
