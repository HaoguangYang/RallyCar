#ifndef _SERIAL_COMMS_INTERFACE_HPP_
#define _SERIAL_COMMS_INTERFACE_HPP_

// reused data structures from bare-metal application
#include "../../firmware/rallycar_arduino_v2024.3/builtin_interfaces/msg/time.h"
#include "../../firmware/rallycar_arduino_v2024.3/transport_layer/hdlc.h"
#include "../../firmware/rallycar_arduino_v2024.3/transport_layer/endpoint.h"
// for serial communication in Linux
#include "serial.h"
#include "serial_impl.h"
#include <cstdlib>

template <uint16_t RX_MTU = 256, uint16_t TX_MTU = 256, uint8_t MAX_ENDPOINTS = 64>
class SerialCommsInterface {
using SerialEndpointBase = sensor_network::EndpointBase;
private:
    serial::Serial ser;
    Hdlc<SerialCommsInterface> framer;

    uint8_t rxBuffer[RX_MTU];
    uint8_t txBuffer[TX_MTU];
    bool txBufferLocked = false;
    SerialEndpointBase* endpoints[MAX_ENDPOINTS] = {nullptr};
    uint8_t max_stream_record = 0;
    NodeState driverState = NodeState::OUT_OF_SYNC;

protected:
    void reset() {
        uint8_t* buf = getEndpointBuffer((uint8_t)NodeServiceID::RESET_ID);
        *buf = 0;
        returnEndpointBuffer(1);
        driverState = NodeState::OUT_OF_SYNC;
    }

    using SerialTime = builtin_interfaces__msg::Time;
    virtual SerialTime time_now() = 0;

    bool clock_sync_reply(const uint8_t* req, const uint16_t& length) {
        static SerialTime t;
        if (length != SerialTime::deserialize(t, req)) return false;
        // construct reply message
        uint8_t* txBuff = getEndpointBuffer((uint8_t)NodeServiceID::TIME_SYNC_ID);
        if (txBuff == nullptr) return false;
        t = time_now();
        uint8_t* ptr = txBuff;
        for (uint16_t n = 0; n < length; n ++) {
            // part 1: repeat input timestamp
            *ptr = req[n];
            ptr ++;
        }
        ptr += SerialTime::serialize(t, ptr);
        returnEndpointBuffer(ptr - txBuff);
        return true;
    }

public:
    const uint8_t session_id;

    SerialCommsInterface(const uint8_t& sessionId = 0, const std::string& port = "", const uint32_t& baud = 115200) :
        ser{port, baud, serial::Timeout::simpleTimeout(1000)},
        framer{
            &SerialCommsInterface::send_char,
            &SerialCommsInterface::onIncomingData,
            RX_MTU, &(rxBuffer[0]), this},
        session_id(sessionId)
    {}

    ~SerialCommsInterface() {
        reset();
        ser.close();
    }

    void setPort (const std::string &port) { ser.setPort(port); }
    void setBaudrate (const uint32_t &baud) { ser.setBaudrate(baud); }

    void transferInit() {
        if (ser.isOpen()) ser.close();
        ser.open();
    }

    void spin() {
        while (ser.available()){
            static uint8_t data;
            ser.read(&data, 1);
            framer.receiveChar(data);
        }
    }

    uint8_t* getEndpointBuffer(const uint8_t& stream_id) {
        if (txBufferLocked) return nullptr;
        txBufferLocked = true;
        txBuffer[0] = session_id;
        txBuffer[1] = stream_id;
        return &(txBuffer[2]);
    }

    void returnEndpointBuffer(const uint16_t& size) {
        if (size)
            framer.sendFrame((const char*)(&txBuffer[0]), size+2);
        txBufferLocked = false;
    }

    bool registerEndpoint(SerialEndpointBase* endpoint, const uint8_t& stream_id) {
        if (stream_id >= MAX_ENDPOINTS) return false;
        if (endpoints[stream_id] != nullptr) return false;
        endpoints[stream_id] = endpoint;
        endpoints[stream_id]->stream_id = stream_id;
        if (stream_id >= max_stream_record) max_stream_record = stream_id + 1;
        return true;
    }

    void send_char(const uint8_t& data) {
        ser.write(&data, 1);
    }

    void onIncomingData(const uint8_t* payload, const uint16_t& length) {
        if (payload[0] != session_id || length <= 2) return;
        switch (payload[1]) {
            case (uint8_t)NodeServiceID::TIME_SYNC_ID:
                if (clock_sync_reply(&payload[2], length-2))
                    driverState = NodeState::RUNNING;
                return;
            default:
                if (driverState != NodeState::RUNNING || payload[1] >= max_stream_record)
                    return;
                break;
        }
        endpoints[payload[1]] -> spin(&payload[2], length-2);
    }
};

#endif  /* _SERIAL_COMMS_INTERFACE_HPP_ */
