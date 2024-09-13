#ifndef ZMQ_OUT_PORT_H
#define ZMQ_OUT_PORT_H

#include <music.hh>
#include <zmq.hpp> // https://github.com/zeromq/cppzmq
#include "OutPort.h"

#define DEBUG_OUTPUT false 

/**
 * Implements the incoming ZMQ socket.
 */ 
class ZMQOutPort : public OutPort
{

    const std::string DEFAULT_ZMQ_ADDR = "tcp://127.0.0.1:5556";
    const std::string DEFAULT_ZMQ_TOPIC = "out";

    public:
        void init(MUSIC::Setup* setup, char* port_name);
        bool send(const std::string& string);

    private:
        zmq::context_t *context;
        zmq::socket_t *publisher; 
        std::string zmq_addr;

};

#endif // ZMQ_OUT_PORT_H


