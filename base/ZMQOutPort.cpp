#include "ZMQOutPort.h"

void
ZMQOutPort::init(MUSIC::Setup* setup, char* port_name)
{
    zmq_addr = DEFAULT_ZMQ_ADDR;
    setup->config("zmq_addr", &zmq_addr);

    context = new zmq::context_t(1);
    publisher = new zmq::socket_t(*context, ZMQ_PUB);

    publisher->bind(zmq_addr.c_str());

}

//  Convert string to 0MQ string and send to socket
bool
ZMQOutPort::send (const std::string & string) {

    zmq::message_t message(string.size());
    memcpy (message.data(), string.data(), string.size());

    bool rc = publisher->send (message);
    return (rc);
}


