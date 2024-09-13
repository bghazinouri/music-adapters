#ifndef EVENT_OUT_PORT_H
#define EVENT_OUT_PORT_H

#include <music.hh>
#include "OutPort.h"

#define DEBUG_OUTPUT false 

/**
 * Implements the outgoing MUSIC event port.
 */
class EventOutPort : public OutPort{
    public:
        void init(MUSIC::Setup* setup, char* port_name);
        void send(int n, double t);

    private:

        MUSIC::EventOutputPort* port;

};

#endif // EVENT_OUT_PORT_H


