#ifndef EVENT_IN_PORT_H
#define EVENT_IN_PORT_H

#include <music.hh>
#include "InPort.h"

#include <queue>

#define DEBUG_OUTPUT false 


/**
 * Implements the incoming MUSIC event port.
 */
class EventInPort : public InPort, MUSIC::EventHandlerGlobalIndex{

    class Event {
	    public:
	        double t;
	        int id;
	        Event (double t_, int id_) : t (t_), id (id_) { }
	        bool operator< (const Event& other) const { return t > other.t; }
	    };

    public:
        std::priority_queue<Event> spikes;

        void init(MUSIC::Setup* setup, char* port_name);

    private:
        MUSIC::EventInputPort* port;

        void operator() (double t, MUSIC::GlobalIndex id );


};

#endif // EVENT_IN_PORT_H


