#include "EventInPort.h"


void
EventInPort::init(MUSIC::Setup* setup, char* port_name)
{

    port = setup->publishEventInput(port_name);

    // get dimensions of data
    if (port->hasWidth())
    {
        data_size = port->width();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        std::exit(-1);
    }
    
    // map linear index to event out port 
    MUSIC::LinearIndex l_index(0, data_size);
    port->map(&l_index, this, 0.0, 1); //TODO acceptable latency of 0. Does that work? 
}

void EventInPort::operator () (double t, MUSIC::GlobalIndex id){
    spikes.push (Event (t, id)); //TODO add acceptable latency to t?
}
