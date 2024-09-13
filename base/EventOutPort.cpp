#include "EventOutPort.h"

void
EventOutPort::init(MUSIC::Setup* setup, char* port_name)
{

    port = setup->publishEventOutput(port_name);

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
    port->map(&l_index, MUSIC::Index::GLOBAL, 1);
}

void
EventOutPort::send(int n, double t)
{
    port->insertEvent(t, MUSIC::GlobalIndex(n));
}

