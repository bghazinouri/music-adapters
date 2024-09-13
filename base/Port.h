#ifndef PORT_H
#define PORT_H

#include <music.hh>

/**
 *  Base class for all ports. Ports can either be incoming or outgoing. Moreover ports can be MUSIC ports or ports to external libraries such as ZeroMQ.
 */
class Port 
{
    public:
        virtual void init(MUSIC::Setup* setup, char* port_name) = 0; /**< Initializes the port. */
        int data_size = 0; /**< Width of the port. */
        double* data; /** Data for the current time step. */ 
};

#endif // ADAPTER_H

