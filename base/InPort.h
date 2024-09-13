#ifndef IN_PORT_H
#define IN_PORT_H

#include "Port.h"

/**
 *  Base class for all incoming ports.
 */
class InPort : public Port 
{
    public:
        virtual void init(MUSIC::Setup* setup, char* port_name) = 0;
};

#endif // ADAPTER_H

