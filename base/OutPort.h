#ifndef OUT_PORT_H
#define OUT_PORT_H

#include "Port.h"

/**
 *  Base class for all outgoing ports.
 */
class OutPort : public Port 
{
    public:
        virtual void init(MUSIC::Setup* setup, char* port_name) = 0;
};

#endif // ADAPTER_H

