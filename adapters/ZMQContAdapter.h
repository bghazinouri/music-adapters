#ifndef ZMQ_CONT_ADAPTER_H
#define ZMQ_CONT_ADAPTER_H

#include <iostream>
#include <map>
#include <math.h>

#include <music.hh>
#include <mpi.h>

#include "Adapter.h"
#include "ZMQInPort.h"
#include "ContOutPort.h"

#include "sys/time.h"

#include <iostream>
#include "jsoncpp/json/json.h"


/**
 * Listens to a ZeroMQ socket and passes the data to a continuous MUSIC port.
 * The ZeroMQ port is specified in the corresponding port. 
 * Different message types can be used via the argument 'message_type' in the MUSIC configuration:
 * 'FloatArray' and 'GymObservation'
 */
class ZMQContAdapter : public Adapter
{

    enum msg_types {FloatArray, GymObservation};
    const msg_types DEFAULT_MESSAGE_TYPE = GymObservation;
    public:
        ZMQContAdapter();
        void init(int argc, char** argv);
        void tick();
        // void asyncTick();

    private:
        msg_types msg_type;

};

#endif // ZMQ_CONT_ADAPTER_H
