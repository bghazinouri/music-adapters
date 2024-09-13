#ifndef CONT_ZMQ_ADAPTER_H
#define CONT_ZMQ_ADAPTER_H

#include <iostream>
#include <map>
#include <math.h>

#include <music.hh>
#include "music/error.hh"
#include <mpi.h>

#include "Adapter.h"
#include "ContInPort.h"
#include "ZMQOutPort.h"

#include "sys/time.h"

#include <iostream>
#include "jsoncpp/json/json.h"

/**
 * Publishes continous data unsing a ZeroMQ socket using JSON serialization. 
 * Different messages types can be used as output format.
 * The message type 'FloatArray' determines the width of the array from the width of the incoming port and does not need any further parameters.
 * The message type 'GymCommand' needs the parameter 'min' and 'max' which can be set in the MUSIC configuration file.
 */
class ContZMQAdapter : public Adapter
{
    enum msg_types {FloatArray, GymCommand}; 
    const msg_types DEFAULT_MESSAGE_TYPE = FloatArray;
    public:
        ContZMQAdapter();
        void init(int argc, char** argv);
        void tick();

    private:
        msg_types msg_type;
        Json::FastWriter writer;
        double min;
        double max;
};

#endif // CONT_ZMQ_ADAPTER_H
