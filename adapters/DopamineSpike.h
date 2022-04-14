#ifndef DOPAMINE_SPIKE_H
#define DOPAMINE_SPIKE_H

#include <iostream>
#include <map>
#include <math.h>
#include <fstream>

#include <music.hh>
#include <mpi.h>

#include "Adapter.h"
#include "ContInPort.h"
#include "EventOutPort.h"

#include "sys/time.h"

#include "jsoncpp/json/json.h"


/**
 * Listens to a ZeroMQ socket and passes the data to a continuous MUSIC port.
 * The ZeroMQ port is specified in the corresponding port. 
 * Different message types can be used via the argument 'message_type' in the MUSIC configuration:
 * 'FloatArray' and 'GymObservation'
 */
class DopamineSpike : public Adapter
{

    public:
        DopamineSpike();
        void init(int argc, char** argv);
        void tick();
        std::ofstream rew_file;
        std::ofstream dop_spk_file;
        double spk_time = 0;

    private:
};

#endif // DOPAMINE_SPIKE_H
