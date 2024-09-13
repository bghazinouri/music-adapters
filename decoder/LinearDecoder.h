#ifndef LINEAR_DECODER_H
#define LINEAR_DECODER_H

#include <iostream>
#include <map>
#include <math.h>
#include <cmath>

#include <music.hh>
#include <mpi.h>

#include "Adapter.h"
#include "EventInPort.h"
#include "ContOutPort.h"

#include "sys/time.h"

#include "jsoncpp/json/json.h"
#include <fstream>


/**
 * Decodes spiking activity to continuous values by convolving incoming spikes using an exponential kernel with time-constant tau.
 * The parameter tau can be set in the MUSIC configuration file.
 * The number of input channels equals the number of output channels.
 *
 */
class LinearDecoder : public Adapter
{

    const double DEFAULT_TAU = 0.03;
    public:
        LinearDecoder();
        void init(int argc, char** argv);
        void tick();

    private:
        double tau, inv_tau, propagator;
};

#endif // LINEAR_DECODER_H

