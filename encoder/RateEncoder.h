#ifndef RATE_ENCODER_H
#define RATE_ENCODER_H

#include <iostream>
#include <map>
#include <math.h>
#include <cmath>

#include <music.hh>
#include <mpi.h>

#include "Adapter.h"
#include "ContInPort.h"
#include "EventOutPort.h"

#include "sys/time.h"

#include "jsoncpp/json/json.h"
#include <fstream>

/**
 * The rate encoder translates continuous values into spike trains with regular inter-spike-intervals (ISI) by simulating Integrate-And-Fire neurons.
 * The parameters 'rate_min' and 'rate_max' can be set in the MUSIC configuration.
 */
class RateEncoder : public Adapter
{

    const double DEFAULT_RATE_MIN = 0.;
    const double DEFAULT_RATE_MAX = 100.;

    public:
        RateEncoder();
        void init(int argc, char** argv);
        void tick();
        

    private:
        double normalization_factor;
        double rate_min, rate_max;
        double* next_spike;
        double* last_spike;

        double rate2SpikeTime(double r);

};

#endif // RATE_ENCODER_H

