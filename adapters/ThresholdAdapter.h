#ifndef THRESHOLD_ADAPTER_H
#define THRESHOLD_ADAPTER_H

#include <iostream>
#include <map>
#include <math.h>
#include <cmath>

#include <music.hh>
#include <mpi.h>

#include "Adapter.h"
#include "ContInPort.h"
#include "ContOutPort.h"

#include "sys/time.h"
#include "float.h"


/**
 * Applies either the heaviside function or ReLu on the input data.
 * The function can be chosen with the parameter 'is_heaviside' in the MUSIC configuration.
 * In either case the threshold can be set with the parameter 'threshold' and can be scaled and shifted using the parameters 'scale' and 'shift' which can be specified in the MUSIC configuration.
 *
 */
class ThresholdAdapter : public Adapter
{

    const double DEFAULT_THRESHOLD = 0.0;
    const bool DEFAULT_IS_HEAVISIDE = true;
    const double DEFAULT_SCALE = 1.0;
    const double DEFAULT_SHIFT = 0.;

    public:
        ThresholdAdapter();
        void init(int argc, char** argv);
        void tick();

    private:
        double threshold;
        bool is_heaviside;
        double scale;
        double shift;
};

#endif // THRESHOLD_ADAPTER_H
