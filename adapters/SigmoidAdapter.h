#ifndef SIGMOID_ADAPTER_H
#define SIGMOID_ADAPTER_H

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
 * Applies a sigmoidal function.
 * The equation is:
 *
 * sig(x) = scale / ( 1. + std::exp(- port_in->data[i] * slope + x_offset)) + y_offset
 *
 * The parameter 'scale', 'slope', 'x_offset' and 'y_offset' can be set in the MUSIC configuration.
 */
class SigmoidAdapter : public Adapter
{

    const double DEFAULT_SCALE = 1.0;
    const double DEFAULT_SLOPE = 1.0;
    const double DEFAULT_X_OFFSET = 0.0;
    const double DEFAULT_Y_OFFSET = 0.0;

    public:
        SigmoidAdapter();
        void init(int argc, char** argv);
        void tick();

    private:
        double scale, slope, x_offset, y_offset;
};

#endif // SIGMOID_ADAPTER_H
