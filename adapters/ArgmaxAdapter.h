#ifndef ARGMAX_ADAPTER_H
#define ARGMAX_ADAPTER_H

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
 * Outputs the index channel of the maximum incoming value. Projects N incoming channels to 1 outgoing channel.
 */
class ArgmaxAdapter : public Adapter
{
    public:
        ArgmaxAdapter();
        void init(int argc, char** argv);
        void tick();

};

#endif // ARGMAX_ADAPTER_H

