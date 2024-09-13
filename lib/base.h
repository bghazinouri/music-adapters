#ifndef BASE_ADAPTER_H
#define BASE_ADAPTER_H

#include <music.hh>
#include <mpi.h>

#include <vector>
#include <cmath>
#include <unistd.h>
#include "sys/time.h"


template<class T, class T>
class BaseAdapter
{
    private:
        MUSIC::ContInputPort* port_in;
        MUSIC::ContOutputPort* port_out;
}

#endif /* BASE_ADAPTER.H */

