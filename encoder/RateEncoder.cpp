#include "RateEncoder.h"

int
main(int argc, char** argv)
{

    RateEncoder* adapter = new RateEncoder();
    adapter->init(argc, argv);

    adapter->run(false);
    
    adapter->finalize();
}

RateEncoder::RateEncoder()
{
    port_in = new ContInPort();
    port_out = new EventOutPort();
}

void RateEncoder::init(int argc, char** argv)
{
    rate_min = DEFAULT_RATE_MIN;
    rate_max = DEFAULT_RATE_MAX;

    Adapter::init(argc, argv, "RateEncoder");

    // config needed for this specific adapter
    setup->config("rate_min", &rate_min);
    setup->config("rate_max", &rate_max);
    normalization_factor = (rate_max - rate_min) / 2.; 

    
    next_spike = new double[port_in->data_size];
    last_spike = new double[port_in->data_size];
    for (int i = 0; i < port_in->data_size; ++i)
    {
        last_spike[i] = 0.;
        next_spike[i] = rate2SpikeTime(port_in->data[i]); 
    }
 
}


void
RateEncoder::tick()
{

    double next_t = runtime->time() + timestep;
    for (int n = 0; n < port_in->data_size; ++n)
    {
        double old_isi = next_spike[n] - last_spike[n];
        double part_time_left = (runtime->time() - last_spike[n]) / old_isi; 
        double new_isi = rate2SpikeTime(port_in->data[n]) * (1 - part_time_left);
        next_spike[n] = runtime->time() + new_isi;

        while(next_spike[n] < next_t)
        {
            static_cast<EventOutPort*>(port_out)->send(n, next_spike[n] + timestep);
            last_spike[n] = next_spike[n];
            next_spike[n] += rate2SpikeTime(port_in->data[n]); 
        }
    }


}


double
RateEncoder::rate2SpikeTime(double r)
{
    // the incoming data, which is interpreted as rate, is between -1 and 1.
    // scales rate between [rate_min, rate_max]
    // returns next spike time
    double isi = 1. / ((r+1) * normalization_factor + rate_min);
    return isi;
}


