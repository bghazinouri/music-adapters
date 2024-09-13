#include "PoissonEncoder.h"

int
main(int argc, char** argv)
{

    PoissonEncoder* adapter = new PoissonEncoder();
    adapter->init(argc, argv);

    adapter->run(false);
    
    adapter->finalize();
}

PoissonEncoder::PoissonEncoder()
{
    port_in = new ContInPort();
    port_out = new EventOutPort();
}

void PoissonEncoder::init(int argc, char** argv)
{
    rate_min = DEFAULT_RATE_MIN;
    rate_max = DEFAULT_RATE_MAX;

    Adapter::init(argc, argv, "PoissonEncoder");

    // config needed for this specific adapter
    setup->config("rate_min", &rate_min);
    setup->config("rate_max", &rate_max);
    normalization_factor = (rate_max - rate_min) / 2.; 

    srand48_r(time(NULL), &randBuffer);
    
    next_spike = new double[port_in->data_size];
    last_spike = new double[port_in->data_size];
    for (int i = 0; i < port_in->data_size; ++i)
    {
        last_spike[i] = 0.;
        next_spike[i] = negexp(denormalize(port_in->data[i])); 
    }
 
}


void
PoissonEncoder::tick()
{
    for (int n = 0; n < port_in->data_size; ++n)
    {
        next_spike[n] = runtime->time() + negexp(denormalize(port_in->data[n]));

        while(next_spike[n] < runtime->time() + timestep)
        {
	        static_cast<EventOutPort*>(port_out)->send(n, runtime->time() + timestep);
	        next_spike[n] += negexp(denormalize(port_in->data[n]));
        }
    }

}


double
PoissonEncoder::denormalize(double s)
{
    // incoming data is normalized between -1 and 1
    // returns interspike interval with rate between [min_rate, max_rate]
    
    return 1. / ((s+1) * normalization_factor + rate_min);
}

double
PoissonEncoder::negexp (double m)
{
    double r;
    drand48_r(&randBuffer, &r);
    return - m * log (r);
}


