#include "NefEncoder.h"

int
main(int argc, char** argv)
{

    NefEncoder* adapter = new NefEncoder();
    adapter->init(argc, argv);

    adapter->run(false);
    
    adapter->finalize();
}

NefEncoder::NefEncoder()
{
    port_in = new ContInPort();
    port_out = new EventOutPort();
}

void NefEncoder::init(int argc, char** argv)
{
    resolution = DEFAULT_NEURON_RESOLUTION;
    one_hot = DEFAULT_ONE_HOT;

    Adapter::init(argc, argv, "NefEncoder");

    // config needed for this specific adapter
    setup->config("neuron_resolution", &resolution);
    setup->config("one_hot", &one_hot);
    
    for (int n = 0; n < port_out->data_size; ++n){

        srand48_r(time(NULL) + n, &randBuffer);
        IAFNeuron neuron(port_in->data_size, randBuffer);
        neuron.setResolution(resolution);
        srand48_r(time(NULL) - n, &randBuffer);
        neuron.init_nef(port_in->data_size, randBuffer, one_hot);
        neuron.encode(port_in->data);
        neurons.push_back(neuron);
    }
 
}


void
NefEncoder::tick()
{
    t = runtime->time();
    
    for (unsigned int n = 0; n < neurons.size(); ++n)
    {
        neurons[n].encode(port_in->data);
    }
    
    double next_t = t + timestep;
    while (t <= next_t)
    {
        for (unsigned int n = 0; n < neurons.size(); ++n)
        {
            if (neurons[n].propagate())
            {
                static_cast<EventOutPort*>(port_out)->send(n, runtime->time() + timestep);
            }
        }
        t += resolution; // propagate one timestep 
    }
}


