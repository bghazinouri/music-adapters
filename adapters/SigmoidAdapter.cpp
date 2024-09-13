#include "SigmoidAdapter.h"

int
main(int argc, char** argv)
{

    SigmoidAdapter* adapter = new SigmoidAdapter();
    adapter->init(argc, argv);

    adapter->run(false);
    
    adapter->finalize();
}

SigmoidAdapter::SigmoidAdapter()
{
    port_in = new ContInPort();
    port_out = new ContOutPort();
}

void SigmoidAdapter::init(int argc, char** argv)
{

    scale = DEFAULT_SCALE;
    slope = DEFAULT_SLOPE;
    x_offset = DEFAULT_X_OFFSET;
    y_offset = DEFAULT_Y_OFFSET;

    Adapter::init(argc, argv, "Sigmoid");

    // config needed for this specific adapter
    setup->config("scale", &scale);
    setup->config("slope", &slope);
    setup->config("x_offset", &x_offset);
    setup->config("y_offset", &y_offset);
}

void
SigmoidAdapter::tick()
{
    for (int i = 0; i < port_in->data_size; ++i)
    {
        port_out->data[i] = scale / ( 1. + std::exp(- port_in->data[i] * slope + x_offset)) + y_offset;
    }
}


