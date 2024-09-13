#include "ThresholdAdapter.h"

int
main(int argc, char** argv)
{

    ThresholdAdapter* adapter = new ThresholdAdapter();
    adapter->init(argc, argv);

    adapter->run(false);
    
    adapter->finalize();
}

ThresholdAdapter::ThresholdAdapter()
{
    port_in = new ContInPort();
    port_out = new ContOutPort();
}

void ThresholdAdapter::init(int argc, char** argv)
{
    Adapter::init(argc, argv, "Threshold");

    if (not setup->config("threshold", &threshold))
    {
        threshold = DEFAULT_THRESHOLD;
    }

    if (not setup->config("is_heaviside", &is_heaviside))
    {
        is_heaviside = DEFAULT_IS_HEAVISIDE;
    }

    if (not setup->config("scale", &scale))
    {
        scale = DEFAULT_SCALE;
    }

    if (not setup->config("shift", &shift))
    {
        shift = DEFAULT_SHIFT;
    }
}


void
ThresholdAdapter::tick()
{
    double out;
    for (int i = 0; i < port_in->data_size; ++i)
    {
        if (port_in->data[i] < threshold)
        {
            out = 0.;
        }
        else
        {
            if (is_heaviside)
            {
                out = 1.;
            }
            else
            {
                out = port_in->data[i];
            }
        }
        port_out->data[i] = scale * out + shift;
    }
}
