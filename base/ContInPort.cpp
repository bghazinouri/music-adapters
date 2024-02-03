#include "ContInPort.h"


void 
ContInPort::init(MUSIC::Setup* setup, char* port_name)
{

    port = setup->publishContInput(port_name);

    // get dimensions of data
    if (port->hasWidth())
    {
        data_size = port->width();
        // Print the value of data_size
    std::cout << "Data Size: " << data_size << std::endl;
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        std::exit(-1);
    }

    data = new double[data_size];
    for (int i = 0; i < data_size; ++i)
    {
        data[i] = 0.;
    }

    // Declare where in memory to put command_data
    MUSIC::ArrayData dmap(data,
      		 MPI::DOUBLE,
      		 0,
      		 data_size);
    port->map (&dmap, 0., 1, false);
 
}

