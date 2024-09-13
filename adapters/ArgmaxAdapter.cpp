#include "ArgmaxAdapter.h"

int
main(int argc, char** argv)
{

    ArgmaxAdapter* adapter = new ArgmaxAdapter();
    adapter->init(argc, argv);

    adapter->run(false);
    
    adapter->finalize();
}

ArgmaxAdapter::ArgmaxAdapter()
{
    port_in = new ContInPort();
    port_out = new ContOutPort();
}

void ArgmaxAdapter::init(int argc, char** argv)
{
    Adapter::init(argc, argv, "Argmax");

    // config needed for this specific adapter
}


void
ArgmaxAdapter::tick()
{
      double max = -DBL_MAX;
      double argmax = 0;
      for (int i = 0; i < port_in->data_size; ++i)
      {
          if (port_in->data[i] > max){
              max = port_in->data[i];
              argmax = i;
          }
      }

      port_out->data[0] = argmax;
   
}



