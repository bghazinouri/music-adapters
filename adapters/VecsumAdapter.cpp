#include "VecsumAdapter.h"

int
main(int argc, char** argv)
{

    VecsumAdapter* adapter = new VecsumAdapter();
    adapter->init(argc, argv);

    adapter->run(false);
    
    adapter->finalize();
}

VecsumAdapter::VecsumAdapter()
{
    port_in = new EventInPort();
    port_out = new ContOutPort();
}

void VecsumAdapter::init(int argc, char** argv)
{   
    //tau = DEFAULT_TAU;  It is replaced by the function to update the parameters.

    readParams();

    Adapter::init(argc, argv, "Vecsum");

    // config needed for this specific adapter

    // initialize the decay_rate for the exp. decay
    decay_rate = std::exp(-timestep/tau);

    action_dir = new double*[port_in->data_size];
    action_fr  = new double[port_in->data_size];

    for (int i = 0; i < port_in->data_size; ++i) {
        action_dir[i] = new double[2];
    // each i-th pointer is now pointing to dynamic array (port_in->data_size]) of actual int values
    }

    assign_action_to_neurons();
    initialize_action_fr();
    // action_tr.open("action_traces.dat");
    // action_vec_fl.open("action_vector.dat");
}


void
VecsumAdapter::tick()
{
    double next_t = runtime->time() + timestep;
    action_vec[0] = 0.0;
    action_vec[1] = 0.0;

    for (int i = 0; i < port_in->data_size; i++){

        action_fr[i] *= decay_rate;
        // if (next_t>90){
            // action_tr.close();
        // }
        // else{
            // action_tr << i << "\t" << action_fr[i] << "\n";
        // }
    }

    while (!static_cast<EventInPort*>(port_in)->spikes.empty () && static_cast<EventInPort*>(port_in)->spikes.top ().t < next_t)
    {
        double t_spike = static_cast<EventInPort*>(port_in)->spikes.top ().t;
        int id = static_cast<EventInPort*>(port_in)->spikes.top ().id;

        action_fr[id] += 0.0001;
        // std::cout << "Neuron # " << id << " fired at " << runtime->time() << ". Firing value is " << action_fr[id] << std::endl;
        // port_out->data[id] += (std::exp ((t_spike - runtime->time()) * inv_tau) * inv_tau);
        
        static_cast<EventInPort*>(port_in)->spikes.pop (); // remove spike from queue
    }

        // if (!static_cast<EventInPort*>(port_in)->spikes.empty () && 
        //     static_cast<EventInPort*>(port_in)->spikes.top ().t < next_t)
        // {
        //     action_fr[i] += 1.0;
        //     std::cout << "Neuron " << i << " fired! " << action_fr[i] << std::endl;
        // }
    for (int i = 0; i < port_in->data_size; i++){
        action_vec[0] += action_dir[i][0] * action_fr[i];
        action_vec[1] += action_dir[i][1] * action_fr[i];
    }
    // std::cout << port_out->data_size << std::endl;
    // std::cout << "VECSUMADAPTER" << std::endl;

    port_out->data[0] = action_vec[0];
    port_out->data[1] = action_vec[1];
    port_out->data[2] = next_t;

    /*
    for (int i = 0; i < port_out->data_size; ++i)
    {
        port_out->data[i] = action_vec[i];
        // port_out->data[i] = constact[i];
        // std::cout << "Action vec: " << i << ": " << action_vec[i] << std::endl;
    }
    */
    // action_vec_fl << action_vec[0] << "\t" << action_vec[1] << "\n";
    /*
      while (!static_cast<EventInPort*>(port_in)->spikes.empty () && static_cast<EventInPort*>(port_in)->spikes.top ().t < next_t)
      {
        double t_spike = static_cast<EventInPort*>(port_in)->spikes.top ().t;
        int id = static_cast<EventInPort*>(port_in)->spikes.top ().id;
        
        port_out->data[id] += (std::exp ((t_spike - runtime->time()) * inv_tau) * inv_tau);
        
        static_cast<EventInPort*>(port_in)->spikes.pop (); // remove spike from queue
      }

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
    */
}


void VecsumAdapter::assign_action_to_neurons()
{
    int num_action_neurons =  port_in->data_size;
    for (int i = 0; i < num_action_neurons; i++)
    {
        action_dir[i][0] = sin(2*PI*i/num_action_neurons);
        action_dir[i][1] = cos(2*PI*i/num_action_neurons);
        // std::cout << "Neuron " << i << "x: " << action_dir[i][0] << ", y: " << action_dir[i][1] << std::endl;
    }
}

void VecsumAdapter::initialize_action_fr()
{
    for (int i = 0; i < port_in->data_size; i++)
    {
        action_fr[i] = 0.0;
        // action_fr[i] = 0.0;
    }
}

void VecsumAdapter::readParams()
{
    // ********  Read From file ********
    //action_fr & tau calculation
    std::ifstream file("simulator/parameter_sets/current_parameter/network_params_spikingnet.json");
    Json::Reader reader;
    Json::Value json_file;
    reader.parse(file, json_file);
    act_fr = json_file["action"]["orientation_sel_dic"]["mov_step"].asFloat();
    tau = json_file["action"]["orientation_sel_dic"]["tau"].asFloat();
    file.close();
    // *********************************   
}