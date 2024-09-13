#include "DopamineSpike.h"

int
main(int argc, char** argv)
{

    DopamineSpike* adapter = new DopamineSpike();
    adapter->init(argc, argv);

    adapter->run(true);

    
    adapter->finalize();
}

DopamineSpike::DopamineSpike()
{
    port_in = new ContInPort();
    port_out = new EventOutPort();

}

void DopamineSpike::init(int argc, char** argv)
{
    Adapter::init(argc, argv, "Discretize");
    rew_file.open("reward_value.dat");
    dop_spk_file.open("reward_spikes.dat");
}

void
DopamineSpike::tick()
{
    for (int i = 0; i < port_out->data_size; ++i){
        // calculate distance to this place cell 
        for (int j = 0; j < port_in->data_size; ++j){
            rew_file << runtime->time() << "\t" << port_in->data[j] << std::endl;
            if (port_in->data[j] > 0){
                // send only one spike once the reward was found
                if (abs(runtime->time() - spk_time) > timestep/5){
                    static_cast<EventOutPort*>(port_out)->send(i, runtime->time() + timestep);
                    dop_spk_file << runtime->time() + timestep << std::endl;
                }
                spk_time = runtime->time() + timestep;
            }
        }
    }
}