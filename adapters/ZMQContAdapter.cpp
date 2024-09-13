#include "ZMQContAdapter.h"

int
main(int argc, char** argv)
{

    ZMQContAdapter* adapter = new ZMQContAdapter();
    adapter->init(argc, argv);

    adapter->run(true);

    
    adapter->finalize();
}

ZMQContAdapter::ZMQContAdapter()
{
    port_in = new ZMQInPort();
    port_out = new ContOutPort();

}

void ZMQContAdapter::init(int argc, char** argv)
{
    msg_type = DEFAULT_MESSAGE_TYPE;

    Adapter::init(argc, argv, "ZMQCont");

    // config needed for this specific adapter
    std::string msg_type_;

    if (not setup->config("message_type", &msg_type_))
    {
      MUSIC::error ("message_type not specified (zmq_cont_adapter)");
    }
    if (msg_type_.compare("GymObservation") == 0) 
    {
        msg_type = GymObservation;
    }
    else if (msg_type_.compare("FloatArray") == 0)
    {
        msg_type = FloatArray;
    }
}

void
ZMQContAdapter::tick()
{
    Json::Value json_msg = static_cast<ZMQInPort*>(port_in)->recvAsJson();

    if (msg_type == FloatArray){
        for (int i = 0; i < port_out->data_size; ++i)
        {
          //TODO check for input range and rescale to [-1, 1]
          port_out->data[i] = json_msg[i].asDouble() / 2.;
        } 
    }

    if (msg_type == GymObservation){

        struct timeval now_;
        gettimeofday(&now_, NULL);
        double ts_now = now_.tv_sec + now_.tv_usec/1000000.;

        int i = 0;
        Json::Value::iterator it = json_msg.begin();
        while (it != json_msg.end()){
            Json::Value v = (*it);

            port_out->data[i] = v["value"].asFloat();

            // port_out->data[i] = 2 * (v["value"].asFloat() - v["min"].asFloat()) / 
            //           ((v["max"].asFloat() - v["min"].asFloat())) - 1;

            // double t_diff = ts_now - v["ts"].asDouble();
            // if (t_diff > 0.01){
            //     std::cout << "WARNING: ZMQ_in_adapter " << " might be out of sync" << std::endl;
            // }

            ++it;
            ++i;
        }

        if (!json_msg.empty() && json_msg[0].isMember("crnt_trial_num")) {
            port_out->data[2] = json_msg[0]["crnt_trial_num"].asFloat();
        }
        if (!json_msg.empty() && json_msg[0].isMember("crnt_trial_time")) {
            port_out->data[3] = json_msg[0]["crnt_trial_time"].asFloat();
        }
    }
}

// void
// ZMQContAdapter::asyncTick()
// {
//     Json::Value json_msg = static_cast<ZMQInPort*>(port_in)->recvAsJson();

//     if (msg_type == FloatArray){
//         for (int i = 0; i < port_out->data_size; ++i)
//         {
//           //TODO check for input range and rescale to [-1, 1]
//           port_out->data[i] = json_msg[i].asDouble() / 2.;
//         } 
//     }

//     if (msg_type == GymObservation){

//         struct timeval now_;
//         gettimeofday(&now_, NULL);
//         double ts_now = now_.tv_sec + now_.tv_usec/1000000.;

//         int i = 0;
//         Json::Value::iterator it = json_msg.begin();
//         while (it != json_msg.end()){
//             Json::Value v = (*it);

//             port_out->data[i] = v["value"].asFloat();

//             // port_out->data[i] = 2 * (v["value"].asFloat() - v["min"].asFloat()) / 
//             //           ((v["max"].asFloat() - v["min"].asFloat())) - 1;

//             double t_diff = ts_now - v["ts"].asDouble();
//             if (t_diff > 0.01){
//                 std::cout << "WARNING: ZMQ_in_adapter " << " might be out of sync" << std::endl;
//             }

//             ++it;
//             ++i;
//         }

//     }
// }

