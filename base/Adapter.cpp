#include "Adapter.h" 


/** 
 * Starts asynchronous thread and calls asyncTick() regularly.
 */
static void*
asyncThread(void* arg)
{
    Adapter* adapter = static_cast<Adapter*>(arg);

  
    for (int t = 0; adapter->runtime->time() < adapter->stoptime; t++)
    {
        adapter->asyncTick();

    }

    return 0;
}


/**
 * Initializes the adapter with default values and reads the configuration. Sets up the MUSIC communicator and in- and out ports.
 */
void
Adapter::init(int argc, char** argv, std::string label)
{
    timestep = DEFAULT_TIMESTEP;
    stoptime = DEFAULT_STOPTIME;
    rtf = DEFAULT_RTF;
    verbose = DEFAULT_VERBOSE;

    setup = new MUSIC::Setup (argc, argv);
    std::cout << label << "Adapter init " << std::endl;

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);
    setup->config("rtf", &rtf);
    setup->config("verbose", &verbose);

    comm = setup->communicator ();
    //int rank = comm.Get_rank ();       
    int nProcesses = comm.Get_size (); 
    if (nProcesses > 1)
    {
        std::cout << "ERROR: num processes (np) not equal 1" << std::endl;
        comm.Abort(1);
    }

    port_in->init(setup, "in");
    port_out->init(setup, "out");

    this->label = label;

}

/**
 * Sets up the MUSIC runtime and the real-time clock and runs the adapter.
 */
void 
Adapter::run(bool threaded)
{

    MPI::COMM_WORLD.Barrier();
    RTClock clock( timestep / rtf );

    runtime = new MUSIC::Runtime (setup, timestep);

    if (threaded)
    {
        pthread_create (&thread, NULL, asyncThread, this);
    }
    
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    unsigned int ticks_skipped = 0;

    for (int t = 0; runtime->time() < stoptime; t++)
    {
   
        tick();

        if (verbose){  
           std::cout << label << "Adapter: ";
           for (int i = 0; i < port_out->data_size; ++i)
           {
               std::cout << port_out->data[i] << " ";
           }
           std::cout << std::endl;
        }   

        clock.sleepNext(); 
        runtime->tick();
    }

    if (threaded)
    {
        pthread_join(thread, NULL);
    }
    
    gettimeofday(&end, NULL);
    unsigned int dt_s = end.tv_sec - start.tv_sec;
    unsigned int dt_us = end.tv_usec - start.tv_usec;
    if (end.tv_sec > start.tv_sec)
    {
        dt_us += 1000000;
    }
    std::cout << label << "Adapter: total simtime: " << dt_s << " " << dt_us << " ticks skipped " << ticks_skipped <<  std::endl;
}


/**
 * Finalizes the adapter and cleans up the MUSIC runtime.
 */
void
Adapter::finalize()
{
    runtime->finalize();
    delete runtime;
}

