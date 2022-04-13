#include "DiscretizeAdapterPois.h"

int
main(int argc, char** argv)
{

    DiscretizeAdapterPois* adapter = new DiscretizeAdapterPois();
    adapter->init(argc, argv);

    adapter->run(false);
    
    adapter->finalize();
}

DiscretizeAdapterPois::DiscretizeAdapterPois()
{
    port_in = new ContInPort();
    port_out = new EventOutPort();
}

void DiscretizeAdapterPois::init(int argc, char** argv)
{
    //grid_positions_filename = DEFAULT_GRID_POSITIONS_FILENAME;
    //representation_type = DEFAULT_REPRESENTATION;

    Adapter::init(argc, argv, "Discretize");

    // config needed for this specific adapter
    setup->config("grid_positions_filename", &grid_positions_filename);
    setup->config("representation_type", &representation_type);
    
    readParams();  //Initialize the parameters from files.
    readGridPositionFile();
    readSeedfromNetParams();
    readdatapath();

    // setting up the random number generator with seed=Seed
    // std::uniform_real_distribution<> dis(0.0, 1.0);

    gen.seed(Seed);
    std::cout << "The seed number is:" << Seed << std::endl;
    std::cout << "Data path: " << data_path+"/agents_location.dat" << std::endl;
    location_fl.open(data_path+"/agents_location.dat");
    location_fl << "time\tx\ty\n";

    // firing_rate.open(data_path+"/firing_rate.dat");
    // firing_rate << "x\ty\tID\tfr\n";

    std::cout << "\nRepresentation used is " << representation_type << "\n";

}


void
DiscretizeAdapterPois::tick()
{
    double fr_prob_tmp = 0;
    double tmp_ = 0;
    double int_mult = 0;
    // int t = 0;
    double rnd = 0.0;
    double phi_l, omega;
    double kl [2];
    std::uniform_real_distribution<> dis(0.0, 1.0);

    location_fl << runtime->time() << "\t" \
                << port_in->data[0] << "\t" \
                << port_in->data[1] << "\n";

    for (int i = 0; i < port_out->data_size; ++i){
        fr_prob_tmp = 0.;
        tmp_ = 0.;
        int_mult = 0.;
        // t = 0;
        // calculate distance to this place cell 
        if (rep_type[i] == 0){ //place
            for (int j = 0; j < port_in->data_size; ++j){
                tmp_ += std::pow((port_in->data[j] - grid_positions[i][j]) / sigmas[i][j], 2);
            }
            tmp_ = std::exp(-tmp_/2);
            // t = 1;
        }
        else if (rep_type[i] == 1){ //grid
            for (int hex_axis = 0; hex_axis < 3; hex_axis++){
                phi_l = -PI/6 + hex_axis*PI/3;
                omega = 2*PI/(sin(PI/3)*sigmas[i][0]);
                kl[0] = cos(phi_l);
                kl[1] = sin(phi_l);
                int_mult = kl[0]*(port_in->data[0] - grid_positions[i][0]) + kl[1]*(port_in->data[1] - grid_positions[i][1]);
                tmp_ += cos(omega*int_mult) - 1;
            }
            tmp_ = -tmp_*sigmas[i][1]/1.5;
            // t = 1;
        }
        else if (rep_type[i] == 2){ //border
        //     // for (int j = 0; j < port_in->data_size; ++j){
            if((grid_positions[i][0]-sigmas[i][0])<=port_in->data[0] && (grid_positions[i][0]+sigmas[i][0])>=port_in->data[0] && \
                (grid_positions[i][1]-sigmas[i][1])<=port_in->data[1] && (grid_positions[i][1]+sigmas[i][1])>=port_in->data[1]){
                tmp_ = 1;
                }
        //     //     std::cout << "border cells should be active! #" << i << std::endl;
        //     //     // }
        //     // // if (tmp_ == port_in->data_size){
        //     // //     tmp_ = 1;
        //     //     // t = 1;
        //     //     }
        //     }
        }
       
        fr_prob_tmp = max_fr[i] * timestep * tmp_;// * t;

        rnd = dis(gen);

        if (rnd < fr_prob_tmp){
            static_cast<EventOutPort*>(port_out)->send(i, runtime->time() + timestep);
        }
    }

    if (runtime->time()>=Simtime-timestep){
        location_fl.close();
    }
}

/**
 * Read the provided JSON file to extracts the multidimensional mean and sigma for the Gaussian tuning curves (place cells).
 * 
 * The file must be structured such that for each output neuron the corresponding means and sigmas are concatenated.
 * A valid JSON string for two output neurons in a two dimensional space would look like:
 *  
 * [[mean_x0, mean_y0, sigma_x0, sigma_y0],
 *  [mean_x1, mean_y1, sigma_x1, sigma_y1]]
 *
 * Example Python script to create a valid JSON file:
 *
 * @code
 * import numpy as np
 * import json
 * 
 * num_neurons_x = 5
 * num_neurons_y = 10
 * 
 * sigma_x = 0.2 
 * sigma_y = 0.1
 * 
 * pos = []
 * 
 * for i, x in enumerate(np.linspace(-1.0, 1.0, num_neurons_x)):
 *     for j, y in enumerate(np.linspace(-1.0, 1.0, num_neurons_y)):
 *         pos.append([x, y, sigma_x, sigma_y])
 * 
 * with open("grid_pos.json", "w+") as f:
 *     json.dump(pos, f)
 *
 * @endcode
 *
 * 
 */
void 
DiscretizeAdapterPois::readGridPositionFile()
{

    Json::Reader json_reader;
    int offset=0;
    std::ifstream grid_positions_file;
    grid_positions_file.open(grid_positions_filename.c_str(), std::ios::in);
    string json_grid_positions_ = "";
    string line;

    while (std::getline(grid_positions_file, line))
    {
        json_grid_positions_+= line;
    }
    grid_positions_file.close();
    
    if ( !json_reader.parse(json_grid_positions_, json_grid_positions))
    {
      // report to the user the failure and their locations in the document.
      std::cout   << "WARNING: ros discretize adapter: Failed to parse file \"" << grid_positions_filename << "\"\n" 
		  << json_grid_positions_ << " It has to be in JSON format.\n "
		  << json_reader.getFormattedErrorMessages();
        
        return;
    }
    else
    {
        // std::cout   << "Rep_type: " << representation_type << "\"\n";
        if (representation_type == "place")
        {
            offset = 0;
            // std::cout   << "If statement: Rep_type: " << "place" << "\"\n";
        }
        else if (representation_type == "grid")
        {
            offset = num_place_cells;
            // std::cout   << "If statement: Rep_type: " << "grid" << "\"\n";
        }
        else if (representation_type == "border")
        {
            offset = num_place_cells + num_grid_cells;
            // std::cout   << "If statement: Rep_type: " << "border" << "\"\n";
        }
        std::cout  << "offset=" << offset << "\"\n";
        rep_type = new int[port_out->data_size];
        max_fr = new float[port_out->data_size];
        for (int i = 0; i < port_out->data_size; ++i)
        {
            rep_type[i] = json_grid_positions[i+offset][4].asInt();
            max_fr[i] = json_grid_positions[i+offset][5].asFloat();
            
            double* pos_ = new double[port_in->data_size];
            double* sigmas_ = new double[port_in->data_size];

            for (int j = 0; j < port_in->data_size; ++j)
            {
                pos_[j] = json_grid_positions[i+offset][j].asDouble();
               
                //put sigmas on the diagonal
                sigmas_[j] = json_grid_positions[i+offset][port_in->data_size + j].asDouble(); 
            }
            grid_positions.insert(std::pair<int, double*>(i, pos_));
            sigmas.insert(std::pair<int, double*>(i, sigmas_));
        }


    }

}

void DiscretizeAdapterPois::readSeedfromNetParams()
{
    std::ifstream file("sim_params.json");
    Json::Reader reader;
    Json::Value json_file;
    reader.parse(file, json_file);
    Seed = json_file["input_rng_seed"].asInt();
    Simtime = json_file["simtime"].asFloat();
    Simtime = Simtime/1000;
    std::cout << "kernel_params: seed: " << Seed << std::endl;
    file.close();
}

// The following function helps to assign the parameters from the file for the other functions to use
void DiscretizeAdapterPois::readParams()
{

    std::ifstream file("network_params_spikingnet.json");
    Json::Reader reader;
    Json::Value json_file;
    reader.parse(file, json_file);
    firing_rate_parameter = json_file["place"]["spatial_prop"]["max_fr"].asFloat();
    num_place_cells = json_file["place"]["num_neurons"].asInt();
    num_grid_cells = json_file["grid"]["num_neurons"].asInt();
    num_border_cells = json_file["border"]["num_neurons"].asInt();
    file.close();

}

void DiscretizeAdapterPois::readdatapath()
{

    std::ifstream file("sim_params.json");
    Json::Reader reader;
    Json::Value json_file;
    reader.parse(file, json_file);
    data_path = json_file["data_path"].asString();
    file.close();

}
