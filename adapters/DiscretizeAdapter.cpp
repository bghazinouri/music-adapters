#include "DiscretizeAdapter.h"

int
main(int argc, char** argv)
{

    DiscretizeAdapter* adapter = new DiscretizeAdapter();
    adapter->init(argc, argv);

    adapter->run(false);
    
    adapter->finalize();
}

DiscretizeAdapter::DiscretizeAdapter()
{
    port_in = new ContInPort();
    port_out = new ContOutPort();
}

void DiscretizeAdapter::init(int argc, char** argv)
{
    grid_positions_filename = DEFAULT_GRID_POSITIONS_FILENAME;

    Adapter::init(argc, argv, "Discretize");

    // config needed for this specific adapter
    setup->config("grid_positions_filename", &grid_positions_filename);
    
    readGridPositionFile();
}


void
DiscretizeAdapter::tick()
{
    for (int i = 0; i < port_out->data_size; ++i){
        double tmp_ = 0; 
    
        // calculate distance to this place cell 
        for (int j = 0; j < port_in->data_size; ++j){
            tmp_ += std::pow((port_in->data[j] - grid_positions[i][j]) / sigmas[i][j], 2);
        }
       
        // calculate activation in respect to gaussion kernel
        port_out->data[i] = -1. + 2. * std::exp(-tmp_/2.);
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
DiscretizeAdapter::readGridPositionFile()
{

    Json::Reader json_reader;

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

        for (int i = 0; i < port_out->data_size; ++i)
        {
            double* pos_ = new double[port_in->data_size];
            double* sigmas_ = new double[port_in->data_size];

            for (int j = 0; j < port_in->data_size; ++j)
            {
                pos_[j] = json_grid_positions[i][j].asDouble();
               
                //put sigmas on the diagonal
                sigmas_[j] = json_grid_positions[i][port_in->data_size + j].asDouble(); 
            }
            grid_positions.insert(std::pair<int, double*>(i, pos_));
            sigmas.insert(std::pair<int, double*>(i, sigmas_));
        }


    }

}



