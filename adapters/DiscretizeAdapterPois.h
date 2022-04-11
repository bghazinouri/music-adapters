#ifndef DISCRETIZE_ADAPTER_POIS_H
#define DISCRETIZE_ADAPTER_POIS_H

#include <iostream>
#include <map>
#include <math.h>
#include <cmath>
#include <random>

#include <music.hh>
#include <mpi.h>

#include "Adapter.h"
#include "ContInPort.h"
#include "EventOutPort.h"

#include "sys/time.h"

#include "jsoncpp/json/json.h"
#include <fstream>


/**
 * The discretize adapter models M unnormalized N-dimensional Gaussian tuning curves similar to 'place cells'.
 * Each Gaussian tuning curve is defined by mean and sigma in each dimension and is normalized between -1 and 1.
 * The equation for the activation function is:
 *
 * a_m = -1 + 2 * exp( sum_n( (a_n - mean_nm) / sigma_nm ) )
 *
 * The parameters for each Gaussian tuning curve (place cell) must be defined in an extra file in JSON format.
 * The filename can be set in MUSIC configuration using the parameter 'grid_position_filename'.
 */
class DiscretizeAdapterPois : public Adapter
{
    const string DEFAULT_GRID_POSITIONS_FILENAME = "grid_positions.json";
    const string DEFAULT_REPRESENTATION = "place";
    public:
        DiscretizeAdapterPois();
        void init(int argc, char** argv);
        void tick();
        double time;
        std::ofstream location_fl;
        //std::ofstream firing_rate;
        // random number generator
        // double sim_res = 0.01;
        int Seed = 0;
        int Simtime = 10.0;
        std::mt19937 gen;
        float firing_rate_parameter = 0.0;
        int num_place_cells = 0;
        int num_grid_cells = 0;
        int num_border_cells = 0;
        std::string data_path="./";
        const double PI = M_PI;

    private:
        string grid_positions_filename;
        std::string representation_type;
        Json::Value json_grid_positions; 
        std::map<int, double*> grid_positions;
        std::map<int, double*> sigmas;
        int *rep_type;
        float *max_fr; 
        void readGridPositionFile();
        void readSeedfromNetParams();
        void readParams();
        void readdatapath();

};

#endif // DISCRETIZE_ADAPTER_POIS_H
