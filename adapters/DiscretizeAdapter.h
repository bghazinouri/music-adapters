#ifndef DISCRETIZE_ADAPTER_H
#define DISCRETIZE_ADAPTER_H

#include <iostream>
#include <map>
#include <math.h>
#include <cmath>

#include <music.hh>
#include <mpi.h>

#include "Adapter.h"
#include "ContInPort.h"
#include "ContOutPort.h"

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
class DiscretizeAdapter : public Adapter
{
    const string DEFAULT_GRID_POSITIONS_FILENAME = "grid_positions.json";
    public:
        DiscretizeAdapter();
        void init(int argc, char** argv);
        void tick();
        

    private:
        string grid_positions_filename;
        Json::Value json_grid_positions; 
        std::map<int, double*> grid_positions;
        std::map<int, double*> sigmas;

        void readGridPositionFile();

};

#endif // DISCRETIZE_ADAPTER_H
