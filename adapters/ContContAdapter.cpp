#include "ContContAdapter.h"

int
main(int argc, char** argv)
{

    ContContAdapter* adapter = new ContContAdapter();
    adapter->init(argc, argv);

    adapter->run(false);
    
    adapter->finalize();
}

ContContAdapter::ContContAdapter()
{
    port_in = new ContInPort();
    port_out = new ContOutPort();
}

void ContContAdapter::init(int argc, char** argv)
{
    weights_filename = DEFAULT_WEIGHTS_FILENAME;

    Adapter::init(argc, argv, "ContCont");

    // config needed for this specific adapter
    setup->config("weights_filename", &weights_filename);
    

    weights = new double[port_out->data_size * port_in->data_size];
    for (int i = 0; i < port_out->data_size * port_in->data_size; ++i)
    {
        weights[i] = 0.; 
    }

    vec_data_in = gsl_vector_view_array(port_in->data, port_in->data_size);
    vec_data_out = gsl_vector_view_array(port_out->data, port_out->data_size);
    mat_weights = gsl_matrix_view_array(weights, port_out->data_size, port_in->data_size);
    
    readWeightsFile();
}


void
ContContAdapter::tick()
{
    gsl_blas_dgemv(CblasNoTrans, 1., &mat_weights.matrix, &vec_data_in.vector, 0., &vec_data_out.vector);
}

/**
 * Reads the weight matrix from file in JSON format.
 * If the file does not exists or can not be parsed, all weights are initialized with value 1/N where N is the width of the input port.
 */
void 
ContContAdapter::readWeightsFile()
{
    Json::Reader json_reader;

    std::ifstream weights_file;
    weights_file.open(weights_filename.c_str(), std::ios::in);
    string json_weights_ = "";
    string line;

    while (std::getline(weights_file, line))
    {
        json_weights_ += line;
    }
    weights_file.close();
    
    if ( !json_reader.parse(json_weights_, json_weights))
    {
      // report to the user the failure and their locations in the document.
      std::cout   << "WARNING: connect adapter: Failed to parse file \"" << weights_filename << "\"\n" 
		  << json_weights_ << " It has to be in JSON format.\n Using 1/N for each weight."
		  << json_reader.getFormattedErrorMessages();
        
        for (int i = 0; i < port_out->data_size * port_in->data_size; ++i)
        {
            weights[i] = 1. / port_in->data_size;
        }

        return;
    }
    else
    {
        for (int i = 0; i < port_out->data_size; ++i)
        {
            for (int j = 0; j < port_in->data_size; ++j)
            {
                weights[i*port_in->data_size + j] = json_weights[i][j].asDouble();
            }
        }

    }

}



