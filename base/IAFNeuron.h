#ifndef IAF_NEURON_H
#define IAF_NEURON_H

#include <iostream>
#include <cstdlib>
#include "stdlib.h" 
#include <math.h>
#include <cmath>
#include <vector>
#include <numeric>

/**
 * Basic implementation of an Integrate-And-Fire neuron.
 * This neuron model is used for the regular rate and NEF encoding.
 */
class IAFNeuron{
    double tau_m, V_th, V_reset, I_e, C_m, R, resolution;
    double t_ref;// refactory time in ms 
    int is_ref; // refactory time in steps 
    double propagator;

    // NEF parameters
    std::vector<double> alpha;
    bool is_nef_initialized;


    public:
    double  bias, V_m, scaling;
       IAFNeuron(int dimensions, drand48_data randBuffer){

            double r;

            C_m = 250.0;
            tau_m = 10.0; 
            V_th = 15.0;
            V_reset = 0.0;
            I_e = 0.0;
            t_ref = 2;

            // dependent variables
            R = tau_m / C_m;

            drand48_r(&randBuffer, &r);
            V_m = r  * V_th;
            
            is_ref = 0.;
            resolution = 0.0;


       } 

       void setResolution(double resolution){
            this->resolution = resolution * 1000.;
            propagator = std::exp(-(this->resolution)/tau_m);

       }

       void encode(double* data){
            I_e = std::inner_product(alpha.begin(), alpha.end(), data, bias);
            //std::cout << I_e << std::endl;
       }

       bool propagate(){

           if (propagator == 0){
               std::cerr << "Resolution not set. Call setResolution() first" << std::endl;
               return false; 
           }
           bool emit_spike = false;

           if (is_ref <= 0){
//               V_m += ((-(V_m-V_reset) + R * I_e) / tau_m ) * 1.0; // forward euler?
                V_m = R * (1 - propagator) * I_e + propagator * (V_m - V_reset); // exact!

           }
           else{
               --is_ref;
           }
            
           if (V_m > V_th){
               V_m = V_reset;
               is_ref = int(t_ref/resolution);
               emit_spike = true;
           }
           return emit_spike;
           
       }

       void init_nef(int dimensions, drand48_data randBuffer, bool one_hot){

            if (one_hot == false){
                double r;

                std::vector<double> pref_direction;
                double ssum = 0;
                scaling = 100.;
                for (int i = 0; i < dimensions; ++i){
                    drand48_r(&randBuffer, &r);

                    r = 2 * r - 1.0;

                    ssum += r * r;
                    pref_direction.push_back(r);
                }
                ssum = std::sqrt(ssum);

                for (int i = 0; i < dimensions; ++i){
                    alpha.push_back((pref_direction.back() / ssum) * scaling);
                    pref_direction.pop_back();
                }

                drand48_r(&randBuffer, &r);
                r = 2 * r - 1.0;

                bias = 375.0 + r * 100;
            }
            else{
                double r;

                std::vector<double> pref_direction;
                scaling = 100.;

                drand48_r(&randBuffer, &r);
                r = (int)(r * dimensions);

                for (int i = 0; i < dimensions; ++i){
                    if (r == i)
                        pref_direction.push_back(1);
                    else
                        pref_direction.push_back(0);
                }

                for (int i = 0; i < dimensions; ++i){
                    alpha.push_back(pref_direction.back() * scaling);
                    pref_direction.pop_back();
                }

                drand48_r(&randBuffer, &r);
                r = 2 * r - 1.0;

                bias = 375.0 + r * 100;
            }

       }

};

#endif // IAF_NEURON_H
