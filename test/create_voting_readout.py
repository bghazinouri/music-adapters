import numpy as np
import pylab as plt
import json

def angular_voting_readout():
    num_neurons = 200 
    
    angle = np.linspace(0, np.pi * 2, num_neurons)
    
    translation = list(np.sin(angle) / (num_neurons / 1.))
    rotation = list(np.cos(angle) / (num_neurons / 1.))
    
    weights = [translation, rotation]
    
    print len(translation)
    
    print 
    print
    print len(rotation)
    
    print np.shape(weights)
    json.dump(weights, open("angular_voting_readout.dat", "w+"))

    #plt.plot(rotation)
    #plt.plot(translation)
    #plt.show()
    


def population_voting_readout():

    translation = []
    rotation = []

    for j in range(200):

        if j < 50:
            #translation.append(1/2000.)
            translation.append(1/200.)
            rotation.append(0.)
        elif j < 100:
            translation.append(-1/200.)
            #translation.append(0.)
            rotation.append(0.)
        elif j < 150:
            rotation.append(1/200.)
            translation.append(0.)
        else:
            rotation.append(-1/200.)
            translation.append(0.)


    weights = [translation, rotation]


    print translation
    
    print 
    print
    print rotation
    print np.shape(weights)
    json.dump(weights, open("population_voting_readout.dat", "w+"))
    
    #plt.plot(rotation)
    #plt.plot(translation)
    #plt.show()
    


#angular_voting_readout()
population_voting_readout()




