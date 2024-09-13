
\mainpage MUSIC Adapters Documentation 

\tableofcontents

\section intro_sec Introduction

MUSIC adapters enable the real-time exchange of data between neural network simulators such as NEST with third party software like the Open AI Gym.
The adapters do not only communicate data, but can be used for transformations such as 'signal to spike' encoding and 'spike to signal' decoding.

MUSIC adapters are based on the MUSIC library for synchronous and efficient communication between MPI processes. 
Additionally, the adapters can run asynchronously in a seperated thread.

The current adapters are implement in C++ for maximal efficiency, but can also be implemented in Python for convinience. 
As the MUSIC adapter are well abstracted, they are very easy to adapt and to extend. To implement a new adapter in C++, only the tick() function must be implemented.

\section installation Installation

Best practise is to create a dedicated 'build' and 'install' folder.

@code
mkdir build
mkdir install
cd build

cmake .. -DCMAKE_INSTALL_PREFIX:PATH=../install

make
make install

@endcode

Make sure to set the environment variables ($PATH) accordingly.


\section example_sec Example MUSIC configuration
@code
stoptime=400.
rtf=1.0
[reward]
  binary=zmq_cont_adapter
  args=
  np=1
  music_timestep=0.001
  message_type=GymObservation
  zmq_topic=
  zmq_addr=tcp://localhost:5557
[sensor]
  binary=zmq_cont_adapter
  args=
  np=1
  music_timestep=0.001
  message_type=GymObservation
  zmq_topic=
  zmq_addr=tcp://localhost:5556
[discretize]
  binary=discretize_adapter
  args=
  np=1
  music_timestep=0.001
  grid_positions_filename=grid_pos.json
[nest]
  binary=../actor_critic_network/network.py
  args=-t 400. -n 25 -m 3 -p network_params.json
  np=1
[argmax]
  binary=argmax_adapter
  args=
  np=1
  music_timestep=0.001
[command]
  binary=cont_zmq_adapter
  args=
  np=1
  min=0
  max=2
  music_timestep=0.01
  message_type=GymCommand
  zmq_topic=
  zmq_addr=tcp://*:5555
sensor.out->discretize.in[2]
discretize.out->nest.in[25]
reward.out->nest.reward_in[1]
nest.out->argmax.in[3]
argmax.out->command.in[1]
@endcode

\section content_sec Available adapters 

ArgmaxAdapter

@code
[argmax]
  binary=argmax_adapter
  args=
  np=1
  music_timestep=0.001
@endcode

ContContAdapter

@code
[ContCont]
  binary=cont_cont_adapter
  args=
  np=1
  weights_filename=weights.json
@endcode

ContZMQAdapter

@code
[ZMQ_out]
  binary=cont_zmq_adapter
  args=
  np=1
  music_timestep=0.01
  min=0
  max=2
  message_type=GymCommand
  zmq_topic=
  zmq_addr=tcp://*:5555
@endcode

DiscretizeAdapter

@code
[discretize]
  binary=discretize_adapter
  args=
  np=1
  music_timestep=0.001
  grid_positions_filename=place_cells.json
@endcode

SigmoidAdapter

@code
[sigmoid]
  binary=sigmoid_adapter
  args=
  np=1
  scale=2
  slope=4
  x_offset=4
@endcode

ThresholdAdapter

@code
[threshold]
  binary=threshold_adapter
  args=
  np=1
  music_timestep=0.001
  is_heaviside=false
  scale=1.
  shift=0.
@endcode

ZMQContAdapter

@code
[ZMQ_in]
  binary=zmq_cont_adapter
  args=
  np=1
  music_timestep=0.001
  message_type=GymObservation
  zmq_topic=
  zmq_addr=tcp://localhost:5557
@endcode

\section ports_sec Available ports

ContInPort

ContOutPort

EventInPort

EventOutPort

ZMQInPort

ZMQOutPort

\section base_classes_sec Base classes

Adapter

Port

InPort

OutPort


