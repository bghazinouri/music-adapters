import json
import zmq

ctx = zmq.Context()

sub = ctx.socket(zmq.SUB)
sub.connect('tcp://localhost:5559')

sub.setsockopt(zmq.SUBSCRIBE, b'')

t_max = 0.8
t = 0

while t < t_max:
    message = json.loads(sub.recv_json())
    print(message)
    t = message[0]['value']
    assert(len(message) == 3), exit(1)  # message should have length three, as defined in MUSIC config
    assert((t > -1.) and (t < 1.)), exit(1)  # received values should be scaled to [-1, 1]

exit(0)
