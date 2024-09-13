import zmq
import time
import json

ctx = zmq.Context()

pub = ctx.socket(zmq.PUB)
pub.bind('tcp://*:5558')

t_max = 1.2
t = 0
dt = 0.01

while t < t_max:
    msg = json.dumps([
        {'max': 1., 'min': -1., 'value': t, 'ts': time.time()}
    ])
    pub.send_json(msg)
    time.sleep(dt)

    t += dt

exit(0)
