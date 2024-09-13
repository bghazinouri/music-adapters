#!/usr/bin/python
import zmq
import time
import json
import numpy as np

ctx = zmq.Context()

pub = ctx.socket (zmq.PUB)
pub.bind ("tcp://*:4555")

i = 0

while True:
    msg = json.dumps([{"max": 2, "min": -2, "value": 2 * np.sin(i), "ts": time.time()}, {"max": 2, "min": 0, "value": np.cos(i) + 1, "ts": time.time()}, {"max": -2, "min": -8, "value": 3 * np.cos(i) - 5, "ts": time.time()}])
    pub.send (msg)
    print "send " + msg
    time.sleep(0.01)

    i += 0.01

    


