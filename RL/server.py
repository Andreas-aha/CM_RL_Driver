import zmq
import os
import tkinter as tk
import _thread
import numpy as np
from tf_agents.specs import array_spec

import socket as socket0

from logger import FastLog

log = FastLog()

class Server:

    def __init__(self):
        # Connect to CM Simulation
        context = zmq.Context()
        self.zsocket = context.socket(zmq.PAIR)
        self.zsocket.bind("tcp://*:25555")

        self.not_recieved_attempts = 0

        self.zsocket.setsockopt( zmq.LINGER,      0 )  # ____POLICY: set upon instantiations
        self.zsocket.setsockopt( zmq.AFFINITY,    1 )  # ____POLICY: map upon IO-type thread
        self.zsocket.setsockopt( zmq.RCVTIMEO, 250  )

        

    def server_step(self, action = [0,0]):

        #  Wait for next request from client
        #log.server("Waiting for request...")

        while self.not_recieved_attempts < 100:
            try:
                message = self.zsocket.recv_string()
                log.server("Recieved request: %s" % message)
                self.not_recieved_attempts = 0
                break
            except:
                #log.server("No request. Retry...")
                print("No request. Retry...")
                self.not_recieved_attempts += 1
                message = "1495.9969 0.0829 -0.3090 0.0086 -0.0018 -0.2668 -0.2983 0.3962 -0.2671 -0.3033 0.3946 -0.2946 -0.2665 0.3973 0.2664 -0.2868 0.3997 0.2664 0.0258 0.5280 0.2402 0.1601 0.6124 0.1118 -0.0830 -0.1857 -0.2216 -0.1349 -0.0189 -0.1392 0.0009 -0.0366"
        

        # Convert message to np array
        self.state = np.fromstring(message, dtype=np.float32, sep=' ')

        array_sum = np.sum(self.state)
        array_has_nan = np.isnan(array_sum)

        if array_has_nan:
            print("Nan detected")
            try:
                self.state = self.old_msg
            except:
                pass
        else:
            self.old_msg = self.state

        self.sim_time = self.state[0]
        self.state = np.delete(self.state, 0)
    
        #  Send reply back to client
        s_ctrl = str(action[0]) + " " + str(action[1])

        #log.server("Sending action:" + " %s" % s_ctrl)
        self.zsocket.send_string(s_ctrl)

        return self.state, self.sim_time

    def send_gui (self, msg):

        # Connect to CM GUI
        TCP_IP = socket0.gethostname()
        TCP_PORT = 14100

        self.s = socket0.socket(socket0.AF_INET, socket0.SOCK_STREAM)
        self.s.connect((TCP_IP, TCP_PORT))

        MESSAGE = "eval { %s }\n" % msg

        self.s.send(MESSAGE.encode())

        ans = self.s.recv(1024).decode().split('\r')[0]


        self.s.close()

        return "%s" % ans[1:]
