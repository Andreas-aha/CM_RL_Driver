import zmq
import numpy as np
import socket as socket0

class Server:

    def __init__(self, tcp_port=14100):

        self.tcp_port = tcp_port
        z_context = "ipc:///tmp/%d" % tcp_port

        

        # Connect to CM Simulation
        context = zmq.Context()
        self.zsocket = context.socket(zmq.PAIR)
        self.zsocket.bind(z_context)

        self.not_recieved_attempts = 0

        self.zsocket.setsockopt( zmq.LINGER,      0 )  # ____POLICY: set upon instantiations
        self.zsocket.setsockopt( zmq.AFFINITY,    1 )  # ____POLICY: map upon IO-type thread
        self.zsocket.setsockopt( zmq.RCVTIMEO,  125 )

    def server_step(self, action = [0,0]):

        #  Wait for next request from client
        #log.server("Waiting for request...")

        while self.not_recieved_attempts < 100:
            try:
                message = self.zsocket.recv_string()
                #log.server("Recieved request: %s" % message)
                self.not_recieved_attempts = 0
                self.old_msg_roh = message
                break
            except:
                #log.server("No request. Retry...")
                #print("No request. Retry...")
                self.not_recieved_attempts += 1
    
        if self.not_recieved_attempts == 100:
            message = self.old_msg_roh
        
        self.old_msg_roh = message
        # Convert message to np array
        self.state = np.fromstring(message, dtype=np.float32, sep=' ')

        array_sum = np.sum(self.state)
        array_has_nan = np.isnan(array_sum)
        array_has_inf = np.isinf(array_sum)

        if array_has_nan or array_has_inf:
            #print("Nan detected")
            try:
                self.state = self.old_msg
            except:
                pass
        else:
            self.old_msg = self.state

        self.sim_time = self.state[0]
        self.s_road = self.state[1]
        self.state = np.delete(self.state, [0, 1])
    
        #  Send reply back to client
        s_ctrl = str(action[0]) + " " + str(action[1])

        #log.server("Sending action:" + " %s" % s_ctrl)
        self.zsocket.send_string(s_ctrl)

        #print(self.state, self.sim_time, self.s_road)

        return self.state, self.sim_time, self.s_road

    def send_gui (self, msg):

        # Connect to CM GUI
        TCP_IP = socket0.gethostname()
        TCP_PORT = self.tcp_port

        self.s = socket0.socket(socket0.AF_INET, socket0.SOCK_STREAM)
        self.s.connect((TCP_IP, TCP_PORT))

        MESSAGE = "eval { %s }\n" % msg
        self.s.send(MESSAGE.encode())
        ans = self.s.recv(1024).decode().split('\r')[0]
        self.s.close()

        return "%s" % ans[1:]
