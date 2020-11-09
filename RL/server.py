import zmq
import os
from subprocess import Popen, PIPE
import tkinter as tk
import _thread
import numpy as np
from tf_agents.specs import array_spec

PIPE_PATH = "/tmp/my_pipe"

if not os.path.exists(PIPE_PATH):
    os.mkfifo(PIPE_PATH)

# Popen(['gnome-terminal', '-e', 'tail -f %s' % PIPE_PATH])

# class Server_GUI(tk.Frame):
#     def __init__(self, master=None):
#         super().__init__(master)
#         self.master = master
#         self.pack()
#         self.create_widgets()

#     def create_widgets(self):
#         self.quit = tk.Button(self, text="QUIT", fg="red",
#                               command=self.master.destroy)
#         self.quit.pack(side="bottom")

#         self.master.bind("<KeyPress>", self.keydown)
#         self.master.bind("<KeyRelease>", self.keyup)

#     def keydown(self, event):
#         global action
#         key = event.keysym
#         if key == "w":
#             action[0] = 1.0
#         if key == "s":
#             action[1] = 0.9
#         if key == "a":
#             action[2] = 10.0
#         elif key == "d":
#             action[2] = -10.0

#     def keyup(self, event):
#         global action
#         key = event.keysym
#         if key == "w":
#             action[0] = 0.0
#         if key == "s":
#             action[1] = 0.0
#         if key == "a":
#             action[2] = 0.0
#         if key == "d":
#             action[2] = 0.0

# def create_gui():
#     root = tk.Tk()
#     app = Server_GUI(master=root)
#     root.attributes('-topmost', True)
#     app.mainloop()

action = [0 , 0]

class Server:

    def __init__(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.PAIR)
        self.socket.bind("tcp://*:25555")

        self.socket.setsockopt( zmq.LINGER,      0 )  # ____POLICY: set upon instantiations
        self.socket.setsockopt( zmq.AFFINITY,    1 )  # ____POLICY: map upon IO-type thread
        self.socket.setsockopt( zmq.RCVTIMEO, 2000 )

        self.s_ctrl = " ".join(map(str, action))

        self.uaq_dict = np.zeros(14)
        self.old_msg = np.array([1.0, 0., 
                                  -0., -0., 2,
                                  -0., -0., 2,
                                  -0, 
                                  -0,
                                  -0,
                                  0,
                                  0,
                                  0,], dtype=np.float32)

    def server_step(self, action = [0,0]):

        

        #  Wait for next request from client
        use_old = 0
        # open(PIPE_PATH, "w").write("Received request ")
        try:
            message = self.socket.recv_string()
        except:
            open(PIPE_PATH, "w").write( "Retry..." )
            try:
                message = self.socket.recv_string()
            except:
                open(PIPE_PATH, "w").write( "Didnt work..." )
                try:
                    message = self.socket.recv_string()
                except:
                    open(PIPE_PATH, "w").write( "Using old msg" )
                    self.uaq_dict = self.old_msg
                    message = "1 2 3 4"
                    use_old = 1
        
        # Convert message to np array
        if use_old == 0:
            self.uaq_dict = np.fromstring(message, dtype=np.float32, sep=' ')
            self.old_msg = self.uaq_dict

        self.sim_time = self.uaq_dict[0]
        self.uaq_dict = np.delete(self.uaq_dict, 0)

        # open(PIPE_PATH, "w").write("Time:" + str(self.sim_time) + " Values: " +str(self.uaq_dict) + "\n")
        
        #  Send reply back to client
        s_ctrl = str(action[0]) + " " + str(action[1])

        # open(PIPE_PATH, "w").write("Sending string:" + " %s\n" % s_ctrl)
        self.socket.send_string(s_ctrl)


        return np.reshape(self.uaq_dict, (13,)), self.sim_time

