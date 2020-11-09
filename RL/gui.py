import time
import zmq
import tkinter as tk

gas = b"0.0"

class Application(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        self.hi_there = tk.Button(self)
        self.hi_there["text"] = "Gas"
        self.hi_there["command"] = self.gib_gas
        self.hi_there.pack(side="top")

        self.quit = tk.Button(self, text="QUIT", fg="red",
                              command=self.master.destroy)
        self.quit.pack(side="bottom")

    def gib_gas(self):
        global gas
        if gas == b"0.0":
            gas = b"1.0"
        else:
            gas = b"0.0"
        print(gas)


context = zmq.Context()

#  Socket to talk to server
print("Connecting to broker server")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:25556")

#  Do 10 requests, waiting each time for a response
for request in range(10):
    print("Sending request %s …" % request)
    socket.send(b"Hello")

    #  Get the reply.
    message = socket.recv()
    print("Received reply %s [ %s ]" % (request, message))


root = tk.Tk()
app = Application(master=root)
app.mainloop()