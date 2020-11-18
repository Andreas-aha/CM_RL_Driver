import socket

TCP_IP = socket.gethostname()
TCP_PORT = 14100
BUFFER_SIZE = 1024
MESSAGE = "Log {Hello, World!}\n"

s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect((TCP_IP, TCP_PORT))
s.send(MESSAGE.encode())

s.close()