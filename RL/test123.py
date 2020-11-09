import os
from subprocess import Popen, PIPE
import time

PIPE_PATH = "/tmp/my_pipe"

if not os.path.exists(PIPE_PATH):
    os.mkfifo(PIPE_PATH)

Popen(['gnome-terminal', '-e', 'tail -f %s' % PIPE_PATH])


for _ in range(5):
    with open(PIPE_PATH, "w") as p:
        p.write("Hello world!\n")
        time.sleep(1)