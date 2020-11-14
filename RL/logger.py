import os
from fastlogging import LogInit, Optimize

class FastLog:

    def __init__(self):

        dirname = os.path.dirname(__file__)
        pathName = os.path.join(dirname, 'Log/server-logfile.log')
        self.log_servr = LogInit(pathName=pathName, maxSize = 1e6, backupCnt = 1, console=False)


        self.log_servr.info("Starting logger.py...")

    def server(self, msg):
        self.log_servr.info(msg)

    def flush(self):
        self.log_servr.flush()