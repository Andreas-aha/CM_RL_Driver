# CM_RL_Driver
Repo for bachelor thesis "Soft Actor-Critic as Race Driver in CarMaker"
This repo contains a project folder for CM and the thesis.


# Running this project
CarMaker.linux64 needs to be compiled with make command in src folder according to Programmers Guide.
For compilation, zeromq libaries highlevel and lowlevel are required.
To run RF Learning, start RL/CM_env.py.
Tensorflow, tf_agents, etc. needed.
Communication between Python script and CM with zeromq. zeromq needs to be installed for python.

# Description of the Directories/Files
- Data: Contains TestRun Data for CarMaker.
- Movie: Geometry of the ground of the scenarios
- RL: Python scripts. Contains CarMaker_RL.ipynb file to start training.
- doc: datastorage files of racedriver, route etc.
- src: C files needed to compile CarMaker on Linux

- chart*.vg.json: Vega.js chart for jsons from the json data logger
