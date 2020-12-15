import abc
import tensorflow as tf
import numpy as np
import random
import time
import subprocess

from tf_agents.environments import py_environment
from tf_agents.specs import array_spec
from tf_agents.trajectories import time_step as ts

from server import Server



## Environment

class CarMakerEnv(py_environment.PyEnvironment):

  def __init__(self, RTFac=1, mode='collect', gamma=0.99, server=0):

    self.min_TestRun_Time = 1000
    self.max_EpTrack = 20
    if mode is 'evaluate':
      self.max_EpSteps = 6000
    else:
      self.max_EpSteps = 10000

    self.gamma = gamma
    self.RTFac = RTFac
    self.mode = mode
    
    # Open CarMaker 2 times, used to alternate between apps when car gets reset
    self.tcp_port1 = 14100 + server
    subprocess.Popen( ["/opt/ipg/carmaker/linux64-9.1/bin/CM", "-cmdport", str(self.tcp_port1), "-apphost", "localhost"] )

    time.sleep(1)

    #self.tcp_port2 = 14600 + server
    #subprocess.Popen( ["/opt/ipg/carmaker/linux64-9.1/bin/CM", "-cmdport", str(self.tcp_port2), "-apphost", "localhost"] )

    self.tcp_port = self.tcp_port1

    time.sleep(3)

    if mode is 'collect':
      self.track_list = ['Route_6.rd5']
    elif mode is 'evaluate':
      self.track_list = ['Route_6.rd5']
    else:
      raise ValueError('Unkown mode. Must be collect or evaluate not ' + str(mode))

    Server(self.tcp_port1).send_gui("Application opt {-servern %d}" % self.tcp_port1)
    Server(self.tcp_port1).send_gui("Appl::Start")

    #Server(self.tcp_port2).send_gui("Application opt {-servern %d}" % self.tcp_port2)
    #Server(self.tcp_port2).send_gui("Appl::Start")

    time.sleep(2)

    Server(self.tcp_port1).send_gui("StartSim")
    #Server(self.tcp_port2).send_gui("StartSim")


    while Server(self.tcp_port).send_gui("SimStatus") != "0":
      self._state, self.sim_time, self.s_road = Server(self.tcp_port).server_step()

    self._state, self.sim_time, self.s_road = Server(self.tcp_port).server_step()

    self.time_counter = 0
    self.bad_counter = 0
    self.ep_counter = 1
    self.s_road_old = 0

    self._episode_ended = False

  def action_spec(self):
    return array_spec.BoundedArraySpec(
      shape=(2,), dtype=np.float32, 
      minimum=[-1, -3.14], maximum=[1., 3.14], 
      name='action')

  def observation_spec(self):
    return array_spec.ArraySpec(
      shape= np.shape(self._state),
      dtype=np.float32,
      name='observation')
  
  def _reset(self):
    # Restart TestRun and get initial state

    self.ep_counter += 1

    # Alternate between 2 CM Apps
    #if self.tcp_port == self.tcp_port1:
    #  self.tcp_port = self.tcp_port2
    #else:
    #  self.tcp_port = self.tcp_port1

    #Server(self.tcp_port).send_gui("SetSimTimeAcc %d" % 999999)
    if self.ep_counter > self.max_EpTrack and self.sim_time > self.min_TestRun_Time:

      self.ep_counter = 1
      
      # Choose track of track list
      #fname = random.choice(self.track_list)
      #Server(self.tcp_port).send_gui("Scene::File_Read %s -traffic" % fname)

      while Server(self.tcp_port).send_gui("SimStatus") == "0":
        Server(self.tcp_port).send_gui("StopSim")
        time.sleep(0.5)

      Server(self.tcp_port).send_gui("StartSim")

      while Server(self.tcp_port).send_gui("SimStatus") != "0":
        Server(self.tcp_port).server_step()

      Server(self.tcp_port).server_step([5.,5.])
    else:
      # Action [5.,5.] signals CM, that Episode ended.
      Server(self.tcp_port).server_step([5.,5.])

    for i in range(3):
      Server(self.tcp_port).server_step()

    self._state, self.sim_time, self.s_road = Server(self.tcp_port).server_step()

    self.s_road_old = self.s_road 
    self.time_counter = 0
    self.bad_counter = 0
    self.last_state = {}
    self._episode_ended = False
    #Server(self.tcp_port).send_gui("MySetSimTimeAcc %d" % self.RTFac)

    return ts.restart(self._state)

  def _step(self, action):

    if self._episode_ended:
      return self._reset()

    if self.sim_time < 4.05 and self.sim_time > 4.03:
      reward = -np.square(self._state[0])*0.01
      self._episode_ended = True
    else:
      self._state, self.sim_time, self.s_road = Server(self.tcp_port).server_step(action)
      reward = (self.s_road - self.s_road_old)*10 / (1 + abs(self._state[6])*4.)
      if self._state[0] < 0.1:
        reward = -1
      self.s_road_old = self.s_road



    if self.time_counter > self.max_EpSteps:
      self._episode_ended = True

    self.time_counter += 1

    if self._episode_ended:
      return ts.termination(self._state, reward)
    else:
      return ts.transition(
          self._state, reward, discount=self.gamma)