# from __future__ import absolute_import
# from __future__ import division
# from __future__ import print_function


import abc
import tensorflow as tf
import numpy as np
import time

import tkinter
import _thread

import base64
import matplotlib.pyplot as plt
import reverb
import tempfile
import os

from tf_agents.environments import py_environment
from tf_agents.environments import tf_environment
from tf_agents.environments import tf_py_environment
from tf_agents.environments import utils
from tf_agents.specs import array_spec
from tf_agents.environments import wrappers
from tf_agents.trajectories import time_step as ts

from tf_agents.agents.ddpg import critic_network
from tf_agents.agents.sac import sac_agent
from tf_agents.agents.sac import tanh_normal_projection_network
from tf_agents.environments import suite_pybullet
from tf_agents.experimental.train import actor
from tf_agents.experimental.train import learner
from tf_agents.experimental.train import triggers
from tf_agents.experimental.train.utils import spec_utils
from tf_agents.experimental.train.utils import strategy_utils
from tf_agents.experimental.train.utils import train_utils
from tf_agents.metrics import py_metrics
from tf_agents.networks import actor_distribution_network
from tf_agents.policies import greedy_policy
from tf_agents.policies import py_tf_eager_policy
from tf_agents.policies import random_py_policy
from tf_agents.replay_buffers import reverb_replay_buffer
from tf_agents.replay_buffers import reverb_utils

from tf_agents.utils import common as common_utils

from server import Server

import random

tempdir = "/home/vmroot/CM_Projects/CM_RL_Driver/RL/rl_data"


tcl = tkinter.Tcl()
tcl.eval("package require Tk")

## Hyperparameter

num_iterations = 4000000 # @param {type:"integer"}

initial_collect_steps = 10000 # @param {type:"integer"}
collect_steps_per_iteration = 1 # @param {type:"integer"}
replay_buffer_capacity = 100000 # @param {type:"integer"}

batch_size = 1024 # @param {type:"integer"}

critic_learning_rate = 3e-4 # @param {type:"number"}
actor_learning_rate = 3e-4 # @param {type:"number"}
alpha_learning_rate = 3e-4 # @param {type:"number"}
target_update_tau = 0.005 # @param {type:"number"}
target_update_period = 1 # @param {type:"number"}
gamma = 0.99 # @param {type:"number"}
reward_scale_factor = 1.0 # @param {type:"number"}

actor_fc_layer_params = (256, 256)
critic_joint_fc_layer_params = (256, 256)

log_interval = 500 # @param {type:"integer"}

num_eval_episodes = 10 # @param {type:"integer"}
eval_interval = 4000 # @param {type:"integer"}

policy_save_interval = 5000 # @param {type:"integer"}

speed_eval = 4

## Environment

class CarMakerEnv(py_environment.PyEnvironment):

  def __init__(self):
    # set init state
    # set init obs_spec and action_spec

    tcl.tk.eval("send CarMaker Appl::Start")
    print("send CarMaker Appl::Start")
    
    time.sleep(2)

    tcl.tk.eval("send CarMaker StartSim")
    print("send CarMaker StartSim")

    while tcl.tk.eval("send CarMaker SimStatus") != "0":
      print("send CarMaker SimStatus")
      self._state, self.sim_time =  Server().server_step()

    self.time_counter = 0
    self.bad_counter = 0
    self.last_time = -1

    self._episode_ended = False

    self._action_spec = array_spec.BoundedArraySpec(
        shape=(2,), dtype=np.float32, minimum=[-1, -3.14/8.], maximum=[1., 3.14/8.], name='action')

    self._observation_spec = array_spec.BoundedArraySpec(
                        shape= np.shape(self._state),
                        dtype=np.float32,  
                        minimum= -2,
                        maximum=  2,
                        name='observation'
    )


  def action_spec(self):
    return self._action_spec

  def observation_spec(self):
    return self._observation_spec

  def _reset(self, EOT = 0):
    # Restart TestRun and get initial state
    tcl.tk.eval("send CarMaker SetSimTimeAcc 99999")
    if EOT:

      if (random.random() < 0.5):
        fname = "Route_5.rd5"
      else:
        fname = "Route_4.rd5"


      tcl.tk.eval("send CarMaker Scene::File_Read %s -traffic" % fname)

      time.sleep(3)

      while tcl.tk.eval("send CarMaker SimStatus") == "0":
        tcl.tk.eval("send CarMaker StopSim")
        time.sleep(1)
        if tcl.tk.eval("send CarMaker SimStatus") == "0":
          Server().server_step()
      time.sleep(3)
      tcl.tk.eval("send CarMaker StartSim")

      while tcl.tk.eval("send CarMaker SimStatus") != "0":
        Server().server_step()

      self._state, self.sim_time = Server().server_step()
    else:
      Server().server_step()
      for _ in range(4):
        Server().server_step()
      self._state, self.sim_time = Server().server_step([5.,5.])

    time.sleep(0.1)
    self._state, self.sim_time = Server().server_step()

    self.time_counter = 0
    self.bad_counter = 0
    self.last_state = {}
    self._episode_ended = False
    tcl.tk.eval("send CarMaker SetSimTimeAcc %d" % CM_sim_perf)
    return ts.restart(self._state)

  def _step(self, action):

    # If TestRun end of time
    if self.sim_time >= 1800 or (self.sim_time > 3.02 and self.sim_time < 3.04):
      self._episode_ended = True
      self._reset(1)
    else:
      self.last_time = self.sim_time
    
    if self._episode_ended:
      # The last action ended the episode. Ignore the current action and start
      # a new episode.
      return self._reset()

    # Make sure that car is on track 
    if self.sim_time < 4.05 and self.sim_time > 4.03:
      print("Recv reset")
      reward = -np.square(self._state[0]*60)
      self._episode_ended = True
    else:
      self._state, self.sim_time = Server().server_step(action)
      if self._state[0] <= 0.05:
        self.bad_counter += 1
        reward = (self._state[0] * np.cos(self._state[2]*1.4) ) - 1 
        if self.bad_counter >= 40:
          reward = -10
          self._episode_ended = True
      else:
        self.bad_counter = 0
        reward = (self._state[0]*100 * np.cos(self._state[2]*3.5) )

    if self.time_counter > 1600:
      self._episode_ended = True

    self.time_counter += 1

    if self._episode_ended:
      return ts.termination(self._state, reward)
    else:
      return ts.transition(
          self._state, reward, discount=0.99)


environment = CarMakerEnv()

# One Environment for collecting data during eval, and one for eval

collect_env = environment
eval_env = environment

# Check specs

# print('Observation Spec:')
# print(environment.time_step_spec().observation)
# print('Action Spec:')
# print(environment.action_spec())

## Enable GPU
use_gpu = True 
strategy = strategy_utils.get_strategy(tpu=False, use_gpu=use_gpu)

## Agents
# All variables and Agents need to be created under strategy.scope(), as you'll see below.

# Critic for estimate of action values
# Input is observation and an action
# Output is estimate of action value (to see how good it would be)

observation_spec, action_spec, time_step_spec = (
      spec_utils.get_tensor_specs(collect_env))

with strategy.scope():
  critic_net = critic_network.CriticNetwork(
        (observation_spec, action_spec),
        observation_fc_layer_params=None,
        action_fc_layer_params=None,
        joint_fc_layer_params=critic_joint_fc_layer_params,
        kernel_initializer='glorot_uniform',
        last_kernel_initializer='glorot_uniform')

# Actor network predicts parameters for a tanh-squashed MultivariateNormalDiag distribution.
# Will be sampled conditioned on the current state (observation), if generation of action is needed.

with strategy.scope():
  actor_net = actor_distribution_network.ActorDistributionNetwork(
      observation_spec,
      action_spec,
      fc_layer_params=actor_fc_layer_params,
      continuous_projection_net=(
          tanh_normal_projection_network.TanhNormalProjectionNetwork))

# Init agent

with strategy.scope():
  train_step = train_utils.create_train_step()

  tf_agent = sac_agent.SacAgent(
        time_step_spec,
        action_spec,
        actor_network=actor_net,
        critic_network=critic_net,
        actor_optimizer=tf.compat.v1.train.AdamOptimizer(
            learning_rate=actor_learning_rate),
        critic_optimizer=tf.compat.v1.train.AdamOptimizer(
            learning_rate=critic_learning_rate),
        alpha_optimizer=tf.compat.v1.train.AdamOptimizer(
            learning_rate=alpha_learning_rate),
        target_update_tau=target_update_tau,
        target_update_period=target_update_period,
        td_errors_loss_fn=tf.math.squared_difference,
        gamma=gamma,
        reward_scale_factor=reward_scale_factor,
        train_step_counter=train_step)

  tf_agent.initialize()

## Replay Buffer

# max_size -- in a distributed setting with async collection and training, 
# you will probably want to experiment with rate_limiters.SampleToInsertRatio, 
# using a samples_per_insert somewhere between 2 and 1000

#rate_limiter=reverb.rate_limiters.SampleToInsertRatio(samples_per_insert=3.0, min_size_to_sample=3, error_buffer=3.0)

table_name = 'uniform_table'
table = reverb.Table(
    table_name,
    max_size=replay_buffer_capacity,
    sampler=reverb.selectors.Uniform(),
    remover=reverb.selectors.Fifo(),
    rate_limiter=reverb.rate_limiters.MinSize(1))

reverb_server = reverb.Server([table])

# replay buffer can be accessed with tf_agent.collect_data_spec. Gives tensors.

# sequence_length = 2 because SAC needs current and next state to compute loss (td methods)

reverb_replay = reverb_replay_buffer.ReverbReplayBuffer(
    tf_agent.collect_data_spec,
    sequence_length=2,
    table_name=table_name,
    local_server=reverb_server)

# Generate TF dataset from reverb replay buffer

dataset = reverb_replay.as_dataset(
      sample_batch_size=batch_size, num_steps=2).prefetch(50)
experience_dataset_fn = lambda: dataset

## Policies
CM_sim_perf = 99999
tcl.tk.eval("send CarMaker SetSimTimeAcc %d" % CM_sim_perf)
# Agent has 2 policies

# agent.policy — The main policy that is used for evaluation and deployment.
# agent.collect_policy — A second policy that is used for data collection.

tf_eval_policy = tf_agent.policy
eval_policy = py_tf_eager_policy.PyTFEagerPolicy(
  tf_eval_policy, use_tf_function=True)

tf_collect_policy = tf_agent.collect_policy
collect_policy = py_tf_eager_policy.PyTFEagerPolicy(
  tf_collect_policy, use_tf_function=True)

# Additional random policy for initial collector

random_policy = random_py_policy.RandomPyPolicy(
  collect_env.time_step_spec(), collect_env.action_spec())


## Actors

# Each Actor worker runs a sequence of data collection steps given the local values of the policy variables.
# Variable updates are done explicitly using the variable container client instance in the training script 
# before calling actor.run().

# As the Actors run data collection steps, they pass trajectories of (state, action, reward) to the observer, 
# which caches and writes them to the Reverb replay system. 

# storing trajectories for frames [(t0,t1) (t1,t2) (t2,t3), ...] because stride_length=1

rb_observer = reverb_utils.ReverbAddTrajectoryObserver(
  reverb_replay.py_client,
  table_name,
  sequence_length=2,
  stride_length=1)

# Actor with the random policy and collect experiences to seed the replay buffer with

print('Starting initial_collect_actor')
initial_collect_actor = actor.Actor(
  collect_env,
  random_policy,
  train_step,
  steps_per_run=initial_collect_steps,
  observers=[rb_observer])
initial_collect_actor.run()
print('Finished initial_collect_actor')

# Instantiate an Actor with the collect policy to gather more experiences during training
# actor.collect_metrics(x) = x test runs

print('Starting env_step_metric')
env_step_metric = py_metrics.EnvironmentSteps()
collect_actor = actor.Actor(
  collect_env,
  collect_policy,
  train_step,
  steps_per_run=1,
  metrics=actor.collect_metrics(10),
  summary_dir=os.path.join(tempdir, learner.TRAIN_DIR),
  observers=[rb_observer, env_step_metric])
print('Finished env_step_metric')

# Actor which will be used to evaluate the policy during training. 
# We pass in actor.eval_metrics(num_eval_episodes) to log metrics later.

print('Starting eval_actor')
eval_actor = actor.Actor(
  eval_env,
  eval_policy,
  train_step,
  episodes_per_run=num_eval_episodes,
  metrics=actor.eval_metrics(num_eval_episodes),
  summary_dir=os.path.join(tempdir, 'eval'),
)
print('Finished eval_actor')

# The Learner component contains the agent and performs gradient step updates 
# to the policy variables using experience data from the replay buffer. After 
# one or more training steps, the Learner can push a new set of variable values 
# to the variable container.

saved_model_dir = os.path.join(tempdir, learner.POLICY_SAVED_MODEL_DIR)

# Triggers to save the agent's policy checkpoints.
learning_triggers = [
    triggers.PolicySavedModelTrigger(
        saved_model_dir,
        tf_agent,
        train_step,
        interval=policy_save_interval),
    triggers.StepPerSecondLogTrigger(train_step, interval=1000),
]

agent_learner = learner.Learner(
  tempdir,
  train_step,
  tf_agent,
  experience_dataset_fn,
  triggers=learning_triggers)

## Metrics and Evaluation

def get_eval_metrics():
  eval_actor.run()
  results = {}
  for metric in eval_actor.metrics:
    results[metric.name] = metric.result()
  return results

metrics = get_eval_metrics()

def log_eval_metrics(step, metrics):
  eval_results = (', ').join(
      '{} = {:.6f}'.format(name, result) for name, result in metrics.items())
  print('step = {0}: {1}'.format(step, eval_results))

log_eval_metrics(0, metrics)

# Reset the train step
print('Reset the train step')
tf_agent.train_step_counter.assign(0)

# Evaluate the agent's policy once before training.
print('Evaluate the agents policy once before training')
avg_return = get_eval_metrics()["AverageReturn"]
returns = [avg_return]

CM_sim_perf = speed_eval
tcl.tk.eval("send CarMaker SetSimTimeAcc %d" % CM_sim_perf)

print('Training!')
for _ in range(num_iterations):
  # Training.
  
  collect_actor.run()
  loss_info = agent_learner.run(iterations=1)

  # Evaluating.
  
  step = agent_learner.train_step_numpy

  if eval_interval and step % eval_interval == 0:
    CM_sim_perf = 99999
    tcl.tk.eval("send CarMaker SetSimTimeAcc %d" % CM_sim_perf)
    metrics = get_eval_metrics()
    log_eval_metrics(step, metrics)
    CM_sim_perf = speed_eval
    tcl.tk.eval("send CarMaker SetSimTimeAcc %d" % CM_sim_perf)
    returns.append(metrics["AverageReturn"])

  if log_interval and step % log_interval == 0:
    print('step = {0}: loss = {1}'.format(step, loss_info.loss.numpy()))


rb_observer.close()
reverb_server.stop()
