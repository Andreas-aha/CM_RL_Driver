# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
# %% [markdown]
# # Reinforcement Learning with TF-Agents API with CarMaker as an Environment
# This notebook is used to train and evaluate an agent in a CarMaker environment.
# 
# First of all, necessary libaries have to be imported:

# %%
import os
# If GPU should not be used uncomment following line.
# os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

import tensorflow as tf
import time
import random
import time

from tqdm import tqdm

import logzero
from logzero import logger as logging

from tf_agents.environments import py_environment, batched_py_environment, tf_py_environment

from tf_agents.agents.ddpg import critic_network
from tf_agents.agents.sac import sac_agent
from tf_agents.agents.sac import tanh_normal_projection_network
from tf_agents.experimental.train import actor
from tf_agents.experimental.train import learner
from tf_agents.experimental.train.utils import spec_utils
from tf_agents.experimental.train.utils import strategy_utils
from tf_agents.experimental.train.utils import train_utils
from tf_agents.metrics import py_metrics, tf_metrics
from tf_agents.networks import actor_distribution_network
from tf_agents.policies import greedy_policy
from tf_agents.policies import py_tf_eager_policy, py_tf_policy
from tf_agents.policies import random_py_policy, random_tf_policy
from tf_agents.policies import policy_saver
from tf_agents.replay_buffers import reverb_replay_buffer
from tf_agents.replay_buffers import reverb_utils
from tf_agents.eval import metric_utils

from tf_agents.replay_buffers import tf_uniform_replay_buffer

from tf_agents.drivers import dynamic_step_driver, dynamic_episode_driver


from tf_agents.utils import common

from CM_py_env import CarMakerEnv
from server import Server

# %% [markdown]
# If all imports were succesful, a main directory has to be defined. This directory will contain all selected metrics saved in a tensor file, which can be viewed using tensorboard. 
# 
# Start tensorboard using following command: `tensorboard --logdir path_of_main_directory_or_root_dir`
# 
# Uncomment first 3 lines and comment line 4 to automatically choose a folder name and create a folder.

# %%
#timestr = time.strftime("%Y%m%d-%H%M%S")
#root_dir = "%s/rl_data/run%s" % (os.getcwd(),  timestr)
#os.mkdir(root_dir)
root_dir = "/home/andreas-z97x-ud3h/CM_Projects/CM_RL_Driver/RL/rl_data/first_trial_v-xy"

# %% [markdown]
# ## Hyperparameters
# Important hyperparameters of the SAC algorithm can be changed in the following cell:

# %%
num_iterations = 10000000 # number of steps

initial_collect_steps = 10000 # number of transitions to fill the replay buffer
train_steps_per_iteration = 1 # Train steps for each transition
replay_buffer_capacity = 400000 # number of transitions saved inside the replay buffer
collect_actor_num_steps = 1 # Number of transitions before a train step is made. Must be 'None' if following parameter is given.
collect_actor_num_episodes = None # Number of episodes before a train step is made.

batch_size = 512 # Number of samples which are choosen randomly inside the replay buffer.

critic_learning_rate = 4e-4 # Learning rate of the critic network
actor_learning_rate = 4e-4 # Learning rate of the actor network
alpha_learning_rate = 4e-4 # Update rate of the entropy regularization
target_update_tau = 0.005 # Not changed
target_update_period = 1 # Not changed
gamma = 0.998 # Discount factor
reward_scale_factor = 11.0 # Reward scaling

actor_fc_layer_params = (256, 256) # 2 layers with 256 units each
critic_joint_fc_layer_params = (256, 256) # 2 layers with 256 units each

log_interval = 1000 # Log interval of TensorFlow metrics after a number of steps

num_eval_episodes = 16 # Number of evalutation episodes done at each evaluation
eval_interval = 10000 # Number of steps after a evaluation is done

policy_save_interval = 10000 # Saving the policy after a number of steps to make evaluation possible after interruption or end of training.

summary_interval=1000 # ?
summaries_flush_secs=10 # ?

eval_metrics_callback=None # Not used

# Save intervals for networks and replay buffer to resume training after interruption or end of python kernel at last checkpoint.
train_checkpoint_interval=5000
policy_checkpoint_interval=5000
rb_checkpoint_interval=5000

# %% [markdown]
# ## Logging and summary/metric file writer
# 
# Create a log-file:

# %%
logzero.logfile("%s/rotating-logfile.log" % root_dir, maxBytes=1e6, backupCount=3)
tf.get_logger().setLevel('ERROR')

# %% [markdown]
# Create summary/metric file writer:

# %%
train_dir = os.path.join(root_dir, 'train')
eval_dir = os.path.join(root_dir, 'eval')

train_summary_writer = tf.compat.v2.summary.create_file_writer(
    train_dir, flush_millis=summaries_flush_secs * 1000)
train_summary_writer.set_as_default()

eval_summary_writer = tf.compat.v2.summary.create_file_writer(
    eval_dir, flush_millis=summaries_flush_secs * 1000)
eval_metrics = [
    tf_metrics.AverageReturnMetric(buffer_size=num_eval_episodes),
    tf_metrics.AverageEpisodeLengthMetric(buffer_size=num_eval_episodes)
]

# %% [markdown]
# ## Create Environments:
# 
# Multiple environments can be created and summarized to one batched environment:

# %%
# One Environment for collecting data during eval, and one for eval
#Collect langsamer

collect_env = CarMakerEnv(RTFac=999999, mode='collect', gamma=gamma, server=0)
#collect_env2 = CarMakerEnv(RTFac=2, mode='collect', gamma=gamma, server=1)
#collect_env3 = CarMakerEnv(RTFac=1, mode='collect', gamma=gamma, server=2)
#collect_env4 = CarMakerEnv(RTFac=1, mode='collect', gamma=gamma, server=3)

#collect_env = batched_py_environment.BatchedPyEnvironment( [collect_env1, collect_env2] )
#eval_env = CarMakerEnv(RTFac=999999, mode='evaluate', gamma=gamma, server=1)

#eval_env = collect_env

tf_collect_env = tf_py_environment.TFPyEnvironment(collect_env)
tf_eval_env = tf_collect_env

# %% [markdown]
# Cell to check the observation and action specs:

# %%
print('Observation Spec:')
print(tf_collect_env.time_step_spec().observation)
print('Action Spec:')
print(tf_collect_env.action_spec())

# %% [markdown]
# ## Agents 
# Critic and actor network are created. The same configuration as in the TensorFlow SAC example is used.

# %%
observation_spec, action_spec, time_step_spec = (
      spec_utils.get_tensor_specs(tf_collect_env))


critic_net = critic_network.CriticNetwork(
      (observation_spec, action_spec),
      observation_fc_layer_params=None,
      action_fc_layer_params=None,
      joint_fc_layer_params=critic_joint_fc_layer_params,
      kernel_initializer='glorot_uniform',
      last_kernel_initializer='glorot_uniform')

actor_net = actor_distribution_network.ActorDistributionNetwork(
    observation_spec,
    action_spec,
    fc_layer_params=actor_fc_layer_params,
    continuous_projection_net=(
        tanh_normal_projection_network.TanhNormalProjectionNetwork))

# %% [markdown]
# Create the SAC agent using the critic_net and the actor_net. Also, create or load a global step tensor 

# %%
# Init agent
global_step = tf.compat.v1.train.get_or_create_global_step()

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
    train_step_counter=global_step)

tf_agent.initialize()

# %% [markdown]
# ## Replay buffer
# Initialize an empty replay buffer:

# %%
replay_buffer = tf_uniform_replay_buffer.TFUniformReplayBuffer(
    tf_agent.collect_data_spec,
    batch_size=1,
    max_length=replay_buffer_capacity)

def _filter_invalid_transition(trajectories, unused_arg1):
      return ~trajectories.is_boundary()[0]

dataset = replay_buffer.as_dataset(
    sample_batch_size=batch_size, num_steps=2, num_parallel_calls=2).prefetch(50)

replay_observer = [replay_buffer.add_batch]

iterator = iter(dataset)

# %% [markdown]
# ## Policies

# %%
## Policies
# Agent has 2 policies

# agent.policy — The main policy that is used for evaluation and deployment.
# agent.collect_policy — A second policy that is used for data collection.

tf_eval_policy = greedy_policy.GreedyPolicy(tf_agent.policy)

tf_collect_policy = tf_agent.collect_policy

# Additional random policy for initial collector

random_policy = random_tf_policy.RandomTFPolicy(
  tf_collect_env.time_step_spec(), tf_collect_env.action_spec())
  

# %% [markdown]
# ## Metrics

# %%
train_metrics = [
    tf_metrics.NumberOfEpisodes(),
    tf_metrics.EnvironmentSteps(),
    tf_metrics.AverageReturnMetric(
        buffer_size=num_eval_episodes, batch_size=tf_collect_env.batch_size),
    tf_metrics.AverageEpisodeLengthMetric(
        buffer_size=num_eval_episodes, batch_size=tf_collect_env.batch_size),
]

# %% [markdown]
# ## Checkpointer

# %%
train_checkpointer = common.Checkpointer(
    ckpt_dir=train_dir,
    agent=tf_agent,
    global_step=global_step,
    metrics=metric_utils.MetricsGroup(train_metrics, 'train_metrics'))
policy_checkpointer = common.Checkpointer(
    ckpt_dir=os.path.join(train_dir, 'policy'),
    policy=tf_eval_policy,
    global_step=global_step)
rb_checkpointer = common.Checkpointer(
    ckpt_dir=os.path.join(train_dir, 'replay_buffer'),
    max_to_keep=1,
    replay_buffer=replay_buffer)

train_checkpointer.initialize_or_restore()
rb_checkpointer.initialize_or_restore()

# %% [markdown]
# ## Driver: Random Policy

# %%
# Actor with the random policy and collect experiences to seed the replay buffer with

initial_collect_actor = dynamic_step_driver.DynamicStepDriver(
  tf_eval_env,
  random_policy,
  num_steps=1,
  observers=replay_observer+train_metrics)

# %% [markdown]
# ## Driver: Collect actor
# 
# Gathers experience during training.

# %%
# Instantiate an Actor with the collect policy to gather more experiences during training

# Steps per run nun 5 statt 1, weil smoother?


if collect_actor_num_episodes is not None and collect_actor_num_steps is not None:
  raise ValueError("Define num episodes OR num steps. One of them must be None.")
elif collect_actor_num_episodes is None:
  collect_actor = dynamic_step_driver.DynamicStepDriver(
    tf_collect_env,
    tf_collect_policy,
    num_steps=collect_actor_num_steps,
    observers=replay_observer+train_metrics)
elif collect_actor_num_steps is None:
  collect_actor = dynamic_episode_driver.DynamicEpisodeDriver(
    tf_collect_env,
    tf_collect_policy,
    num_episodes=collect_actor_num_episodes,
    observers=replay_observer+train_metrics)
  

# %% [markdown]
# ## TensorFlow Graph:
# 
# Is not necessary. This was used in order to examine an potential performance gain.

# %%
initial_collect_actor.run = common.function(initial_collect_actor.run)
collect_actor.run = common.function(collect_actor.run)
tf_agent.train = common.function(tf_agent.train)

# %% [markdown]
# ## Get initial state, if resumed

# %%
time_step = None
policy_state = tf_collect_policy.get_initial_state(tf_collect_env.batch_size)
timed_at_step = global_step.numpy()
time_acc = 0


# %%
episode_val = train_metrics[0].result()
Server(14100).send_gui("DVAWrite RL_Agent.Episodes %d" % episode_val)
global_step_val = global_step.numpy()

# %% [markdown]
# ## Random samples for replay buffer
# Fill replay buffer with random samples (using the random policy)

# %%
if replay_buffer.num_frames() == 0:
    Server(14100).send_gui("DVAWrite RL_Agent.Signal %d" % 2)
    # Collect initial replay data.
    for _ in tqdm(range(initial_collect_steps)):
        initial_collect_actor.run()
        if episode_val != train_metrics[0].result().numpy():
            episode_val = train_metrics[0].result().numpy()
            Server(14100).send_gui("DVAWrite RL_Agent.Episodes %d" % episode_val)
    Server(14100).send_gui("DVAWrite RL_Agent.Signal %d" % 0)


# %%
def train_step():
    experience, _ = next(iterator)
    return tf_agent.train(experience)

train_step = common.function(train_step)

# %% [markdown]
# ## Training

# %%
while global_step_val < num_iterations:
    start_time = time.time()
    time_step, policy_state = collect_actor.run(
        time_step=time_step,
        policy_state=policy_state,
    )
    for _ in range(train_steps_per_iteration):
        train_loss = train_step()
    time_acc += time.time() - start_time

    global_step_val = global_step.numpy()
    if episode_val != train_metrics[0].result().numpy():
        episode_val = train_metrics[0].result().numpy()
        Server(14100).send_gui("DVAWrite RL_Agent.Episodes %d" % episode_val)

    if global_step_val % log_interval == 0:
        logging.info('step = %d, loss = %f', global_step_val, train_loss.loss)
        steps_per_sec = (global_step_val - timed_at_step) / time_acc
        logging.info('%.3f steps/sec', steps_per_sec)
        tf.compat.v2.summary.scalar(
            name='global_steps_per_sec', data=steps_per_sec, step=global_step)
        timed_at_step = global_step_val
        time_acc = 0

    for train_metric in train_metrics:
        train_metric.tf_summaries(
            train_step=global_step, step_metrics=train_metrics[:2])

    if global_step_val % eval_interval == 0:
        Server(14100).send_gui("StopSim")
        time.sleep(1)
        Server(14100).send_gui("StartSim")
        time.sleep(0.25)
        Server(14100).send_gui("DVAWrite RL_Agent.Signal %d" % 1)
        time.sleep(0.25)
        tf_eval_env.reset()
        results = metric_utils.eager_compute(
            eval_metrics,
            tf_eval_env,
            tf_eval_policy,
            num_episodes=num_eval_episodes,
            train_step=global_step,
            summary_writer=eval_summary_writer,
            summary_prefix='Metrics',
        )
        tf_eval_env.reset()
        time.sleep(0.5)
        Server(14100).send_gui("DVAWrite RL_Agent.Signal %d" % 0)

    if eval_metrics_callback is not None:
        eval_metrics_callback(results, global_step_val)
    metric_utils.log_metrics(eval_metrics)

    if global_step_val % train_checkpoint_interval == 0:
        train_checkpointer.save(global_step=global_step_val)

    if global_step_val % policy_checkpoint_interval == 0:
        policy_checkpointer.save(global_step=global_step_val)

    if global_step_val % rb_checkpoint_interval == 0:
        rb_checkpointer.save(global_step=global_step_val)

# %% [markdown]
# ## Evaluation of final Policy

# %%
results = metric_utils.eager_compute(
    eval_metrics,
    tf_eval_env,
    tf_eval_policy,
    num_episodes=num_eval_episodes,
    train_step=global_step,
    summary_writer=eval_summary_writer,
    summary_prefix='Metrics',
)


