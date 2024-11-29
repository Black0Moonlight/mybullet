#!/usr/bin/env python
# -*- coding: utf-8 -*-

import warnings

class DefaultConfig(object):
    # global parameter
    env = 'RLReachEnv'   # env name, need to be the same as envs/__init__.py
    """Choose from RLReachEnv / RLPushEnv / RLPickEnv / RLCamReachEnv"""
    algo = 'DADDPG_MLP'  # algo name, need to be the same as algo/__init__.py中的名字一致
    """Choose from DDPG_MLP / TD3_MLP / DADDPG_MLP / DATD3_MLP / DARC_MLP / DDPG_CNN / TD3_CNN / DADDPG_CNN / DATD3_CNN / DARC_CNN"""
    vis_name = 'Reach_DADDPG'  # visdom env
    vis_port = 8097      # visdom port
    jsonfile = "visdata/push/updata_TD3/TD3.json"     # json file dir
    csvname = "visdata/push/updata_TD3/updata_TD3_"   # data save dir

    # reach env parameter
    reach_ctr = 0.02     # to control the robot arm moving rate every step
    reach_dis = 0.01     # to control the target distance

    # train parameter
    # use_gpu = True       # user GPU or not
    # device = t.device('cuda') if use_gpu else t.device('cpu')
    random_seed = 0
    num_episodes = 500   # number of training episodes
    n_train = 40         # number of network updates per episodes
    minimal_episodes = 5  # Minimum number of start rounds for the experience replay buffer
    max_steps_one_episode = 500  # Maximum number of simulation steps per round

    # net parameter
    actor_lr = 1e-3      # actor net learning rate
    critic_lr = 1e-3     # critic net learning rate
    hidden_dim = 256     # mlp hidden size
    batch_size = 256     # batch size

    # public algo parameter
    sigma = 0.1          # Standard Deviation of Gaussian Noise
    tau = 0.005          # Target network soft update parameters
    gamma = 0.98         # discount
    buffer_size = 1000000   # buffer size

    # DQN algo only
    epsilon = 0.01
    target_update = 10

    # TD3, DATD3 algo only
    policy_noise = 0.2   # policy noise
    noise_clip = 0.5     # noise clip
    policy_freq = 3      # Delay update frequency

    # DARC algo only
    q_weight = 0.2
    regularization_weight = 0.005

    # HER algo only
    her_ratio = 0.8      # her rate per batch

opt = DefaultConfig()