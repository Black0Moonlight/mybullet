import numpy as np
import pybullet as p
import pybullet_data
import os
import gym
from gym import spaces
from gym.utils import seeding
import random
import time
import math
from config import opt

class ReachEnv(gym.Env):
    def __init__(self, is_render=False, is_good_view=False):
        """
        用于初始化reach环境中的各项参数，

        Args:
            is_render (bool):       是否创建场景可视化
            is_good_view (bool):    是否创建更优视角

        Returns:
            None
        """
        self.is_render = is_render
        self.is_good_view = is_good_view
        self.max_steps_one_episode = opt.max_steps_one_episode

        if self.is_render:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        # 机械臂移动范围限制
        self.x_low_obs = 0.2
        self.x_high_obs = 0.7
        self.y_low_obs = -0.3
        self.y_high_obs = 0.3
        self.z_low_obs = 0
        self.z_high_obs = 0.55

        # 机械臂动作范围限制
        self.x_low_action = -0.4
        self.x_high_action = 0.4
        self.y_low_action = -0.4
        self.y_high_action = 0.4
        self.z_low_action = -0.6
        self.z_high_action = 0.3

        # 设置相机
        p.resetDebugVisualizerCamera(cameraDistance=1.5,
                                     cameraYaw=0,
                                     cameraPitch=-40,
                                     cameraTargetPosition=[0.55, -0.35, 0.2])

        # 动作空间
        self.action_space = spaces.Box(
            low=np.array([self.x_low_action, self.y_low_action, self.z_low_action]),
            high=np.array([self.x_high_action, self.y_high_action, self.z_high_action]),
            dtype=np.float32)

        # 状态空间
        self.observation_space = spaces.Box(
            low=np.array([self.x_low_obs, self.y_low_obs, self.z_low_obs, self.x_low_obs, self.y_low_obs, self.z_low_obs]),
            high=np.array([self.x_high_obs, self.y_high_obs, self.z_high_obs, self.x_high_obs, self.y_high_obs, self.z_high_obs]),
            dtype=np.float32)

        # 时间步计数器
        self.step_counter = 0

        self.urdf_root_path = pybullet_data.getDataPath()
        # joint damping coefficents
        self.joint_damping = [
            0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001
        ]

        # 初始关节角度
        self.init_joint_positions = [
            0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684
        ]

        self.orientation = p.getQuaternionFromEuler(
            [0., -math.pi, math.pi / 2.])

        self.seed()
        self.reset()

    def seed(self, seed=None):
        """随机种子"""
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        """环境reset，获得初始state"""
        # 初始化时间步计数器
        self.step_counter = 0

        p.resetSimulation()
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

        # 初始化重力以及运行结束标志
        self.terminated = False
        p.setGravity(0, 0, -9.8)

        # 状态空间的限制空间可视化，以白线标识
        p.addUserDebugLine(
            lineFromXYZ=[self.x_low_obs, self.y_low_obs, 0],
            lineToXYZ=[self.x_low_obs, self.y_low_obs, self.z_high_obs])
        p.addUserDebugLine(
            lineFromXYZ=[self.x_low_obs, self.y_high_obs, 0],
            lineToXYZ=[self.x_low_obs, self.y_high_obs, self.z_high_obs])
        p.addUserDebugLine(
            lineFromXYZ=[self.x_high_obs, self.y_low_obs, 0],
            lineToXYZ=[self.x_high_obs, self.y_low_obs, self.z_high_obs])
        p.addUserDebugLine(
            lineFromXYZ=[self.x_high_obs, self.y_high_obs, 0],
            lineToXYZ=[self.x_high_obs, self.y_high_obs, self.z_high_obs])

        p.addUserDebugLine(
            lineFromXYZ=[self.x_low_obs, self.y_low_obs, self.z_high_obs],
            lineToXYZ=[self.x_high_obs, self.y_low_obs, self.z_high_obs])
        p.addUserDebugLine(
            lineFromXYZ=[self.x_low_obs, self.y_high_obs, self.z_high_obs],
            lineToXYZ=[self.x_high_obs, self.y_high_obs, self.z_high_obs])
        p.addUserDebugLine(
            lineFromXYZ=[self.x_low_obs, self.y_low_obs, self.z_high_obs],
            lineToXYZ=[self.x_low_obs, self.y_high_obs, self.z_high_obs])
        p.addUserDebugLine(
            lineFromXYZ=[self.x_high_obs, self.y_low_obs, self.z_high_obs],
            lineToXYZ=[self.x_high_obs, self.y_high_obs, self.z_high_obs])

        # 载入平面
        p.loadURDF(os.path.join(self.urdf_root_path, "plane.urdf"), basePosition=[0, 0, 0])
        # 载入机械臂
        search_path = "/home/blamlight/Github/mybullet/mycobot_description/urdf/mycobot_280_m5"

        p.setAdditionalSearchPath(search_path)
        urdf_file = "mycobot_280_m5.urdf"
        self.kuka_id = p.loadURDF(urdf_file, useFixedBase=True)
        # 载入桌子
        # p.loadURDF(os.path.join(self.urdf_root_path, "table/table.urdf"), basePosition=[0.5, 0, -0.65])
        # p.loadURDF(os.path.join(self.urdf_root_path, "tray/traybox.urdf"), basePosition=[0.55,0,0])
        # object_id=p.loadURDF(os.path.join(self.urdf_root_path, "random_urdfs/000/000.urdf"), basePosition=[0.53,0,0.02])

        # xpos = random.uniform(self.x_low_obs, self.x_high_obs)
        # ypos = random.uniform(self.y_low_obs, self.y_high_obs)
        # zpos = random.uniform(self.z_low_obs, self.z_high_obs) # TODO 原z=0.01
        # ang = 3.14 * 0.5 + 3.1415925438 * random.random()
        # orn = p.getQuaternionFromEuler([0, 0, ang])
        # # 载入物体
        # self.object_id = p.loadURDF("../models/cube_small_target_push.urdf",
        #                             basePosition=[xpos, ypos, zpos],
        #                             baseOrientation=[orn[0], orn[1], orn[2], orn[3]],
        #                             useFixedBase=1)
        # 关节角初始化
        self.num_joints = p.getNumJoints(self.kuka_id)
        for i in range(1, self.num_joints-1):
            p.resetJointState(
                bodyUniqueId=self.kuka_id,
                jointIndex=i,
                targetValue=self.init_joint_positions[i-1],
            )
        # for i in range(self.num_joints):
        #      print(p.getJointInfo(self.kuka_id, i))

        self.robot_pos_obs = p.getLinkState(self.kuka_id,
                                            self.num_joints - 1)[4]
        # print(self.robot_pos_obs)
        # logging.debug("init_pos={}\n".format(p.getLinkState(self.kuka_id,self.num_joints-1)))
        p.stepSimulation()
        # self.object_pos = p.getBasePositionAndOrientation(self.object_id)[0]

        goal = [random.uniform(self.x_low_obs, self.x_high_obs),
                random.uniform(self.y_low_obs, self.y_high_obs),
                random.uniform(self.z_low_obs, self.z_high_obs)]
        # self.object_state = np.array(
        #     p.getBasePositionAndOrientation(self.object_id)[0]).astype(
        #     np.float32)
        # return np.array(self.object_pos).astype(np.float32), self.object_state
        return np.hstack((np.array(self.robot_pos_obs).astype(np.float32)))

    def step(self, action):
        """根据action获取下一步环境的state、reward、done"""
        limit_x = [0.2, 0.7]
        limit_y = [-0.3, 0.3]
        limit_z = [0, 0.55]

        def clip_val(val, limit):
            if val < limit[0]:
                return limit[0]
            if val > limit[1]:
                return limit[1]
            return val
        dv = opt.reach_ctr
        dx = action[0] * dv
        dy = action[1] * dv
        dz = action[2] * dv

        # 获取当前机械臂末端坐标
        self.current_pos = p.getLinkState(self.kuka_id, self.num_joints - 1)[4]
        # 计算下一步的机械臂末端坐标
        self.new_robot_pos = [
            clip_val(self.current_pos[0] + dx, limit_x), clip_val(self.current_pos[1] + dy, limit_y),
            clip_val(self.current_pos[2] + dz, limit_z)
        ]
        # 通过逆运动学计算机械臂移动到新位置的关节角度
        self.robot_joint_positions = p.calculateInverseKinematics(
            bodyUniqueId=self.kuka_id,
            endEffectorLinkIndex=self.num_joints - 1,
            targetPosition=[self.new_robot_pos[0], self.new_robot_pos[1], self.new_robot_pos[2]],
            targetOrientation=self.orientation,
            jointDamping=self.joint_damping,
        )
        # 使机械臂移动到新位置
        for i in range(1, self.num_joints):
            p.resetJointState(
                bodyUniqueId=self.kuka_id,
                jointIndex=i,
                targetValue=self.robot_joint_positions[i-1],
            )
        p.stepSimulation()

        # 在代码开始部分，如果定义了is_good_view，那么机械臂的动作会变慢，方便观察
        if self.is_good_view:
            time.sleep(0.05)

        self.step_counter += 1

        print(self.current_pos, self.new_robot_pos)
        return 1


    def close(self):
        p.disconnect()

if __name__ == '__main__':
    # 这一部分是做baseline，即让机械臂随机选择动作，看看能够得到的分数
    env = ReachEnv(is_good_view=True, is_render=True)
    print('env={}'.format(env))
    obs = env.reset()
    action = env.action_space.sample()
    done = env.step(action)

    sum_reward = 0
    success_times = 0
    for i in range(100):
        env.reset()
        for i in range(1000):
            action = env.action_space.sample()
            done = env.step(action)
            # print('done={}'.format(done))
        # time.sleep(0.1)
    print()
    print('sum_reward={}'.format(sum_reward))
    print('success rate={}'.format(success_times / 50))