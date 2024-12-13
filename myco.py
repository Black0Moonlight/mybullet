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

from pymycobot.mycobot280 import MyCobot280
import time

from ps_controller import PS2Controller
ps2 = PS2Controller()
myco=False
if myco:
    mc = MyCobot280("/dev/ttyUSB0", 115200)
    mc.set_fresh_mode(1)
    mc.send_angles([
            0., 0., 0., 0., 0., 0.
        ], 50)
    time.sleep(2.5)

    def closeGriper():
        mc.set_gripper_state(1, 50)

    def openGriper():
        mc.set_gripper_state(0, 50)

joint_angles = [
            0., 0., 0., 0., 0., 0.
        ]

class ReachEnv:
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

        if self.is_render:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        # 设置相机
        p.resetDebugVisualizerCamera(cameraDistance=1.5,
                                     cameraYaw=0,
                                     cameraPitch=-40,
                                     cameraTargetPosition=[0.55, -0.35, 0.2])

        # 时间步计数器
        self.step_counter = 0

        self.urdf_root_path = pybullet_data.getDataPath()

        self.joint_damping = [
            0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001
        ]

        self.zero_joint_positions = [
            0., 0., 0., 0., 0., 0.
        ]

        self.init_joint_positions = [2.020977160069395, 0.5504488694962674, -1.9839523600153866, -0.0003392996957494921, -0.09259654241789984, 1.1893554977186631]
        self.likep = self.init_joint_positions

        self.init_quat = [-0.40001824498176575, 0.9127882122993469, -0.04376596584916115, 0.06991163641214371]

        self.seed()
        self.reset()

        self.last_time = time.time()

    def seed(self, seed=None):
        """随机种子"""
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        """环境reset，获得初始state"""
        # 初始化时间步计数器
        self.step_counter = 0

        p.resetSimulation()

        self.terminated = False
        p.setGravity(0, 0, -9.8)

        p.loadURDF(os.path.join(self.urdf_root_path, "plane.urdf"), basePosition=[0, 0, -0.2])
        search_path = "/home/blamlight/Github/mybullet/mycobot_description/urdf/mycobot_280_m5"
        p.setAdditionalSearchPath(search_path)
        urdf_file = "mycobot_280_m5.urdf"
        self.kuka_id = p.loadURDF(urdf_file, useFixedBase=True)

        # 关节角初始化
        # self.num_joints = p.getNumJoints(self.kuka_id)
        self.num_joints = 7

        for i in range(self.num_joints-1):
            p.resetJointState(
                bodyUniqueId=self.kuka_id,
                jointIndex=i+1,
                targetValue=self.init_joint_positions[i],
            )
        p.stepSimulation()

        end_link_state = p.getLinkState(self.kuka_id, self.num_joints - 1)
        self.robot_pos_obs = end_link_state[4]
        self.robot_pos_quat = end_link_state[5]
        # print(self.robot_pos_obs, self.robot_pos_quat)

        return np.hstack((np.array(self.robot_pos_obs).astype(np.float32)))

    def step(self, action):
        """根据action获取下一步环境的state、reward、done"""
        limit_x = [-0.25, 0.25]
        limit_y = [-0.25, 0.25]
        limit_z = [-0.1, 0.4]

        def clip_val(val, limit):
            if val < limit[0]:
                return limit[0]
            if val > limit[1]:
                return limit[1]
            return val

        dx = action[0]
        dy = action[1]
        dz = action[2]

        # 获取当前机械臂末端坐标
        end_link_state = p.getLinkState(self.kuka_id,
                                        self.num_joints - 1)
        self.current_pos = end_link_state[4]
        self.current_quat = end_link_state[5]
        rpy = p.getEulerFromQuaternion(self.current_quat)
        self.current_joint_positions = [p.getJointState(self.kuka_id, i)[0] for i in range(1, self.num_joints)]
        # 计算下一步的机械臂末端坐标
        self.new_robot_pos = [
            clip_val(self.current_pos[0] + dx, limit_x), clip_val(self.current_pos[1] + dy, limit_y),
            clip_val(self.current_pos[2] + dz, limit_z)
        ]
        self.new_robot_quat = p.getQuaternionFromEuler([rpy[0]+action[3], rpy[1]+action[4], rpy[2]+action[5]])
        j = (np.array(self.current_joint_positions) + np.array(self.likep)) * 0.5
        self.robot_joint_positions = p.calculateInverseKinematics(
            bodyUniqueId=self.kuka_id,
            endEffectorLinkIndex=self.num_joints - 1,
            targetPosition=[self.new_robot_pos[0], self.new_robot_pos[1], self.new_robot_pos[2]],
            currentPositions=list(j),
            targetOrientation=self.new_robot_quat,
            jointDamping=self.joint_damping,
        )

        for i in range(1, self.num_joints):
            p.resetJointState(
                bodyUniqueId=self.kuka_id,
                jointIndex=i,
                targetValue=self.robot_joint_positions[i-1],
            )
        p.stepSimulation()

        global joint_angles
        joint_angles = self.robot_joint_positions

        # 在代码开始部分，如果定义了is_good_view，那么机械臂的动作会变慢，方便观察
        if self.is_good_view:
            time.sleep(0.1)

        self.step_counter += 1

        print(list(self.robot_joint_positions), self.new_robot_pos, self.current_pos, list(self.current_quat))
        # print(self.current_pos, self.new_robot_pos)
        return 1

    def close(self):
        p.disconnect()

    #
    # def move_mycobot(self):
    #     # self.mc.sync_send_angles(list(np.degrees(self.robot_joint_positions)), 50, 0.1)
    #     last_time = time.time()
    #     self.mc.send_radians(list(self.robot_joint_positions), 50)
    #     dt = time.time()-last_time
    #     print(dt)

def map_to_range(value, old_min, old_max, new_min, new_max):
    scaled = (value - old_min) / (old_max - old_min) * (new_max - new_min) + new_min
    return scaled

import threading
def my_thread_function():
    while True:
        mc.send_radians(list(joint_angles), 50)
        time.sleep(0.05)


# 创建一个线程对象
my_thread = threading.Thread(target=my_thread_function)

# 启动线程
my_thread.start()

if __name__ == '__main__':
    env = ReachEnv(is_good_view=True, is_render=True)
    print('env={}'.format(env))
    obs = env.reset()
    action = [0,0,0,0,0,0]
    done = env.step(action)

    sum_reward = 0
    success_times = 0
    for i in range(10):
        env.reset()
        for i in range(100000):
            ps2.update_events()
            y, x = ps2.get_left_stick()
            yaw, pitch = ps2.get_right_stick()
            button = ps2.get_button_states()
            hat_button = ps2.get_dpad()

            action[0] =  map_to_range(x, 1, -1,
                                       -0.02,
                                       0.02)
            action[1] =  map_to_range(y, 1, -1,
                                      -0.02,
                                      0.02)
            if button[6]:
                z = 0.005
            elif button[7]:
                z = -0.005
            else:
                z = 0
            action[2] = z

            action[4] =  map_to_range(pitch, 1, -1,
                                       0.02,
                                       -0.02)*2
            action[5] =  map_to_range(yaw, 1, -1,
                                      -0.02,
                                      0.02)*2
            if button[4]:
                roll = 0.01
            elif button[5]:
                roll = -0.01
            else:
                roll = 0
            action[3] = roll*4

            action = np.where(np.abs(action) > 0.0005, action, 0.)

            if myco:
                if button[2]:
                    closeGriper()
                elif button[3]:
                    openGriper()

            # if button[11]:
            #     env.move_mycobot()
            done = env.step(action)