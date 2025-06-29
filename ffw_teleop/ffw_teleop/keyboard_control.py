#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Wonho Yun

import tkinter as tk

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class KeyboardController(Node):

    def __init__(self):
        super().__init__('keyboard_joint_controller')

        self.controllers = {
            'arm_l': {
                'joints': [
                    'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                    'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
                    'gripper_l_joint1'
                ],
                'positions': [0.0] * 8,
                'labels': [None] * 8,
                'limits': [(-3.14, 3.14)] * 8,
                'position_step': [0.1] * 8,
                'publisher': self.create_publisher(
                    JointTrajectory,
                    '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
                    10),
                'last_positions': [0.0] * 8
            },
            'arm_r': {
                'joints': [
                    'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                    'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
                    'gripper_r_joint1'
                ],
                'positions': [0.0] * 8,
                'labels': [None] * 8,
                'limits': [(-3.14, 3.14)] * 8,
                'position_step': [0.1] * 8,
                'publisher': self.create_publisher(
                    JointTrajectory,
                    '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory', 10),
                'last_positions': [0.0] * 8
            },
            'head': {
                'joints': ['head_joint1', 'head_joint2'],
                'positions': [0.0] * 2,
                'labels': [None] * 2,
                'limits': [(-1.0, 1.0)] * 2,
                'position_step': [0.1] * 2,
                'publisher': self.create_publisher(
                    JointTrajectory, '/leader/joystick_controller_left/joint_trajectory', 10),
                'last_positions': [0.0] * 2
            },
            'lift': {
                'joints': ['lift_joint'],
                'positions': [0.0],
                'labels': [None],
                'limits': [(-1.0, 0.0)],
                'position_step': [0.05],
                'publisher': self.create_publisher(
                    JointTrajectory, '/leader/joystick_controller_right/joint_trajectory', 10),
                'last_positions': [0.0]
            }
        }

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.joint_received = False
        self.running = True

        self.root = tk.Tk()
        self.root.title('Joint Controller GUI')
        self.hold_buttons = set()
        self.duration = 1.0  # Duration of each jog in seconds
        self.num_points = 100  # Number of points in the trajectory
        self.build_gui()
        self.root.after(100, self.process_held_buttons)

        for ctrl in self.controllers.values():
            ctrl['last_positions'] = list(ctrl['positions'])

    def joint_state_callback(self, msg):
        for ctrl_key, ctrl in self.controllers.items():
            for i, joint in enumerate(ctrl['joints']):
                if joint in msg.name:
                    idx = msg.name.index(joint)
                    ctrl['positions'][i] = msg.position[idx]
                    if ctrl['labels'][i]:
                        value = msg.position[idx]
                        ctrl['labels'][i].config(text=f'{value:.2f}')
            # Update last_positions to the latest joint state
            ctrl['last_positions'] = list(ctrl['positions'])
        self.joint_received = True

    def create_smooth_trajectory(self, joint_names, start_pos, end_pos):
        traj = JointTrajectory()
        traj.joint_names = joint_names

        times = np.linspace(0, self.duration, self.num_points)

        for i in range(self.num_points):
            point = JointTrajectoryPoint()
            t = times[i]

            t_norm = t / self.duration
            t_norm2 = t_norm * t_norm
            t_norm3 = t_norm2 * t_norm
            t_norm4 = t_norm3 * t_norm
            t_norm5 = t_norm4 * t_norm

            pos_coeff = 10 * t_norm3 - 15 * t_norm4 + 6 * t_norm5
            vel_coeff = (30 * t_norm2 - 60 * t_norm3 + 30 * t_norm4) / self.duration
            acc_coeff = (60 * t_norm - 180 * t_norm2
                         + 120 * t_norm3) / (self.duration * self.duration)

            positions = []
            velocities = []
            accelerations = []

            for j in range(len(joint_names)):
                pos = start_pos[j] + (end_pos[j] - start_pos[j]) * pos_coeff
                vel = (end_pos[j] - start_pos[j]) * vel_coeff
                acc = (end_pos[j] - start_pos[j]) * acc_coeff

                positions.append(pos)
                velocities.append(vel)
                accelerations.append(acc)

            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations
            point.time_from_start.sec = int(times[i])
            point.time_from_start.nanosec = int((times[i] % 1) * 1e9)

            traj.points.append(point)

        return traj

    def send_command(self, ctrl_key):
        ctrl = self.controllers[ctrl_key]
        msg = self.create_smooth_trajectory(
            ctrl['joints'],
            ctrl['last_positions'],  # start_pos: previous command
            ctrl['positions']        # end_pos: new positions (after jog)
        )
        ctrl['publisher'].publish(msg)
        ctrl['last_positions'] = list(ctrl['positions'])  # update for next jog
        self.get_logger().info(f"{ctrl_key} smooth command: {ctrl['positions']}")

    def change_joint(self, ctrl_key, joint_index, direction):
        ctrl = self.controllers[ctrl_key]
        min_limit, max_limit = ctrl['limits'][joint_index]
        delta = direction * ctrl['position_step'][joint_index]
        before = ctrl['positions'][joint_index]
        new_pos = before + delta
        clamped_pos = max(min(new_pos, max_limit), min_limit)
        ctrl['positions'][joint_index] = clamped_pos

        if ctrl['labels'][joint_index]:
            ctrl['labels'][joint_index].config(text=f'{clamped_pos:.2f}')

        joint_name = ctrl['joints'][joint_index]
        self.get_logger().info(
            f'Joint [{ctrl_key}/{joint_name}]: {before:.3f} → {clamped_pos:.3f} (delta={delta:.3f}'
        )
        self.send_command(ctrl_key)

    def press_and_hold(self, ctrl_key, joint_index, direction):
        self.hold_buttons.add((ctrl_key, joint_index, direction))

    def release_button(self, ctrl_key, joint_index, direction):
        self.hold_buttons.discard((ctrl_key, joint_index, direction))

    def process_held_buttons(self):
        for ctrl_key, joint_index, direction in list(self.hold_buttons):
            self.change_joint(ctrl_key, joint_index, direction)
        self.root.after(100, self.process_held_buttons)

    def build_gui(self):
        row = 0
        for ctrl_key, ctrl in self.controllers.items():
            tk.Label(
                self.root,
                text=ctrl_key.upper(),
                font=('Arial', 12, 'bold')).grid(row=row, column=0, columnspan=4)
            row += 1
            for i, joint in enumerate(ctrl['joints']):
                tk.Label(self.root, text=joint).grid(row=row, column=0)
                btn_minus = tk.Button(self.root, text='-', width=3)
                btn_plus = tk.Button(self.root, text='+', width=3)
                label = tk.Label(self.root, text=f"{ctrl['positions'][i]:.2f}", width=6)
                ctrl['labels'][i] = label
                btn_minus.grid(row=row, column=1)
                btn_plus.grid(row=row, column=2)
                label.grid(row=row, column=3)
                btn_minus.bind(
                    '<ButtonPress-1>', lambda e, c=ctrl_key, j=i: self.press_and_hold(c, j, -1))
                btn_minus.bind(
                    '<ButtonRelease-1>', lambda e, c=ctrl_key, j=i: self.release_button(c, j, -1))
                btn_plus.bind(
                    '<ButtonPress-1>', lambda e, c=ctrl_key, j=i: self.press_and_hold(c, j, +1))
                btn_plus.bind(
                    '<ButtonRelease-1>', lambda e, c=ctrl_key, j=i: self.release_button(c, j, +1))
                row += 1

    def run(self):
        while not self.joint_received and rclpy.ok() and self.running:
            self.get_logger().info('Waiting for /joint_states...')
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info('GUI control ready. Close the window to exit.')
        try:
            while rclpy.ok() and self.running:
                rclpy.spin_once(self, timeout_sec=0.01)
                self.root.update()
        except tk.TclError:
            self.running = False


def main():
    rclpy.init()
    node = KeyboardController()
    try:
        node.run()
    except KeyboardInterrupt:
        node.running = False
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
