#!/usr/bin/env python3
"""
UAV 目标点分配可视化 (ROS1 + rospy 版本)

用法:
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    rosrun location_allocate visualize_goals.py

灰球 = 各无人机初始位置
彩色三角 = 经匈牙利算法分配后的目标位置
虚线 = 每架无人机的分配路线
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

import rospy
from geometry_msgs.msg import PoseStamped

# ---- 硬编码初始位置 ----
ALL_UAV_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
INITIAL = {
    1:  [1.4,  0.0,  1.5],
    2:  [-0.7, 1.2,  1.5],
    3:  [-0.7, -1.2, 1.5],
    4:  [1.4,  0.0,  3.0],
    5:  [-0.7, 1.2,  3.0],
    6:  [-0.7, -1.2, 3.0],
    7:  [-0.7, 1.2,  4.0],
    8:  [-0.7, -1.2, 4.0],
    9:  [1.4,  0.0,  1.0],
    10: [-0.7, 1.2,  1.0],
}
UAV_COLORS = plt.cm.tab10(np.linspace(0, 1, 10))


class GoalListener:
    """订阅 /uav1~10/goal_pose，缓存最新目标坐标"""

    def __init__(self):
        self.target = {}
        self.received = set()

        for uid in ALL_UAV_IDS:
            topic = '/uav{}/goal_pose'.format(uid)
            rospy.Subscriber(topic, PoseStamped, self._cb, callback_args=uid)

        rospy.loginfo('视觉化监听已启动, 等待 /uav{1..10}/goal_pose ...')

    def _cb(self, msg, uid):
        p = msg.pose.position
        self.target[uid] = [p.x, p.y, p.z]
        if uid not in self.received:
            self.received.add(uid)
            rospy.loginfo('[{}/10] UAV{} -> ({:.2f}, {:.2f}, {:.2f})'.format(
                len(self.received), uid, p.x, p.y, p.z))


def main():
    rospy.init_node('goal_pose_viz')
    node = GoalListener()

    plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('UAV Goal Position Assignment', fontsize=14, fontweight='bold')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_xlim(-2, 6); ax.set_ylim(-3, 3); ax.set_zlim(0, 6)

    for uid in ALL_UAV_IDS:
        x, y, z = INITIAL[uid]
        ax.scatter(x, y, z, c='lightgray', s=100, marker='o',
                   edgecolors='gray', linewidths=0.8, alpha=0.7)
        ax.text(x, y, z - 0.18, 'U{}'.format(uid), fontsize=8, ha='center', color='gray')

    target_scatter = ax.scatter([], [], [], c=[], s=150, marker='^',
                                edgecolors='black', linewidths=0.6)
    lines = {}
    for uid in ALL_UAV_IDS:
        l, = ax.plot([], [], [], 'k--', linewidth=0.7, alpha=0.6)
        lines[uid] = l

    from matplotlib.lines import Line2D
    ax.legend([
        Line2D([0], [0], marker='o', color='w', markerfacecolor='lightgray',
               markersize=8, markeredgecolor='gray', linestyle=''),
        Line2D([0], [0], marker='^', color='w', markerfacecolor='tab:blue',
               markersize=8, markeredgecolor='black', linestyle=''),
        Line2D([0], [0], linestyle='--', color='black', linewidth=0.7)],
        ['Initial Position', 'Assigned Target', 'Assignment'],
        loc='upper left', fontsize=8)

    info_text = ax.text2D(0.02, 0.98, '', transform=ax.transAxes,
                          fontsize=10, va='top', fontfamily='monospace')

    all_done = False

    def update(_frame):
        nonlocal all_done
        # rospy 回调在 sleep 期间处理
        rospy.sleep(0.05)

        tx, ty, tz, tc = [], [], [], []
        for uid in ALL_UAV_IDS:
            if uid in node.target:
                p = node.target[uid]
                tx.append(p[0]); ty.append(p[1]); tz.append(p[2])
                tc.append(UAV_COLORS[uid - 1])
                xi, yi, zi = INITIAL[uid]
                lines[uid].set_data([xi, p[0]], [yi, p[1]])
                lines[uid].set_3d_properties([zi, p[2]])

        if tx:
            target_scatter._offsets3d = (tx, ty, tz)
            target_scatter.set_color(tc)

        n = len(node.received)
        if n == 10 and not all_done:
            all_done = True
            rospy.loginfo('全部 10 架无人机目标点已可视化!')
        info_text.set_text('Received: {} / 10{}'.format(
            n, '  [ALL DONE]' if all_done else ''))

        return [target_scatter, info_text] + list(lines.values())

    ani = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
