"""
location_allocate.py - ROS1 + rospy 版本
从 ROS2 (rclpy) 移植到 ROS1 (rospy)

核心算法 (FormationGenerator, TopologyAllocator) 保持完全不变。
ROS API 变化:
- rclpy.init() → rospy.init_node()
- Node 继承 → 普通类
- create_publisher → rospy.Publisher
- create_subscription → rospy.Subscriber (callback_args 传 uid)
- rclpy.spin_once → rospy.sleep (自动处理回调)
- get_clock().now() → rospy.Time.now()
- get_logger() → rospy.log*()
"""

import math
import time
import numpy as np
from typing import List, Dict
from scipy.optimize import linear_sum_assignment
import json

# ---- ROS1 依赖 ----
import rospy
from uav_swarm_interfaces.msg import UAVSwarmCommand, UAVStatus
from geometry_msgs.msg import Point
# --------------------
from location_allocate.no_location import parse_uav_command

# ====================== 硬编码：无人机初始坐标 + ID ======================
all_uav_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
all_initial_positions = [
    [1.4, 0.0, 1.5], [-0.7, 1.2, 1.5], [-0.7, -1.2, 1.5],
    [1.4, 0.0, 3.0], [-0.7, 1.2, 3.0], [-0.7, -1.2, 3.0],
    [-0.7, 1.2, 4.0], [-0.7, -1.2, 4.0], [1.4, 0.0, 1.0],
    [-0.7, 1.2, 1.0]
]

# ====================== 1. 坐标生成层 (不变) ======================
class FormationGenerator:
    def __init__(self, global_center: List[float], formation_radius: float):
        self.center = global_center
        self.radius = formation_radius

    def generate_line(self, n: int) -> List[List[float]]:
        points = []
        start_x = self.center[0] - (n - 1) * self.radius / 2
        for i in range(n):
            points.append([start_x + i * self.radius, self.center[1], self.center[2]])
        return points

    def generate_circle(self, n: int) -> List[List[float]]:
        points = []
        for i in range(n):
            theta = 2 * math.pi * i / n
            x = self.center[0] + self.radius * math.cos(theta)
            y = self.center[1] + self.radius * math.sin(theta)
            points.append([x, y, self.center[2]])
        return points

    def generate_sphere(self, n: int) -> List[List[float]]:
        points = []
        phi = math.pi * (3. - math.sqrt(5.))
        for i in range(n):
            y_norm = 1 - (i / float(n - 1)) * 2
            radius_at_y = math.sqrt(1 - y_norm * y_norm)
            theta = phi * i
            x = self.center[0] + math.cos(theta) * radius_at_y * self.radius
            y = self.center[1] + y_norm * self.radius
            z = self.center[2] + math.sin(theta) * radius_at_y * self.radius
            points.append([x, y, z])
        return points

    def generate(self, formation_type: str, uav_count: int) -> List[List[float]]:
        if formation_type in ["Line", "Lineup"]: return self.generate_line(uav_count)
        elif formation_type in ["Circle", "Polygon", "Triangle"]: return self.generate_circle(uav_count)
        elif formation_type == "Sphere": return self.generate_sphere(uav_count)
        elif formation_type == "Free": return []
        else: raise ValueError("不支持的编队类型: {}".format(formation_type))

# ====================== 2. 匈牙利算法分配层 (不变) ======================
class TopologyAllocator:
    @staticmethod
    def _is_segments_cross(p1, p2, p3, p4):
        if max(p1[0], p2[0]) < min(p3[0], p4[0]) or max(p3[0], p4[0]) < min(p1[0], p2[0]): return False
        if max(p1[1], p2[1]) < min(p3[1], p4[1]) or max(p3[1], p4[1]) < min(p1[1], p2[1]): return False
        cross = lambda a, b: a[0]*b[1] - a[1]*b[0]
        v1, v2, v3 = p3-p1, p4-p1, p2-p1
        c1, c2 = cross(v1, v3), cross(v2, v3)
        v1, v2, v3 = p1-p3, p2-p3, p4-p3
        c3, c4 = cross(v1, v3), cross(v2, v3)
        return (c1 * c2 < 0) and (c3 * c4 < 0)

    @staticmethod
    def allocate(initial, target, cross_penalty=10.0):
        n = len(initial)
        init_np, tgt_np = np.array(initial), np.array(target)
        cost = np.linalg.norm(init_np[:, None] - tgt_np, axis=2)

        row_ind, col_ind = linear_sum_assignment(cost)
        total_dist = np.linalg.norm(init_np[row_ind] - tgt_np[col_ind], axis=1).sum()
        rospy.loginfo("   全局最优总飞行距离: {:.3f} 米".format(total_dist))

        res = [[] for _ in range(n)]
        for r, c in zip(row_ind, col_ind): res[r] = target[c]
        return res

# ====================== 3. ROS1 核心调度层 ======================
class UAVFormationNode:
    def __init__(self):
        # 状态变量：由 C++ 节点低频发布的 /uav{id}/odom 实时更新
        self.uav_state_map: Dict[int, List[float]] = {}
        for uid in all_uav_ids:
            self.uav_state_map[uid] = [0.0, 0.0, 0.0]

        # ---- 发布者 ----
        self.publisher = {}
        for uid in all_uav_ids:
            topic_name = '/uav{}/swarm_command'.format(uid)
            self.publisher[uid] = rospy.Publisher(topic_name, UAVSwarmCommand, queue_size=10)
            rospy.loginfo("创建发布者: {}".format(topic_name))

        # ---- 订阅者 (odom 位置 + status 状态) ----
        self.uav_hover_status: Dict[int, bool] = {}
        for uid in all_uav_ids:
            self.uav_hover_status[uid] = False
            # 订阅悬停状态 (callback_args 传递 uid)
            topic_name = '/uav{}/status'.format(uid)
            rospy.Subscriber(topic_name, UAVStatus, self._status_callback, callback_args=uid)
            # 订阅 ENU 位置
            topic_name = '/uav{}/odom'.format(uid)
            rospy.Subscriber(topic_name, Point, self._odom_callback, callback_args=uid)
            rospy.loginfo("创建订阅者: /uav{}/status, /uav{}/odom".format(uid, uid))

    def _publish_single_goal(self, uav_id: int, position: List[float],
                             duration: float, motion_style: str, safety_factor: float):
        """向单个无人机发送 swarm_command 自定义消息"""
        if uav_id not in self.publisher:
            rospy.logwarn("未找到 UAV{} 的发布者，跳过".format(uav_id))
            return

        msg = UAVSwarmCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.uav_id = uav_id
        msg.target_pos.x = position[0]
        msg.target_pos.y = position[1]
        msg.target_pos.z = position[2]
        msg.duration = duration
        msg.motion_style = motion_style
        msg.safety_factor = safety_factor

        self.publisher[uav_id].publish(msg)

    def _status_callback(self, msg: UAVStatus, uid: int):
        """接收 C++ 执行层反馈的悬停状态 (callback_args=uid)"""
        if msg.is_hover_stable and not self.uav_hover_status.get(uid, False):
            rospy.loginfo("   >>> UAV{} 到达目标并悬停稳定!".format(uid))
        self.uav_hover_status[uid] = msg.is_hover_stable

    def _odom_callback(self, msg: Point, uid: int):
        """接收 C++ 节点低频发布的 ENU 位置"""
        self.uav_state_map[uid] = [msg.x, msg.y, msg.z]

    def send_goal_positions(self, task_uav_ids: List[int], allocated_positions: List[List[float]],
                            task: Dict):
        """广播 UAVSwarmCommand 自定义消息"""
        rospy.loginfo(">>> 正在向 {} 架无人机发送 swarm_command ...".format(len(task_uav_ids)))

        duration = float(task.get('duration_seconds', 3.0))
        motion_style = task.get('motion_profile', 'normal')
        val = task.get('iapf_safety_margin_factor')
        safety_factor = float(val) if val is not None else 1.0

        # 先重置悬停状态
        for uid in task_uav_ids:
            self.uav_hover_status[uid] = False

        for uid, pos in zip(task_uav_ids, allocated_positions):
            self._publish_single_goal(uid, pos, duration, motion_style, safety_factor)
            rospy.loginfo("UAV{} -> {} dur={}s style={} sf={}".format(
                uid, [round(x, 2) for x in pos], duration, motion_style, safety_factor))

    def wait_for_hover_and_time(self, task_uav_ids: List[int], wait_seconds: float, timeout: float = 120.0):
        """等待所有参与任务的无人机到达目标并悬停稳定"""
        rospy.loginfo(">>> 等待 {} 架无人机到达并悬停 (超时: {}s) ...".format(len(task_uav_ids), timeout))

        # 先重置悬停状态，排空 DDS 队列中旧消息
        for uid in task_uav_ids:
            self.uav_hover_status[uid] = False
        rospy.sleep(2.0)  # 2 秒排空旧消息 (rospy.sleep 内部处理回调)

        start_time = time.time()
        while time.time() - start_time < timeout:
            rospy.sleep(0.2)  # rospy.sleep 自动处理回调

            all_stable = all(self.uav_hover_status.get(uid, False) for uid in task_uav_ids)
            if all_stable:
                elapsed = time.time() - start_time
                rospy.loginfo("   >>> 全部 {} 架无人机已悬停稳定! (耗时 {:.1f}s)".format(
                    len(task_uav_ids), elapsed))

                rospy.loginfo(">>> 开始悬停计时: {} 秒".format(wait_seconds))
                hover_start = time.time()
                while time.time() - hover_start < wait_seconds:
                    rospy.sleep(0.2)
                rospy.loginfo("   悬停等待完成，准备执行下一任务")
                return

            # 打印进度 (每 5 秒)
            elapsed = int(time.time() - start_time)
            if elapsed % 5 == 0 and elapsed > 0:
                stable_count = sum(1 for uid in task_uav_ids if self.uav_hover_status.get(uid, False))
                rospy.loginfo("   等待中... {}/{} 已稳定".format(stable_count, len(task_uav_ids)))

        stable_list = [uid for uid in task_uav_ids if self.uav_hover_status.get(uid, False)]
        unstable_list = [uid for uid in task_uav_ids if not self.uav_hover_status.get(uid, False)]
        rospy.logwarn(">>> 悬停等待超时! 已稳定: {}, 未稳定: {}".format(stable_list, unstable_list))

    def execute_task(self, task: Dict, skip_wait: bool = False):
        """执行单步任务"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("执行任务 {}".format(task['task_sequence_id']))

        center = task['global_center']
        radius = task['parametric_data']['formation_radius']
        f_type = task['parametric_data']['formation_type']
        task_uav_ids: List[int] = task['uav_id']
        task_uav_count: int = task['uav_count']

        rospy.loginfo("任务参与无人机ID: {}".format(task_uav_ids))

        # 收一轮 odom 数据
        for _ in range(10):
            rospy.sleep(0.05)

        # 提取参与机当前位置
        current_subset = []
        for uid in task_uav_ids:
            if uid in self.uav_state_map:
                current_subset.append(self.uav_state_map[uid].copy())
            else:
                rospy.logerr("严重错误：数据库中找不到 UAV{} 的位置！".format(uid))
                return

        rospy.loginfo("   ---------- 本次任务起始坐标 ----------")
        for uid, pos in zip(task_uav_ids, current_subset):
            rospy.loginfo("   {:<8} | [{:.2f}, {:.2f}, {:.2f}]".format(uid, pos[0], pos[1], pos[2]))
        rospy.loginfo("   ---------------------------------------")

        # 生成目标坐标
        generator = FormationGenerator(center, radius)
        targets = generator.generate(f_type, task_uav_count)

        if not targets:
            rospy.loginfo("编队类型: Free (返回初始点)")
            targets = []
            for uid in task_uav_ids:
                idx = all_uav_ids.index(uid)
                targets.append(all_initial_positions[idx].copy())
        else:
            rospy.loginfo("编队类型: {} | 中心: {} | 半径: {}".format(f_type, center, radius))

        # 匈牙利算法分配
        allocator = TopologyAllocator()
        allocated_subset = allocator.allocate(current_subset, targets)

        rospy.loginfo("   ---------- 分配结果映射表 ----------")
        for uid, pos in zip(task_uav_ids, allocated_subset):
            rospy.loginfo("   {:<8} | [{:.2f}, {:.2f}, {:.2f}]".format(uid, pos[0], pos[1], pos[2]))
        rospy.loginfo("   ---------------------------------------")

        # ROS 发送坐标
        self.send_goal_positions(task_uav_ids, allocated_subset, task)

        # 更新全局状态地图
        for uid, new_pos in zip(task_uav_ids, allocated_subset):
            self.uav_state_map[uid] = new_pos.copy()

        # 处理阻塞逻辑
        if not skip_wait:
            if task.get('trigger_condition') in ('hover_and_wait', 'continuous_transit', 'direct_execution') \
               or task.get('task_sequence_id', 1) > 1:
                wt = task.get('wait_time') or 0.0
                self.wait_for_hover_and_time(task_uav_ids, wt)

    def run_mission(self, llm_output: Dict):
        tasks = llm_output.get('task_sequences', [])
        if not tasks:
            rospy.logerr("LLM 输出为空，没有任务可执行")
            return

        i = 0
        while i < len(tasks):
            # 收集连续、UAV 集合不重叠的任务编组（并行执行）
            group = [tasks[i]]
            group_ids = set(tasks[i].get('uav_id', []))
            j = i + 1
            while j < len(tasks):
                next_ids = set(tasks[j].get('uav_id', []))
                if group_ids & next_ids:
                    break
                group.append(tasks[j])
                group_ids |= next_ids
                j += 1

            if len(group) > 1:
                rospy.loginfo(">>> 并行执行任务 {}-{}（UAV 集合不重叠）".format(i+1, j))
                for task in group:
                    self.execute_task(task, skip_wait=True)
                all_ids = list(group_ids)
                rospy.loginfo(">>> 等待 {} 架无人机全部悬停...".format(len(all_ids)))
                self.wait_for_hover_and_time(all_ids, 1.0)
            else:
                if i > 0:
                    prev_ids = set(tasks[i-1].get('uav_id', []))
                    rospy.loginfo(">>> 等待前一任务悬停...")
                    self.wait_for_hover_and_time(list(prev_ids), 1.0)
                self.execute_task(tasks[i])
            i = j

        rospy.loginfo(">>> 所有任务序列执行完毕！")


# ====================== 主入口 ======================
def main():
    rospy.init_node('location_allocate')

    test_ros = "当前可用无人机编号: [1,2,3,4,5,6,7,8,9,10]，总数: 10"
    node = UAVFormationNode()

    try:
        while not rospy.is_shutdown():
            user_command = raw_input("\n请输入无人机编队指令: ")

            if user_command.strip().lower() in ["exit", "quit", "q"]:
                break

            if not user_command.strip():
                continue

            rospy.loginfo("正在调用 LLM 解析指令...")
            llm_result = parse_uav_command(user_command, test_ros)

            rospy.loginfo("=" * 50)
            rospy.loginfo("最终解析结果：")
            rospy.loginfo("=" * 50)
            rospy.loginfo(json.dumps(llm_result, indent=2, ensure_ascii=False))

            node.run_mission(llm_result)

            rospy.loginfo("\n任务执行完毕，等待下一条指令...")

    except KeyboardInterrupt:
        rospy.loginfo("收到 Ctrl+C，停止任务")


if __name__ == "__main__":
    main()
