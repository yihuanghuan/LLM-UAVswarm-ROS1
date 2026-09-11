#!/usr/bin/env python3
"""Wait for fresh, healthy startup completion from every simulated UAV."""
import argparse
import time
import rospy
from uav_swarm_interfaces.msg import UAVStatus


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ids', default='1,2,3,4,5')
    parser.add_argument('--timeout', type=float, default=90.0)
    args = parser.parse_args()
    ids = [int(value) for value in args.ids.split(',')]
    if not ids or len(set(ids)) != len(ids) or args.timeout <= 0:
        parser.error('IDs must be unique and timeout must be positive')
    rospy.init_node('wait_for_swarm_ready', anonymous=True)
    states = {}
    def receive(message, uid):
        states[uid] = (message, time.monotonic())
    subscribers = [rospy.Subscriber(
        f'/uav{uid}/status', UAVStatus,
        lambda message, uid=uid: receive(message, uid), queue_size=1)
        for uid in ids]
    deadline = time.monotonic() + args.timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        now = time.monotonic()
        samples = dict(states)
        if all(uid in samples and now - samples[uid][1] < 2.0
               and samples[uid][0].system_ready and samples[uid][0].armed
               and samples[uid][0].offboard and not samples[uid][0].failsafe
               for uid in ids):
            print(f'READY: {ids}')
            return
        time.sleep(0.1)
    missing = [uid for uid in ids if uid not in states or not states[uid][0].system_ready]
    raise SystemExit(f'Startup readiness timed out; inspect /uavN/status. Not ready: {missing}')


if __name__ == '__main__':
    main()
