#!/usr/bin/env python3
"""Gazebo-only execution-layer stress probe: exchange two UAV destinations.

Intentionally bypasses the planning allocator to exercise runtime IAPF. Profiles
are still produced by the frozen central compiler; this is not a paper trial.
"""
import json,time,math
from pathlib import Path
from types import SimpleNamespace
import rospy
from nav_msgs.msg import Odometry
from uav_swarm_interfaces.msg import UAVExecutionCommand,UAVStatus,IAPFDebug
from location_allocate.policy_adapter import load_runtime_policy
from location_allocate.lfs_types import ExecutableLFS
from location_allocate.execution_profile_compiler import compile_execution_profiles
from location_allocate.execution_command_builder import build_task_command_batch

def main():
    rospy.init_node('iapf_crossing_probe');ids=(1,2);states={};statuses={};active=[0];minimum=[float('inf')];seen={u:False for u in ids}
    def state(m,u):
        p=m.pose.pose.position;states[u]=(p.x,p.y,p.z)
        if len(states)==2:minimum[0]=min(minimum[0],math.dist(states[1],states[2]))
    def status(m,u):
        statuses[u]=m
        if m.mission_id==100 and not m.is_hover_stable:seen[u]=True
    def debug(m):active[0]+=int(m.iapf_active)
    for u in ids:
        rospy.Subscriber('/uav%d/swarm_state'%u,Odometry,lambda m,u=u:state(m,u),queue_size=1)
        rospy.Subscriber('/uav%d/status'%u,UAVStatus,lambda m,u=u:status(m,u),queue_size=1)
        rospy.Subscriber('/uav%d/iapf_debug'%u,IAPFDebug,debug,queue_size=100)
    pubs={u:rospy.Publisher('/uav%d/execution_command'%u,UAVExecutionCommand,queue_size=1) for u in ids}
    deadline=time.monotonic()+15
    while not (len(states)==2 and all(p.get_num_connections() for p in pubs.values())):
        if time.monotonic()>deadline:raise RuntimeError('missing flight streams')
        time.sleep(.02)
    initial=[states[u] for u in ids];targets=list(reversed(initial))
    executable=ExecutableLFS(ids,{'type':'Line'},tuple(sum(x)/2 for x in zip(*initial)),3.,12.,'normal',1.,{'mode':'direct'})
    config,policy=load_runtime_policy('/ros1_ws/src/lfs_policy/config/lfs_policy.paper_current.yaml')
    safety=policy.resolve_safety(1.)
    profiles=compile_execution_profiles(executable,initial,targets,policy.profile,safety.soft_iapf)
    resolved=SimpleNamespace(executable_lfs=executable,assigned_targets=targets,profiles=profiles)
    for command in build_task_command_batch(resolved,100,100,stamp=rospy.Time.now()):pubs[command.uav_id].publish(command)
    deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        if all(seen[u] and statuses[u].is_hover_stable and statuses[u].mission_id==100 for u in ids):break
        if any(s.failsafe or s.startup_state==6 for s in statuses.values()):raise RuntimeError('controller fault')
        time.sleep(.02)
    success=all(seen[u] and statuses[u].is_hover_stable and statuses[u].mission_id==100 for u in ids)
    report={'completed':success,'minimum_pair_distance_m':minimum[0],'iapf_active_samples':active[0],'final_errors':{u:statuses[u].position_error for u in ids},'targets':targets}
    Path('/ros1_ws/validation/paper_ros1/iapf_crossing.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    if not success or not active[0] or minimum[0]<config.safety['d_hard']:raise SystemExit(2)
if __name__=='__main__':main()
