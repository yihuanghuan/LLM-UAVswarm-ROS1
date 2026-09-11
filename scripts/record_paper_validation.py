#!/usr/bin/env python3
"""Record real ROS1 flight evidence; fail on missing streams or controller faults."""
import argparse,csv,json,math,time,threading
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from uav_swarm_interfaces.msg import UAVStatus,UAVExecutionCommand,IAPFDebug

def main():
    p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=90);p.add_argument('--output',default='/ros1_ws/validation/paper_ros1');p.add_argument('--ids',default='1,2,3');a=p.parse_args()
    ids=[int(x) for x in a.ids.split(',')];out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    rospy.init_node('paper_validation_recorder');lock=threading.Lock();states={};status={};counts={u:0 for u in ids};masks={u:set() for u in ids};active={u:0 for u in ids};commands=[];faults=[];min_distance=[float('inf')]
    f=(out/'telemetry.csv').open('w');writer=csv.writer(f);writer.writerow(['time','uav','x','y','z','vx','vy','vz','ready','hover','position_error','mission_id'])
    def odom(msg,uid):
        with lock:
            pos=msg.pose.pose.position;vel=msg.twist.twist.linear;states[uid]=(pos.x,pos.y,pos.z);counts[uid]+=1;st=status.get(uid)
            writer.writerow([msg.header.stamp.to_sec(),uid,pos.x,pos.y,pos.z,vel.x,vel.y,vel.z,st.system_ready if st else False,st.is_hover_stable if st else False,st.position_error if st else None,st.mission_id if st else None])
            for other,xyz in states.items():
                if other!=uid:min_distance[0]=min(min_distance[0],math.sqrt(sum((a-b)**2 for a,b in zip(states[uid],xyz))))
    def st(msg,uid):
        with lock:
            status[uid]=msg
            if msg.startup_state==6 or msg.failsafe:faults.append({'uav':uid,'time':time.time(),'startup':msg.startup_state})
    def cmd(msg):
        with lock:commands.append({'uav':msg.uav_id,'mission':msg.mission_id,'task':msg.task_id,'target':[msg.target_pos.x,msg.target_pos.y,msg.target_pos.z],'duration':msg.profile.duration,'style':msg.profile.style,'configuration_id':msg.profile.configuration_id})
    def raw(msg,uid):
        with lock:masks[uid].add(msg.type_mask)
    def iapf(msg,uid):
        with lock:active[uid]+=int(msg.iapf_active)
    subs=[]
    for uid in ids:
        for topic,kind,fn in [('swarm_state',Odometry,odom),('status',UAVStatus,st),('mavros/setpoint_raw/local',PositionTarget,raw),('iapf_debug',IAPFDebug,iapf)]:
            subs.append(rospy.Subscriber('/uav%d/%s'%(uid,topic),kind,lambda msg,u=uid,fn=fn:fn(msg,u),queue_size=100,tcp_nodelay=True))
        subs.append(rospy.Subscriber('/uav%d/execution_command'%uid,UAVExecutionCommand,cmd))
    time.sleep(a.duration)
    for sub in subs:sub.unregister()
    with lock:
        f.close();report={'samples':counts,'setpoint_masks':{u:sorted(v) for u,v in masks.items()},'iapf_active_samples':active,'minimum_pair_distance_m':min_distance[0],'commands':commands,'fault_count':len(faults),'final':{u:{'ready':s.system_ready,'stable':s.is_hover_stable,'position_error':s.position_error,'speed':s.speed,'armed':s.armed,'offboard':s.offboard,'altitude':s.altitude} for u,s in status.items()}}
    (out/'flight_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    if faults or not all(counts.values()) or len(status)!=len(ids):raise SystemExit(2)
if __name__=='__main__':main()
