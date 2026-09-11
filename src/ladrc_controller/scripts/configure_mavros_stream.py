#!/usr/bin/env python3
"""Request the transport rate needed by the unchanged C0-B freshness policy."""
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval

def main():
    rospy.init_node('configure_mavros_stream')
    connected=[False];configured=[False]
    def state(msg):
        if not msg.connected:configured[0]=False
        connected[0]=msg.connected
    rospy.Subscriber('mavros/state',State,state,queue_size=1)
    service=rospy.ServiceProxy('mavros/set_message_interval',MessageInterval)
    rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        if connected[0] and not configured[0]:
            try:
                configured[0]=service(message_id=32,message_rate=100.0).success
                if configured[0]:rospy.loginfo('LOCAL_POSITION_NED requested at 100 Hz for frozen state freshness')
            except rospy.ServiceException as exc:rospy.logwarn_throttle(5,str(exc))
        rate.sleep()
if __name__=='__main__':main()
