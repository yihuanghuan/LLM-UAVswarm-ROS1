"""ROS1 resources for the frozen middleware-independent mission runtime."""
from __future__ import annotations
import time
from types import SimpleNamespace
import rospy
import yaml

qos_profile_sensor_data = 1

def init(args=None):
    rospy.init_node('location_allocate', disable_signals=True)
    if args:
        for i, value in enumerate(args):
            if value == '-p':
                name, encoded = args[i+1].split(':=', 1)
                rospy.set_param('~' + name, yaml.safe_load(encoded))

def ok():
    return not rospy.is_shutdown()

def spin_once(node, timeout_sec=0.05):
    # rospy dispatches subscribers on their own threads.
    time.sleep(max(0.0, timeout_sec))

def shutdown():
    rospy.signal_shutdown('scheduler complete')

class Parameter:
    def __init__(self, value): self.value = value
    def get_parameter_value(self):
        return SimpleNamespace(string_value=self.value, double_value=self.value,
                               integer_value=self.value, bool_value=self.value)

class Stamp:
    def __init__(self): self.stamp = rospy.Time.now()
    @property
    def nanoseconds(self): return self.stamp.to_nsec()
    def to_msg(self): return self.stamp

class Node:
    def __init__(self, name): self.resources = []
    def declare_parameter(self, name, default):
        if not rospy.has_param('~' + name): rospy.set_param('~' + name, default)
    def get_parameter(self, name): return Parameter(rospy.get_param('~' + name))
    def get_logger(self):
        return SimpleNamespace(info=rospy.loginfo, warn=rospy.logwarn,
                               warning=rospy.logwarn, error=rospy.logerr, debug=rospy.logdebug)
    def get_clock(self): return SimpleNamespace(now=Stamp)
    def create_subscription(self, kind, topic, callback, queue):
        resource = rospy.Subscriber(topic, kind, callback, queue_size=queue, tcp_nodelay=True)
        self.resources.append(resource)
        return resource
    def create_publisher(self, kind, topic, queue):
        resource = rospy.Publisher(topic, kind, queue_size=queue, tcp_nodelay=True)
        resource.get_subscription_count = resource.get_num_connections
        self.resources.append(resource)
        return resource
    def get_subscriptions_info_by_topic(self, topic):
        # Inspect connected TCPROS peers, not just registered subscribers or
        # aggregate counts (a recorder must not satisfy controller readiness).
        peers = []
        for resource in self.resources:
            if isinstance(resource, rospy.Publisher) and resource.resolved_name == topic:
                for connection in tuple(resource.impl.connections):
                    caller = connection.endpoint_id
                    peers.append(SimpleNamespace(node_namespace=caller.rsplit('/', 1)[0] or '/'))
        return peers
    def destroy_node(self):
        for resource in self.resources: resource.unregister()
