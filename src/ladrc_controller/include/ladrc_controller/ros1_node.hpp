#pragma once
// Small ROS1 resource adapter. Numerical/controller code has no ROS2 runtime dependency.
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <memory>
#include <vector>
#include <set>
#include <chrono>
#include <stdexcept>
namespace ros1_support {
struct Parameter {
  XmlRpc::XmlRpcValue value;
  double as_double() const { auto v=value; return v.getType()==XmlRpc::XmlRpcValue::TypeInt ? double(int(v)):double(v); }
  int as_int() const { auto v=value; return int(v); }
  bool as_bool() const { auto v=value; return bool(v); }
  std::string as_string() const { auto v=value; return std::string(v); }
  std::vector<double> as_double_array() const {auto v=value;std::vector<double> a;for(int i=0;i<v.size();++i) a.push_back(Parameter{v[i]}.as_double());return a;}
  std::vector<int64_t> as_integer_array() const {auto v=value;std::vector<int64_t> a;for(int i=0;i<v.size();++i)a.push_back(int(v[i]));return a;}
};
struct QoS { int depth; bool latch=false; QoS(int n):depth(n){} QoS transient_local(){latch=true;return *this;} };
inline QoS SensorDataQoS(){return QoS(1);}
class Node {
protected:
 ros::NodeHandle nh_, private_nh_{"~"};
 std::set<std::string> overrides_;
 std::string namespace_=ros::this_node::getNamespace();
public:
 explicit Node(const char*) {std::vector<std::string> names;ros::param::getParamNames(names);for(auto &n:names){auto prefix=private_nh_.getNamespace()+"/";if(n.find(prefix)==0)overrides_.insert(n.substr(prefix.size()));}}
 template<class T> void declare_parameter(const std::string& name,const T& value) {if(!private_nh_.hasParam(name))private_nh_.setParam(name,value);}
 void declare_parameter(const std::string& name,const std::vector<int64_t>& value) {std::vector<int> v(value.begin(),value.end());declare_parameter(name,v);}
 template<class T> void declare_parameter(const std::string& name) {if(!private_nh_.hasParam(name))throw std::runtime_error("Missing required parameter: "+name);}
 Parameter get_parameter(const std::string& name) const {XmlRpc::XmlRpcValue v;if(!private_nh_.getParam(name,v))throw std::runtime_error("Missing parameter "+name);return {v};}
 const Node* get_node_parameters_interface()const{return this;}
 const std::set<std::string>& get_parameter_overrides()const{return overrides_;}
 const char* get_namespace()const{return namespace_.c_str();}
 int get_logger()const{return 0;}
 const Node* get_clock()const{return this;}
 ros::Time now()const{return ros::Time::now();}
 template<class M,class F> std::shared_ptr<ros::Subscriber> create_subscription(const std::string& topic,QoS q,F callback){return std::make_shared<ros::Subscriber>(nh_.subscribe<M>(topic,q.depth,boost::function<void(const typename M::ConstPtr&)>(callback),ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay()));}
 template<class M> std::shared_ptr<ros::Publisher> create_publisher(const std::string& topic,QoS q){return std::make_shared<ros::Publisher>(nh_.advertise<M>(topic,q.depth,q.latch));}
 template<class Rep,class Period,class F> std::shared_ptr<ros::Timer> create_wall_timer(std::chrono::duration<Rep,Period> d,F f){return std::make_shared<ros::Timer>(nh_.createTimer(ros::Duration(std::chrono::duration<double>(d).count()),[f](const ros::TimerEvent&){f();}));}
};
}
#define RCLCPP_INFO(logger, ...) ROS_INFO(__VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ROS_WARN(__VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)
#define RCLCPP_INFO_ONCE(logger, ...) ROS_INFO_ONCE(__VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, ms, ...) ROS_WARN_THROTTLE((ms)/1000.0, __VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, ms, ...) ROS_INFO_THROTTLE((ms)/1000.0, __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, ms, ...) ROS_ERROR_THROTTLE((ms)/1000.0, __VA_ARGS__)
