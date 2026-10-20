// PC2 motion entry: official ROS2 client, FSM 501, plus read-only odometry.
#include <g1/g1_loco_client.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <future>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
volatile std::sig_atomic_t cancelled=0;
void cancel(int) { cancelled=1; }
struct Sample {
  bool valid=false;
  int64_t stamp=-1;
  double x=0,y=0,yaw=0,speed=0,rate=0;
  Clock::time_point received;
  nlohmann::json odom;
};
int main(int argc,char** argv) {
  nlohmann::json report;
  double vx=0,vy=0,w=0,duration=0;
  const bool calibrate=argc>1 && std::string(argv[argc-1])=="--calibrate";
  if(calibrate) --argc;
  const bool odom_only=argc==2 && std::string(argv[1])=="--odom";
  try {
    if(calibrate && odom_only) throw std::runtime_error("--calibrate requires a motion action");
    if(!odom_only) {
    if(argc!=4 && argc!=5) throw std::runtime_error("usage: g1_motion vx [vy] yaw_rate duration (executes immediately)");
    auto number=[](const char* s) { size_t end=0; double v=std::stod(s,&end); if(end!=std::string(s).size() || !std::isfinite(v)) throw std::runtime_error("invalid number"); return v; };
    vx=number(argv[1]);
    if(argc==5) vy=number(argv[2]);
    w=number(argv[argc-2]); duration=number(argv[argc-1]);
    if(duration<=0 || ((vx!=0 || vy!=0) && w!=0))
      throw std::runtime_error("duration must be positive; translation and rotation must be separate");
    if(calibrate && (vy!=0 || vx<0 || (vx==0 && w==0)))
      throw std::runtime_error("calibration requires forward, left turn or right turn");
    }
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 2; }
  report={{"fsm_required",501},{"vx",vx},{"vy",vy},{"yaw_rate",w},{"duration",duration},
          {"motion_ack",false},{"stop_ack",false}};
  report["zero_velocity_sent"]=false;
  const bool auto_expiry=(vx!=0 || vy!=0 || w!=0);
  report["stop_policy"]=auto_expiry ? "duration_expiry_with_error_fallback" : "explicit_zero";
  // Direct invocations in one container serialize here; runner also locks host.
  int lock_fd=open("/tmp/g1_motion_session.lock",O_CREAT|O_RDWR,0600);
  if(lock_fd<0 || flock(lock_fd,LOCK_EX|LOCK_NB)!=0) { std::cerr<<"motion session busy/unavailable\n"; if(lock_fd>=0) close(lock_fd); return 2; }
  rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,cancel); std::signal(SIGTERM,cancel);
  auto node=std::make_shared<rclcpp::Node>("unigoal_g1_motion");
  std::mutex mutex;
  Sample latest;
  std::deque<nlohmann::json> poses;
  int64_t pose_stamp=-1;
  Clock::time_point pose_received;
  std::string pose_error;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
  if(calibrate) {
    pose_sub=node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/g1/live_bev/robot",rclcpp::SensorDataQoS(),
      [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        auto received=Clock::now(); auto ros_ns=node->get_clock()->now().nanoseconds();
        int64_t stamp=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;
        auto p=m->pose.position; auto q=m->pose.orientation;
        double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
        std::lock_guard<std::mutex> lock(mutex);
        if(m->header.frame_id!="map" || !std::isfinite(p.x) || !std::isfinite(p.y) ||
           !std::isfinite(norm) || norm<1e-6) { pose_error="invalid PosNav map pose"; return; }
        if(stamp<pose_stamp) { pose_error="PosNav timestamp moved backward"; return; }
        if(stamp==pose_stamp) return;
        q.x/=norm; q.y/=norm; q.z/=norm; q.w/=norm;
        double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        poses.push_back({{"stamp_ns",stamp},{"received_ros_ns",ros_ns},
          {"received_steady_ns",std::chrono::duration_cast<std::chrono::nanoseconds>(received.time_since_epoch()).count()},
          {"frame_id","map"},{"x",p.x},{"y",p.y},{"yaw_rad",yaw}});
        pose_stamp=stamp; pose_received=received;
        // Only the final window is needed, even for a long commanded action.
        while(poses.size()>2048 || (!poses.empty() && ros_ns-poses.front()["received_ros_ns"].get<int64_t>()>5000000000LL))
          poses.pop_front();
      });
  }
  auto pose_ok=[&] {
    std::lock_guard<std::mutex> lock(mutex);
    auto age=node->get_clock()->now().nanoseconds()-pose_stamp;
    return pose_error.empty() && pose_stamp>0 && Clock::now()-pose_received<=500ms && age>=0 && age<=500000000LL;
  };
  auto sub=node->create_subscription<nav_msgs::msg::Odometry>("/dog_odom",rclcpp::SensorDataQoS(),
    [&](nav_msgs::msg::Odometry::ConstSharedPtr m) {
      auto p=m->pose.pose.position; auto q=m->pose.pose.orientation; auto v=m->twist.twist;
      Sample s; s.stamp=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;
      s.x=p.x; s.y=p.y; s.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      s.speed=std::hypot(v.linear.x,v.linear.y); s.rate=v.angular.z; s.received=Clock::now();
      s.valid=std::isfinite(s.x)&&std::isfinite(s.y)&&std::isfinite(s.yaw)&&std::isfinite(s.speed)&&std::isfinite(s.rate);
      s.odom={{"stamp_ns",s.stamp},{"frame_id",m->header.frame_id},{"child_frame_id",m->child_frame_id},
        {"x",p.x},{"y",p.y},{"z",p.z},{"yaw_rad",s.yaw},
        {"orientation",{{"x",q.x},{"y",q.y},{"z",q.z},{"w",q.w}}},
        {"linear_velocity",{{"x",v.linear.x},{"y",v.linear.y},{"z",v.linear.z}}},
        {"angular_velocity",{{"x",v.angular.x},{"y",v.angular.y},{"z",v.angular.z}}},
        {"stationary_by_twist",s.speed<=.03 && std::abs(s.rate)<=.08}};
      std::lock_guard<std::mutex> lock(mutex); if(s.stamp>latest.stamp) latest=s;
    });
  auto sample=[&] { std::lock_guard<std::mutex> lock(mutex); return latest; };
  auto fresh=[](const Sample& s) { return s.valid && Clock::now()-s.received<=250ms; };
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); std::thread spin([&] { executor.spin(); });
  if(odom_only) {
    report={{"mode","odom"},{"odom_received",false}};
    auto deadline=Clock::now()+5s;
    while(!cancelled && Clock::now()<deadline && !fresh(sample())) std::this_thread::sleep_for(20ms);
    auto current=sample();
    bool ok=!cancelled && fresh(current);
    if(ok) { report["odom_received"]=true; report["odom"]=current.odom; }
    else report["error"]="cancelled or fresh odometry unavailable";
    executor.cancel(); spin.join(); rclcpp::shutdown(); close(lock_fd);
    std::cout<<"RESULT "<<report.dump()<<std::endl;
    return ok ? 0 : 1;
  }
  // Read-only mode returns before constructing any sport client or sending RPCs.
  unitree::robot::g1::LocoClient client(node.get()),stop_client(node.get());
  auto response_observer=node->create_subscription<unitree_api::msg::Response>(
    "/api/sport/response",rclcpp::QoS(100),[](unitree_api::msg::Response::ConstSharedPtr) {});
  bool attempted=false;
  std::future<int> motion;
  Sample first;
  auto began=Clock::now();
  try {
    auto deadline=Clock::now()+5s;
    while(!cancelled && Clock::now()<deadline &&
          (node->count_subscribers("/api/sport/request")==0 || node->count_publishers("/api/sport/response")==0 || !fresh(sample()) || (calibrate && !pose_ok())))
      std::this_thread::sleep_for(20ms);
    if(cancelled || node->count_subscribers("/api/sport/request")==0 || node->count_publishers("/api/sport/response")==0)
      throw std::runtime_error("cancelled or sport endpoints unavailable");
    std::this_thread::sleep_for(500ms);
    int fsm=-1; int code=client.GetFsmId(fsm);
    report["fsm"]=fsm; report["fsm_code"]=code;
    if(code!=0 || fsm!=501) throw std::runtime_error("FSM 501 query required; no automatic mode switch");
    first=sample();
    if(cancelled || !fresh(first) || first.speed>.03 || std::abs(first.rate)>.08)
      throw std::runtime_error("preflight: cancelled, stale odometry or robot not stationary");
    if(calibrate && !pose_ok()) throw std::runtime_error("preflight: fresh PosNav map pose required (source/receive age <= 0.5 s)");
    // Time zero is the local SDK invocation, not an unobservable firmware start.
    int64_t command_ros_ns=0;
    std::promise<Clock::time_point> start_promise;
    auto started=start_promise.get_future();
    attempted=true;
    motion=std::async(std::launch::async,[&] {
      auto start=Clock::now(); command_ros_ns=node->get_clock()->now().nanoseconds();
      start_promise.set_value(start);
      return client.SetVelocity(vx,vy,w,duration);
    });
    began=started.get();
    const int64_t expiry_ros_ns=command_ros_ns+int64_t(double(float(duration))*1e9);
    if(calibrate) report["calibration"]={{"action",vx>0 ? "forward" : (w>0 ? "left_turn" : "right_turn")},
      {"pose_topic","/g1/live_bev/robot"},{"command_ros_ns",command_ros_ns},{"expiry_ros_ns",expiry_ros_ns},
      {"time_basis","local SDK invocation + float32 command duration; firmware expiry is not observable"}};
    while(Clock::now()-began<std::chrono::duration<double>(duration)) {
      auto s=sample();
      if(cancelled || !fresh(s)) throw std::runtime_error("cancelled or stale odometry");
      if(calibrate && !pose_ok()) throw std::runtime_error("PosNav pose invalid/stale during motion");
      if(motion.valid() && motion.wait_for(0ms)==std::future_status::ready) {
        int code=motion.get(); report["motion_code"]=code; report["motion_ack"]=(code==0);
        if(code!=0) throw std::runtime_error("motion request failed");
      }
      std::this_thread::sleep_for(10ms);
    }
  } catch(const std::exception& e) { report["error"]=e.what(); }
  if(attempted) {
    report["command_elapsed_s"]=std::chrono::duration<double>(Clock::now()-began).count();
    if(auto_expiry && !report.contains("error")) {
      try {
      if(motion.valid() && motion.wait_for(0ms)==std::future_status::ready) {
        int code=motion.get(); report["motion_code"]=code; report["motion_ack"]=(code==0);
      }
      if(report["motion_ack"]!=true) report["error"]="motion ACK unavailable at expiry";
      } catch(const std::exception& e) { report["error"]=e.what(); }
      if(cancelled || !fresh(sample())) report["error"]="cancelled/stale at expiry";
    }
    if(calibrate && !report.contains("error")) {
      try {
        const int64_t expiry_ns=report["calibration"]["expiry_ros_ns"];
        auto deadline=began+std::chrono::duration<double>(duration+3.0);
        while(true) {
          if(cancelled || !fresh(sample()) || !pose_ok()) throw std::runtime_error("cancelled or stale feedback during calibration tail");
          auto elapsed=std::chrono::duration<double>(Clock::now()-began).count();
          auto ros_elapsed=(node->get_clock()->now().nanoseconds()-report["calibration"]["command_ros_ns"].get<int64_t>())/1e9;
          if(std::abs(elapsed-ros_elapsed)>.05) throw std::runtime_error("ROS clock changed during calibration");
          bool complete;
          { std::lock_guard<std::mutex> lock(mutex); complete=pose_stamp>=expiry_ns+2000000000LL; }
          if(complete) break;
          if(Clock::now()>deadline) throw std::runtime_error("PosNav did not cover expiry + 2 s");
          std::this_thread::sleep_for(5ms);
        }
        std::lock_guard<std::mutex> lock(mutex);
        report["calibration"]["poses"]=nlohmann::json::array();
        for(const auto& pose:poses)
          if(pose["stamp_ns"].get<int64_t>()>=expiry_ns-500000000LL)
            report["calibration"]["poses"].push_back(pose);
      } catch(const std::exception& e) { report["error"]=e.what(); }
    }
    // Independent client: never wait for motion ACK before attempting stop.
    if(!auto_expiry || report.contains("error")) {
      report["zero_velocity_sent"]=true;
      try { int code=stop_client.SetVelocity(0,0,0,.3F); report["stop_code"]=code; report["stop_ack"]=(code==0); }
      catch(const std::exception& e) { report["stop_error"]=e.what(); }
    }
    if(motion.valid()) {
      try { int code=motion.get(); report["motion_code"]=code; report["motion_ack"]=(code==0); }
      catch(const std::exception& e) { report["motion_error"]=e.what(); }
    }
  }
  // Completion means RPC sequence completed, NOT physical settling/calibration.
  bool ok=!report.contains("error") && report["motion_ack"]==true &&
          (auto_expiry || report["stop_ack"]==true);
  report["command_completed"]=ok;
  executor.cancel(); spin.join(); rclcpp::shutdown(); close(lock_fd);
  std::cout<<"RESULT "<<report.dump()<<std::endl;
  return ok ? 0 : 1;
}
