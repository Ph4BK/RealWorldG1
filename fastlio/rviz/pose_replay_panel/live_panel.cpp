#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <QMessageBox>
#include <QJsonDocument>
#include <QJsonObject>
namespace g1_pose_replay_panel {
class LivePanel : public rviz_common::Panel {
 Q_OBJECT
public:
 explicit LivePanel(QWidget* parent=nullptr):rviz_common::Panel(parent) {
  auto layout=new QVBoxLayout(this);
  info_=new QLabel("Waiting for live BEV interface..."); info_->setWordWrap(true); layout->addWidget(info_);
  goal_=new QLabel("PosNav goal: not set."); goal_->setWordWrap(true); layout->addWidget(goal_);
  start_=new QPushButton("Start Live Localization + BEV"); layout->addWidget(start_);
  stop_=new QPushButton("Stop Localization + Updates"); layout->addWidget(stop_);
  compute_=new QPushButton("Compute FMM + STG macro"); layout->addWidget(compute_);
  plan_=new QLabel("FMM + STG: not calculated"); plan_->setWordWrap(true); layout->addWidget(plan_);
  reset_=new QPushButton("Restore Static Task Map"); layout->addWidget(reset_);
  nav_start_=new QPushButton("Start Navigation"); layout->addWidget(nav_start_);
  nav_stop_=new QPushButton("Stop Navigation"); layout->addWidget(nav_stop_);
  nav_info_=new QLabel("Navigation: idle"); nav_info_->setWordWrap(true); layout->addWidget(nav_info_);
  auto hint=new QLabel("Keep G1 stationary. Draw CURRENT position and heading with 2D Pose Estimate, then start localization.\nDraw a goal with 2D Goal Pose and press Start Navigation to move G1. Redrawing a goal stops navigation.\nMap layers: enable one at a time.");
  hint->setWordWrap(true); layout->addWidget(hint); layout->addStretch();
  node_=std::make_shared<rclcpp::Node>("g1_live_bev_panel",rclcpp::NodeOptions().use_global_arguments(false));
  start_client_=node_->create_client<std_srvs::srv::Trigger>("/g1/live_bev/start");
  stop_client_=node_->create_client<std_srvs::srv::Trigger>("/g1/live_bev/stop");
  compute_client_=node_->create_client<std_srvs::srv::Trigger>("/g1/live_bev/compute_stg");
  reset_client_=node_->create_client<std_srvs::srv::Trigger>("/g1/live_bev/reset");
  nav_start_client_=node_->create_client<std_srvs::srv::Trigger>("/g1/navigation/start");
  nav_stop_client_=node_->create_client<std_srvs::srv::Trigger>("/g1/navigation/stop");
  nav_sub_=node_->create_subscription<std_msgs::msg::String>("/g1/navigation/status",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr msg){
   auto o=QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
   navigating_=o["active"].toBool();
   nav_info_->setText("Navigation: "+o["state"].toString()+"\n"+o["message"].toString()); buttons();
  });
  sub_=node_->create_subscription<std_msgs::msg::String>("/g1/live_bev/status",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr msg){
   auto o=QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
   QString pose="Initial pose: draw an arrow.";
   if(o["pose"].isObject()) {auto p=o["pose"].toObject(); pose=QString("x=%1 m  y=%2 m\nyaw=%3 deg  IMU z=%4 m").arg(p["x"].toDouble(),0,'f',3).arg(p["y"].toDouble(),0,'f',3).arg(p["yaw_deg"].toDouble(),0,'f',1).arg(p["z"].toDouble(),0,'f',3);}
   info_->setText(pose+"\n"+QString("Window: %1/%2  Updates: %3\n").arg(o["frames"].toInt()).arg(o["window_frames"].toInt(5)).arg(o["updates"].toInt())+o["message"].toString());
   running_=o["running"].toBool(); has_goal_=o["goal"].isObject(); buttons();
   plan_->setText(o["plan_message"].toString("FMM + STG: not calculated"));
   if(o["goal"].isObject()) {auto g=o["goal"].toObject(); goal_->setText(QString("PosNav goal: x=%1 m, y=%2 m, yaw=%3 deg").arg(g["x"].toDouble(),0,'f',3).arg(g["y"].toDouble(),0,'f',3).arg(g["yaw"].toDouble()*180./3.141592653589793,0,'f',1));}
   else {goal_->setText("PosNav goal: not set.");}
  });
  connect(start_,&QPushButton::clicked,this,[this](){call(start_client_);});
  connect(stop_,&QPushButton::clicked,this,[this](){call(stop_client_);});
  connect(compute_,&QPushButton::clicked,this,[this](){call(compute_client_);});
  connect(reset_,&QPushButton::clicked,this,[this](){call(reset_client_);});
  connect(nav_start_,&QPushButton::clicked,this,[this](){call(nav_start_client_);});
  connect(nav_stop_,&QPushButton::clicked,this,[this](){call(nav_stop_client_);});
  executor_.add_node(node_); timer_=new QTimer(this);
  connect(timer_,&QTimer::timeout,this,[this](){executor_.spin_some();}); timer_->start(50); buttons();
 }
 ~LivePanel() override {timer_->stop(); executor_.remove_node(node_);}
private:
 using Client=rclcpp::Client<std_srvs::srv::Trigger>;
 void buttons(){start_->setEnabled(!running_&&!pending_);stop_->setEnabled(running_&&!pending_);reset_->setEnabled(!pending_);compute_->setEnabled(has_goal_&&!pending_&&!navigating_);nav_start_->setEnabled(running_&&has_goal_&&!pending_&&!navigating_);nav_stop_->setEnabled(navigating_);}
 void call(Client::SharedPtr client){
  if(!client->service_is_ready()){QMessageBox::warning(this,"Live BEV","Interface is not connected.");return;}
  pending_=true;buttons();
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this](Client::SharedFuture future){
   pending_=false; buttons(); auto result=future.get();
   if(!result->success) {
    auto box=new QMessageBox(QMessageBox::Warning,"Live BEV",QString::fromStdString(result->message),QMessageBox::Ok,this);
    box->setAttribute(Qt::WA_DeleteOnClose); box->open();
   }
  });
 }
 QLabel *info_,*goal_,*plan_,*nav_info_; QPushButton *start_,*stop_,*reset_,*compute_,*nav_start_,*nav_stop_; QTimer* timer_; bool running_=false,pending_=false,has_goal_=false,navigating_=false;
 rclcpp::Node::SharedPtr node_; rclcpp::executors::SingleThreadedExecutor executor_;
 Client::SharedPtr start_client_,stop_client_,reset_client_,compute_client_,nav_start_client_,nav_stop_client_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_,nav_sub_;
};
}
PLUGINLIB_EXPORT_CLASS(g1_pose_replay_panel::LivePanel,rviz_common::Panel)
#include "live_panel.moc"
