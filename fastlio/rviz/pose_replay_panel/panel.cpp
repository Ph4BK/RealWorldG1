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
#include <QJsonValue>
namespace g1_pose_replay_panel {
class ReplayPanel : public rviz_common::Panel {
 Q_OBJECT
public:
 explicit ReplayPanel(QWidget* parent=nullptr):rviz_common::Panel(parent) {
  auto layout=new QVBoxLayout(this);
  info_=new QLabel(QString::fromUtf8("Waiting for the initial-pose interface..."));info_->setWordWrap(true);layout->addWidget(info_);
  start_=new QPushButton(QString::fromUtf8("Start Localization Replay (1x)"));layout->addWidget(start_);
  auto hint=new QLabel(QString::fromUtf8("Use 2D Pose Estimate to set the startup position and heading.\nDrawing saves the pose. Press the button to start replay."));hint->setWordWrap(true);layout->addWidget(hint);layout->addStretch();
  node_=std::make_shared<rclcpp::Node>("g1_replay_panel",rclcpp::NodeOptions().use_global_arguments(false));
  client_=node_->create_client<std_srvs::srv::Trigger>("/g1/start_replay");
  subscription_=node_->create_subscription<std_msgs::msg::String>("/g1/pose_init_status",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr msg){
   auto o=QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
   QString pose=QString::fromUtf8("Initial pose: not set. Draw an arrow first.");
   if(o["pose"].isObject()) {auto p=o["pose"].toObject();pose=QString("x=%1 m   y=%2 m\nyaw=%3 deg   z=%4 m").arg(p["x"].toDouble(),0,'f',3).arg(p["y"].toDouble(),0,'f',3).arg(p["yaw_deg"].toDouble(),0,'f',1).arg(p["z"].toDouble(),0,'f',3);}
   info_->setText(o["dataset"].toString()+"\n"+pose+"\n"+o["message"].toString());
   running_=o["running"].toBool();start_->setEnabled(!running_&&!pending_);
  });
  connect(start_,&QPushButton::clicked,this,[this](){
   if(!client_->service_is_ready()){QMessageBox::warning(this,QString::fromUtf8("Cannot Start Replay"),QString::fromUtf8("The initial-pose interface is not connected. Please try again shortly."));return;}
   pending_=true;start_->setEnabled(false);
   client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f){
    pending_=false;auto r=f.get();start_->setEnabled(!running_);
    if(!r->success)QMessageBox::warning(this,QString::fromUtf8("Cannot Start Replay"),QString::fromStdString(r->message));
   });
  });
  executor_.add_node(node_);timer_=new QTimer(this);connect(timer_,&QTimer::timeout,this,[this](){executor_.spin_some();});timer_->start(50);
 }
 ~ReplayPanel() override {timer_->stop();executor_.remove_node(node_);}
private:
 QLabel* info_;QPushButton* start_;QTimer* timer_;bool running_=false,pending_=false;
 rclcpp::Node::SharedPtr node_;rclcpp::executors::SingleThreadedExecutor executor_;
 rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
}
PLUGINLIB_EXPORT_CLASS(g1_pose_replay_panel::ReplayPanel,rviz_common::Panel)
#include "panel.moc"
