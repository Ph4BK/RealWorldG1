#include <rviz_default_plugins/tools/goal_pose/goal_tool.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QIcon>
#include <QPainter>
#include <QPixmap>
#include <QPolygonF>

namespace g1_pose_replay_panel {
// Keep the standard goal input and publication behavior; customize only its visuals.
class RedGoalTool : public rviz_default_plugins::tools::GoalTool {
public:
  void onInitialize() override {
    rviz_default_plugins::tools::GoalTool::onInitialize();
    setName("2D Goal Pose");
    arrow_->setColor(1.f, 0.f, 0.f, 1.f);
    QPixmap pixmap(32, 32);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(230, 0, 0));
    QPolygonF polygon;
    polygon << QPointF(3, 27) << QPointF(18, 12) << QPointF(14, 8)
            << QPointF(30, 2) << QPointF(24, 18) << QPointF(20, 14) << QPointF(5, 29);
    painter.drawPolygon(polygon);
    painter.end();
    setIcon(QIcon(pixmap));
  }
};
}
PLUGINLIB_EXPORT_CLASS(g1_pose_replay_panel::RedGoalTool, rviz_common::Tool)
