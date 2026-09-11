#pragma once
#include <memory>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QSlider>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rviz_common/panel.hpp>

namespace go2_rviz_controls {
class SpeedPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit SpeedPanel(QWidget * parent = nullptr);
  ~SpeedPanel() override;
  void onInitialize() override;
private:
  void tick();
  void requestApply();
  QDoubleSpinBox * speed_;
  QSlider * slider_;
  QLabel * status_;
  QLabel * navigation_status_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr navigation_log_;
  QTimer * timer_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  double requested_{0.30};
  bool pending_{true};
  bool in_flight_{false};
  int refresh_ticks_{0};
};
}
