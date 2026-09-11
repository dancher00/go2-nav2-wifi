#include "go2_rviz_controls/speed_panel.hpp"
#include <cmath>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

namespace go2_rviz_controls {
SpeedPanel::SpeedPanel(QWidget * parent) : rviz_common::Panel(parent) {
  auto * layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel(QString::fromUtf8("Maximum speed")));
  auto * row = new QHBoxLayout();
  speed_ = new QDoubleSpinBox(this);
  speed_->setObjectName("speed_value");
  speed_->setRange(0.10, 0.60);
  speed_->setDecimals(2);
  speed_->setSingleStep(0.05);
  speed_->setValue(0.30);
  speed_->setSuffix(QString::fromUtf8(" m/s"));
  auto * apply = new QPushButton(QString::fromUtf8("Apply"), this);
  apply->setObjectName("apply_speed");
  row->addWidget(speed_); row->addWidget(apply); layout->addLayout(row);
  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setRange(10, 60); slider_->setValue(30); slider_->setSingleStep(5);
  layout->addWidget(slider_);
  status_ = new QLabel(QString::fromUtf8("Waiting for Nav2…"), this);
  status_->setObjectName("speed_status");
  status_->setWordWrap(true); layout->addWidget(status_);
  auto * hint = new QLabel(QString::fromUtf8("Release slider to apply.\nUse 2D Goal Pose to set a destination."), this);
  hint->setWordWrap(true); layout->addWidget(hint);
  navigation_status_ = new QLabel("Navigation: waiting for a goal", this);
  navigation_status_->setObjectName("navigation_status");
  navigation_status_->setWordWrap(true);
  layout->addWidget(navigation_status_); layout->addStretch();
  connect(slider_, &QSlider::valueChanged, this, [this](int value) {
    QSignalBlocker blocked(speed_); speed_->setValue(value / 100.0);
  });
  connect(speed_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
    QSignalBlocker blocked(slider_); slider_->setValue(static_cast<int>(std::round(value * 100)));
  });
  connect(slider_, &QSlider::sliderReleased, this, &SpeedPanel::requestApply);
  connect(speed_, &QDoubleSpinBox::editingFinished, this, &SpeedPanel::requestApply);
  connect(apply, &QPushButton::clicked, this, &SpeedPanel::requestApply);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &SpeedPanel::tick);
}

SpeedPanel::~SpeedPanel() {
  timer_->stop();
  if (node_) executor_.remove_node(node_);
}

void SpeedPanel::onInitialize() {
  node_ = std::make_shared<rclcpp::Node>("go2_rviz_speed_panel");
  executor_.add_node(node_);
  client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/controller_server");
  navigation_log_ = node_->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::QoS(50), [this](rcl_interfaces::msg::Log::ConstSharedPtr msg) {
      if (msg->name != "go2_goal_pose_nav") return;
      // Keep the useful planner error visible instead of replacing it with
      // the generic completion message emitted immediately afterwards.
      if (msg->msg == "leg failed") return;
      navigation_status_->setText("Navigation: " + QString::fromStdString(msg->msg));
    });
  timer_->start(100);
}

void SpeedPanel::requestApply() {
  requested_ = speed_->value();
  pending_ = true;
  status_->setText(QString::fromUtf8("Applying…"));
}

void SpeedPanel::tick() {
  if (!rclcpp::ok()) return;
  executor_.spin_some();
  if (!client_->service_is_ready()) {
    pending_ = true;
    status_->setText(QString::fromUtf8("Waiting for Nav2…"));
    return;
  }
  // Refresh the selected limit after a controller reconfiguration, too.
  if (++refresh_ticks_ >= 20) { refresh_ticks_ = 0; pending_ = true; }
  if (!pending_ || in_flight_) return;
  const double value = requested_;
  pending_ = false; in_flight_ = true;
  try {
    client_->set_parameters_atomically({rclcpp::Parameter("FollowPath.max_vel_x", value),
                                       rclcpp::Parameter("FollowPath.max_speed_xy", value)},
      [this, value](std::shared_future<rcl_interfaces::msg::SetParametersResult> future) {
        in_flight_ = false;
        try {
          const auto result = future.get();
          if (result.successful) {
            status_->setText(QString::fromUtf8("Nav2: limit %1 m/s").arg(value, 0, 'f', 2));
          } else {
            status_->setText(QString::fromUtf8("Not applied: ") + QString::fromStdString(result.reason));
          }
        } catch (const std::exception & error) {
          status_->setText(QString::fromUtf8("Error: ") + QString::fromUtf8(error.what()));
        }
      });
  } catch (const std::exception & error) {
    in_flight_ = false; pending_ = true;
    status_->setText(QString::fromUtf8("Error: ") + QString::fromUtf8(error.what()));
  }
}
}
PLUGINLIB_EXPORT_CLASS(go2_rviz_controls::SpeedPanel, rviz_common::Panel)
