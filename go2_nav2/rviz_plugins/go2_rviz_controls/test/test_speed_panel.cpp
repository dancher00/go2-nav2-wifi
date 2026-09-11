#include "go2_rviz_controls/speed_panel.hpp"
#include <QApplication>
#include <QPushButton>
#include <QTest>
#include <iostream>
#include <stdexcept>

int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto server = std::make_shared<rclcpp::Node>("controller_server");
    server->declare_parameter("FollowPath.max_vel_x", 0.3);
    server->declare_parameter("FollowPath.max_speed_xy", 0.3);
    auto reject = server->add_on_set_parameters_callback([](const auto & params) {
      rcl_interfaces::msg::SetParametersResult r; r.successful = true;
      for (const auto & p : params) if (p.as_double() > 0.55) {
        r.successful = false; r.reason = "test rejection";
      }
      return r;
    });
    rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(server);
    go2_rviz_controls::SpeedPanel panel; panel.onInitialize();
    auto * spin = panel.findChild<QDoubleSpinBox *>("speed_value");
    auto * apply = panel.findChild<QPushButton *>("apply_speed");
    auto * status = panel.findChild<QLabel *>("speed_status");
    auto * navigation = panel.findChild<QLabel *>("navigation_status");
    auto logs = server->create_publisher<rcl_interfaces::msg::Log>("/rosout", 10);
    const auto until = [&](auto condition) {
      for (int i = 0; i < 500; ++i) {
        executor.spin_some(); app.processEvents(); QTest::qWait(10);
        if (condition()) return;
      }
      throw std::runtime_error("Panel did not confirm the parameter response");
    };
    until([&] { return status->text().contains("0.30"); });
    rcl_interfaces::msg::Log log;
    log.name = "go2_goal_pose_nav"; log.msg = "No valid path";
    until([&] { logs->publish(log); return navigation->text().contains("No valid path"); });
    spin->setValue(0.45); apply->click();
    until([&] { return status->text().contains("0.45"); });
    for (auto name : {"FollowPath.max_vel_x", "FollowPath.max_speed_xy"}) {
      if (server->get_parameter(name).as_double() != 0.45) throw std::runtime_error("Wrong applied limit");
    }
    spin->setValue(1.0);
    if (spin->value() != 0.60) throw std::runtime_error("UI range not enforced");
    apply->click();
    until([&] { return status->text().contains("test rejection"); });
    if (server->get_parameter("FollowPath.max_vel_x").as_double() != 0.45) {
      throw std::runtime_error("Rejected speed changed controller");
    }
    std::cout << "Speed panel: atomic update, UI cap, acknowledgement and rejection passed\n";
  } catch (const std::exception & e) { std::cerr << e.what() << '\n'; result = 1; }
  rclcpp::shutdown(); return result;
}
