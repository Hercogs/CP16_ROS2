#ifndef ACTION_SERVER_COMPONENT
#define ACTION_SERVER_COMPONENT

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/msg/string.hpp"
#include "tortoisebot_waypoints_interfaces/action/waypoint_action.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace mylib {

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using namespace std::placeholders;

using WaypointAction = tortoisebot_waypoints_interfaces::action::WaypointAction;
using GoalHandleWaypointAction =
    rclcpp_action::ServerGoalHandle<WaypointAction>;

/* This example creates a subclass of Node and uses std::bind() to register
 * a member function as a callback from the timer. */

class ActionServer : public rclcpp::Node {
public:
  explicit ActionServer();

private:
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;
  rclcpp_action::Server<WaypointAction>::SharedPtr action_server_;

  Point pose_;
  float yaw_ = 0.0;

  float yaw_precision_ = M_PI / 30;
  float dist_precision_ = 0.07;

  Twist twist_msg = Twist();

  /* Odom callback*/
  void odom_clb(const Odometry::SharedPtr msg);

  /* Action server callbacks */
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointAction::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);

  void
  handle_accepted(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);

  void execute(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);
};
} // namespace mylib

#endif