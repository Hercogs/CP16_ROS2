#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/msg/string.hpp"
#include "tortoisebot_waypoints_interfaces/action/waypoint_action.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
  ActionServer() : Node("action_server") {
    RCLCPP_INFO(this->get_logger(), "Action server creating...");

    sub_odom_ = this->create_subscription<Odometry>(
        "/odom", 3, std::bind(&ActionServer::odom_clb, this, _1));
    pub_twist_ = this->create_publisher<Twist>("/cmd_vel", 5);

    action_server_ = rclcpp_action::create_server<WaypointAction>(
        this, "tortoisebot_as",
        std::bind(&ActionServer::handle_goal, this, _1, _2),
        std::bind(&ActionServer::handle_cancel, this, _1),
        std::bind(&ActionServer::handle_accepted, this, _1));
    RCLCPP_INFO(this->get_logger(), "Action server created!");
  }

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
  void odom_clb(const Odometry::SharedPtr msg) {
    pose_ = msg->pose.pose.position;

    // auto q = msg->pose.pose.orientation;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_ = yaw;

    // RCLCPP_INFO(this->get_logger(), "I heard x %.2f", pose_.x);
    // RCLCPP_INFO(this->get_logger(), "I heard x %.2f", pose_.y);
    // RCLCPP_INFO(this->get_logger(), "I heard y %.2f", yaw_);

    // twist_msg.linear.x = 0.2;
    // twist_msg.angular.z = 0.2;

    // pub_twist_->publish(twist_msg);
  }

  /* Action server callbacks */
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointAction::Goal> goal) {
    (void)uuid;
    (void)goal;

    RCLCPP_INFO(this->get_logger(),
                "Received request to execute goal: x: %.2f, y: %.2f, yaw: %.2f",
                goal->position.x, goal->position.y, goal->position.z);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void
  handle_accepted(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    std::thread(std::bind(&ActionServer::execute, this, _1), goal_handle)
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    /* Helper variables */
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<WaypointAction::Result>();
    Point target_pos = goal->position;
    float target_yaw = target_pos.z;
    bool success = true;

    float d_x = target_pos.x - this->pose_.x;
    float d_y = target_pos.y - this->pose_.y;
    float desired_heading = std::atan2(d_y, d_x);
    float err_heading = desired_heading - this->yaw_;
    float err_target_yaw = target_yaw - this->yaw_;

    float err_pos = std::sqrt(std::pow(d_x, 2) + std::pow(d_y, 2));
    /* Helper variables: END */
    RCLCPP_INFO(this->get_logger(),
                "Executing goal: x: %.2f, y: %.2f, yaw: %.2f", goal->position.x,
                goal->position.y, goal->position.z);
    while (success && (std::abs(err_pos) > this->dist_precision_ ||
                       std::abs(err_target_yaw) > this->yaw_precision_)) {

      // Update vars
      d_x = target_pos.x - this->pose_.x;
      d_y = target_pos.y - this->pose_.y;
      desired_heading = std::atan2(d_y, d_x);
      err_heading = desired_heading - this->yaw_;
      err_target_yaw = target_yaw - this->yaw_;

      err_pos = std::sqrt(std::pow(d_x, 2) + std::pow(d_y, 2));

      // Logging
      // RCLCPP_INFO(this->get_logger(), "Heading err: %.2f", err_heading);
      // RCLCPP_INFO(this->get_logger(), "Target yaw err: %.2f",
      // err_target_yaw);

      // Skip goal cancle, but TODO
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        // Stop the robot
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        this->pub_twist_->publish(twist_msg);
        return;
      } else if (std::abs(err_heading) > this->yaw_precision_ &&
                 std::abs(err_pos) > this->dist_precision_) {
        // Fix yaw
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = err_heading > 0 ? 0.35 : -0.35;
        this->pub_twist_->publish(twist_msg);
      } else if (std::abs(err_pos) > this->dist_precision_) {
        // Go to point
        twist_msg.linear.x = 0.2;
        twist_msg.angular.z = err_heading > 0.1 ? 0.1 : -0.1;
        this->pub_twist_->publish(twist_msg);
      } else {
        // Turn at final point
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = err_target_yaw > 0 ? 0.35 : -0.35;
        this->pub_twist_->publish(twist_msg);
      }

      // Sleep
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    // Stop the robot
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    this->pub_twist_->publish(twist_msg);

    result->success = success;

    goal_handle->succeed(result);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionServer>());
  rclcpp::shutdown();
  return 0;
}