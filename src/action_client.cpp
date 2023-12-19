#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tortoisebot_waypoints_interfaces/action/waypoint_action.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using std_srvs::srv::Empty;
using namespace std::placeholders;

using WaypointAction = tortoisebot_waypoints_interfaces::action::WaypointAction;
using GoalHandleWaypointAction =
    rclcpp_action::ClientGoalHandle<WaypointAction>;

/* This example creates a subclass of Node and uses std::bind() to register
 * a member function as a callback from the timer. */

class ActionClient : public rclcpp::Node {
public:
  ActionClient() : Node("action_client") {
    RCLCPP_INFO(this->get_logger(), "Action client creating...");

    sub_odom_ = this->create_subscription<Odometry>(
        "/odom", 3, std::bind(&ActionClient::odom_clb, this, _1));
    pub_twist_ = this->create_publisher<Twist>("/cmd_vel", 5);

    action_client_ =
        rclcpp_action::create_client<WaypointAction>(this, "tortoisebot_as");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&ActionClient::send_goal, this));
    srv_client_ = this->create_client<Empty>("/reset_world");

    // Define action
    action_goal.position.x = 0.3;
    action_goal.position.y = 0.3;
    action_goal.position.z = 1.57;
  }

  void send_goal() {
    this->timer_->cancel();

    while (!this->srv_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Wait for Gazebo worl to start!");
    }

    RCLCPP_INFO(this->get_logger(), "Gazebo started!");
    rclcpp::sleep_for(std::chrono::seconds(3));

    if (!this->action_client_->wait_for_action_server(
            std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");

      rclcpp::shutdown();
    }

    // Send goal
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options =
        rclcpp_action::Client<WaypointAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&ActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&ActionClient::result_callback, this, _1);
    this->action_client_->async_send_goal(action_goal, send_goal_options);
  }

private:
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;
  rclcpp_action::Client<WaypointAction>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<Empty>::SharedPtr srv_client_;

  WaypointAction::Goal action_goal;

  bool action_finished = false;

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
  }

  /* Action client callbacks */
  void goal_response_callback(
      const GoalHandleWaypointAction::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleWaypointAction::SharedPtr,
      const std::shared_ptr<const WaypointAction::Feedback> feedback) {
    (void)feedback;
  }

  void result_callback(const GoalHandleWaypointAction::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    std::string status = result.result->success ? "True" : "False";

    RCLCPP_INFO(this->get_logger(), "Goal finished: %s", status.c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionClient>());
  rclcpp::shutdown();
  return 0;
}