#include "gtest/gtest.h"
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <string>

#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tortoisebot_waypoints_interfaces/action/waypoint_action.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <thread>

#include "tortoisebot_waypoints/action_server_component.hpp"

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using std_srvs::srv::Empty;
using namespace std::placeholders;

using WaypointAction = tortoisebot_waypoints_interfaces::action::WaypointAction;
using GoalHandleWaypointAction =
    rclcpp_action::ClientGoalHandle<WaypointAction>;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class ActionClientTest : public ::testing::Test {
public:
  ActionClientTest() {
    node = rclcpp::Node::make_shared("action_client_test");
    action_server_node = std::make_shared<mylib::ActionServer>();

    RCLCPP_INFO(node->get_logger(), "Action test client creating...");

    sub_odom_ = node->create_subscription<Odometry>(
        "/odom", 3, std::bind(&ActionClientTest::odom_clb, this, _1));
    pub_twist_ = node->create_publisher<Twist>("/cmd_vel", 5);

    action_client_ =
        rclcpp_action::create_client<WaypointAction>(node, "tortoisebot_as");
    // action_client_ = rclcpp_action::create_client<WaypointAction>()

    this->timer_ =
        node->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&ActionClientTest::send_goal, this));
    srv_client_ = node->create_client<Empty>("/reset_world");

    // Define action
    action_goal.position.x = 0.3;
    action_goal.position.y = 0.3;
    action_goal.position.z = 1.57;
  }

  bool test_goal() {
    while (!action_finished) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      rclcpp::spin_some(this->node);
      rclcpp::spin_some(this->action_server_node);
    }

    if (!action_status) {
      return false;
    }
    // Action finished succesfully, not should check position
    float d_x = action_goal.position.x - this->pose_.x;
    float d_y = action_goal.position.y - this->pose_.y;
    float err_pos = std::sqrt(std::pow(d_x, 2) + std::pow(d_y, 2));
    bool linear_goal = err_pos < dist_precision_;
    // Check linear location
    float err_target_yaw = action_goal.position.z - this->yaw_;
    bool yaw_goal = std::abs(err_target_yaw) < yaw_precision_;

    RCLCPP_INFO(node->get_logger(), "Lin err: %d, yaw err: %d", linear_goal,
                yaw_goal);

    return linear_goal && yaw_goal;
  }

protected:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Node> action_server_node;

private:
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;
  rclcpp_action::Client<WaypointAction>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<Empty>::SharedPtr srv_client_;

  WaypointAction::Goal action_goal;

  bool action_finished = false;
  bool action_status = false;

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

  void send_goal() {
    this->timer_->cancel();

    if (!this->srv_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(node->get_logger(),
                  "Gazebo world is not started. Cancel test");
      action_finished = true;
      return;
    }

    RCLCPP_INFO(node->get_logger(), "Gazebo started!");
    rclcpp::sleep_for(std::chrono::seconds(3));

    if (!this->action_client_->wait_for_action_server(
            std::chrono::seconds(3))) {
      RCLCPP_ERROR(node->get_logger(),
                   "Action server not available after waiting");

      action_finished = true;
    }

    // Send goal
    RCLCPP_INFO(node->get_logger(), "Sending goal");
    auto send_goal_options =
        rclcpp_action::Client<WaypointAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ActionClientTest::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&ActionClientTest::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&ActionClientTest::result_callback, this, _1);
    this->action_client_->async_send_goal(action_goal, send_goal_options);
  }

  /* Action client callbacks */
  void goal_response_callback(
      const GoalHandleWaypointAction::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
      action_finished = true;
    } else {
      RCLCPP_INFO(node->get_logger(),
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
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      action_finished = true;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      action_finished = true;
      return;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      action_finished = true;
      return;
    }

    std::string status = result.result->success ? "True" : "False";
    action_status = result.result->success;

    RCLCPP_INFO(node->get_logger(), "Goal finished: %s", status.c_str());
    action_finished = true;
  }
};

TEST_F(ActionClientTest, GoalLocationTest) { EXPECT_TRUE(test_goal()); }

// TEST(MyPublisherTestSuite, MyFirstPublisherTest) { EXPECT_TRUE(true); }