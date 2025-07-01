// multi_map_action_server.cpp

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include "multi_map_nav/pose_saver_db.hpp" 
#include "multi_map_nav/wormhole_db.hpp"  
#include "custom_msg/action/navigate_to_named_pose.hpp"  // Custom action

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateNamed = custom_msg::action::NavigateToNamedPose;

class MultiMapActionServer : public rclcpp::Node {
public:
  MultiMapActionServer()
  : Node("multi_map_action_server"),
    pose_db_("wormholes.db"),
    wormhole_db_("wormholes.db") {

    this->declare_parameter("current_map", "room1");
    this->get_parameter("current_map", current_map_);

    initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);

    navigate_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    action_server_ = rclcpp_action::create_server<NavigateNamed>(
      this,
      "navigate_to_named_pose",
      std::bind(&MultiMapActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MultiMapActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MultiMapActionServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  std::string current_map_;
  std::string previous_map_;
  PoseSaverDB pose_db_;
  WormholeDB wormhole_db_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
  rclcpp_action::Server<NavigateNamed>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateNamed::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal: pose_name=%s, target_map=%s",
                goal->pose_name.c_str(), goal->map_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateNamed>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateNamed>> goal_handle) {
    std::thread{std::bind(&MultiMapActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateNamed>> goal_handle) {
    const auto goal = goal_handle->get_goal();
    std::string target_map = goal->map_name;
    std::string pose_name = goal->pose_name;

    if (target_map != current_map_) {
      double wx, wy, wtheta;
      wormhole_db_.getWormhole(current_map_, target_map, wx, wy, wtheta);
      navigate_to(wx, wy, wtheta);

      auto load_map_client = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
      auto map_request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
      map_request->map_url = "/home/mendacium/" + target_map + ".yaml";
      load_map_client->wait_for_service();
      auto map_future = load_map_client->async_send_request(map_request);
      auto response = map_future.get();
      if (response->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Successfully loaded map: %s", target_map.c_str());
        previous_map_ = current_map_;
        current_map_ = target_map;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", target_map.c_str());
        auto result = std::make_shared<NavigateNamed::Result>();
        result->success = false;
        result->message = "Failed to load map";
        goal_handle->abort(result);
        return;
      }

      double tx, ty, ttheta;
      wormhole_db_.getWormhole(current_map_, previous_map_, tx, ty, ttheta);
      publish_initialpose(tx, ty, normalize_angle(ttheta + M_PI));

      spin_robot();
    }

    double gx, gy, gtheta;
    pose_db_.getSavedPose(current_map_, pose_name, gx, gy, gtheta);
    navigate_to(gx, gy, gtheta);

    auto result = std::make_shared<NavigateNamed::Result>();
    result->success = true;
    result->message = "Goal reached";
    goal_handle->succeed(result);
  }

  void publish_initialpose(double x, double y, double theta) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.orientation.z = sin(theta / 2.0);
    pose.pose.pose.orientation.w = cos(theta / 2.0);
    initialpose_pub_->publish(pose);
  }

  void spin_robot() {
    RCLCPP_INFO(this->get_logger(), "Spinning robot in place");
    auto cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    rclcpp::Rate rate(20);
    geometry_msgs::msg::Twist twist_msg;
    const double angular_speed = 0.5;
    const double total_rotation = 2 * 2 * M_PI;
    double rotated = 0.0;
    rclcpp::Time start_time = this->now();

    while (rclcpp::ok() && rotated < total_rotation) {
      twist_msg.angular.z = angular_speed;
      cmd_vel_pub->publish(twist_msg);
      rclcpp::Duration elapsed = this->now() - start_time;
      rotated = angular_speed * elapsed.seconds();
      rate.sleep();
    }

    twist_msg.angular.z = 0.0;
    cmd_vel_pub->publish(twist_msg);
    RCLCPP_INFO(this->get_logger(), "Finished spinning");
  }

  double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
  }


  void navigate_to(double x, double y, double theta) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.z = sin(theta / 2.0);
    goal.pose.orientation.w = cos(theta / 2.0);

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    auto future_goal = navigate_client_->async_send_goal(nav_goal, send_goal_options);
    if (rclcpp::ok()) {
      auto goal_handle = future_goal.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Navigation goal was rejected by server.");
        return;
      }

      auto result_future = navigate_client_->async_get_result(goal_handle);
      auto result = result_future.get();
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(this->get_logger(), "Navigation failed.");
      } else {
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded.");
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiMapActionServer>());
  rclcpp::shutdown();
  return 0;
}
