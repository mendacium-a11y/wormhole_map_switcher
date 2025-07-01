#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "multi_map_nav/wormhole_db.hpp"
#include "custom_msg/srv/save_wormhole.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class WormholeSaver : public rclcpp::Node {
public:
  WormholeSaver()
    : Node("wormhole_saver"), db_("wormholes.db") {

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        last_pose_ = *msg;
        pose_received_ = true;
      });

    service_ = this->create_service<custom_msg::srv::SaveWormhole>(
      "save_wormhole",
      std::bind(&WormholeSaver::handle_save, this, _1, _2));
  }

private:
  void handle_save(
    const std::shared_ptr<custom_msg::srv::SaveWormhole::Request> req,
    std::shared_ptr<custom_msg::srv::SaveWormhole::Response> res)
  {
    if (!pose_received_) {
      res->success = false;
      res->message = "No pose received yet.";
      return;
    }

    // You can change this to be dynamically obtained via param or service
    // std::string current_map = req->current_map;  // or from a param

    const auto &pose = last_pose_.pose.pose;
    double x = pose.position.x;
    double y = pose.position.y;

    // Extract yaw from quaternion
    double siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z);
    double theta = std::atan2(siny_cosp, cosy_cosp);

    bool success = db_.saveWormhole(req->current_map, req->target_map, x, y, theta);
    res->success = success;
    res->message = success ? "Wormhole saved." : "DB error";
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Service<custom_msg::srv::SaveWormhole>::SharedPtr service_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
  bool pose_received_ = false;
  WormholeDB db_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WormholeSaver>());
  rclcpp::shutdown();
  return 0;
}
