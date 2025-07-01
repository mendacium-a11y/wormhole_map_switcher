#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "custom_msg/srv/save_pose.hpp"
#include "multi_map_nav/pose_saver_db.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class SavePoseNode : public rclcpp::Node {
public:
  SavePoseNode()
    : Node("save_pose_node"), db_("wormholes.db") {

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        latest_pose_ = *msg;
        pose_received_ = true;
      });

    service_ = this->create_service<custom_msg::srv::SavePose>(
      "save_pose",
      std::bind(&SavePoseNode::handle_save_pose, this, _1, _2));
  }

private:
  void handle_save_pose(
    const std::shared_ptr<custom_msg::srv::SavePose::Request> req,
    std::shared_ptr<custom_msg::srv::SavePose::Response> res)
  {
    if (!pose_received_) {
      res->success = false;
      res->message = "No AMCL pose received yet.";
      return;
    }

    const auto &pose = latest_pose_.pose.pose;
    double x = pose.position.x;
    double y = pose.position.y;

    // Get theta (yaw) from quaternion
    double siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z);
    double theta = std::atan2(siny_cosp, cosy_cosp);

    bool success = db_.savePose(req->map_name, req->point_name, x, y, theta);
    res->success = success;
    res->message = success ? "Pose saved!" : "Failed to save pose to DB.";
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Service<custom_msg::srv::SavePose>::SharedPtr service_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;
  bool pose_received_ = false;
  PoseSaverDB db_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SavePoseNode>());
  rclcpp::shutdown();
  return 0;
}
