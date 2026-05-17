#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

class FollowerCommandRelay : public rclcpp::Node
{
public:
  FollowerCommandRelay() : Node("leader_follower_teleop") {
    RCLCPP_INFO(get_logger(), "Initializing FollowerCommandRelay...");

    // Parameters
    // Arm output mode:
    //  - "joint_trajectory" => JointTrajectoryController topic
    //  - "forward_position" => ForwardController commands topic
    arm_mode_ = this->declare_parameter<std::string>("arm_mode", "joint_trajectory");

    leader_topic_ = declare_parameter<std::string>("leader_topic", "/leader/joint_states");
    follower_jtc_topic_ = declare_parameter<std::string>(
        "jtc_topic", "/follower/trajectory_controller/joint_trajectory");
    follower_fwd_topic_ =
        declare_parameter<std::string>("fwd_topic", "/follower/forward_controller/commands");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 0.25);
    point_dt_s_ = declare_parameter<double>("point_dt_s", 0.06);
    lpf_alpha_ = declare_parameter<double>("lpf_alpha", 1.0);
    gripper_map_enabled_ = declare_parameter<bool>("gripper_map_enabled", false);
    gripper_joint_name_ = declare_parameter<std::string>("gripper_joint_name", "gripper");
    gripper_leader_open_ = declare_parameter<double>("gripper_leader_open", -0.174533);
    gripper_leader_close_ = declare_parameter<double>("gripper_leader_close", 1.74533);
    gripper_follower_open_ = declare_parameter<double>("gripper_follower_open", -0.174533);
    gripper_follower_close_ = declare_parameter<double>("gripper_follower_close", 1.74533);

    arm_joints_ = declare_parameter<std::vector<std::string>>(
        "arm_joints", std::vector<std::string>{"shoulder_pan", "shoulder_lift", "elbow_flex",
                                               "wrist_flex", "wrist_roll", "gripper"});

    filtered_.assign(arm_joints_.size(), 0.0);

    RCLCPP_INFO(get_logger(), "Leader: %s", leader_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Follower JTC: %s", follower_jtc_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Rate: %.1f Hz, Arm joints: %zu", publish_rate_hz_,
                arm_joints_.size());
    if (gripper_map_enabled_) {
      RCLCPP_INFO(get_logger(),
                  "Gripper remap enabled: %s leader %.3f..%.3f -> follower %.3f..%.3f",
                  gripper_joint_name_.c_str(), gripper_leader_open_, gripper_leader_close_,
                  gripper_follower_open_, gripper_follower_close_);
    }

    // ROS interfaces
    leader_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        leader_topic_, rclcpp::SensorDataQoS(),
        std::bind(&FollowerCommandRelay::joint_state_callback, this, std::placeholders::_1));

    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        follower_jtc_topic_, rclcpp::QoS(10).reliable());
    forward_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(follower_fwd_topic_,
                                                                      rclcpp::QoS(10).reliable());

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz_),
                               std::bind(&FollowerCommandRelay::control_loop, this));

    raw_arm_.resize(arm_joints_.size(), 0.0);

    RCLCPP_INFO(get_logger(), "FollowerCommandRelay initialized.");
  }

private:
  // Parameters
  std::string arm_mode_;
  std::string leader_topic_;
  std::string follower_jtc_topic_;
  std::string follower_fwd_topic_;
  double publish_rate_hz_{50.0};
  double stale_timeout_s_{0.25};
  double point_dt_s_{0.02};
  double lpf_alpha_{1.0}; // 1.0 = no filtering
  bool gripper_map_enabled_{false};
  std::string gripper_joint_name_{"gripper"};
  double gripper_leader_open_{-0.174533};
  double gripper_leader_close_{1.74533};
  double gripper_follower_open_{-0.174533};
  double gripper_follower_close_{1.74533};
  bool have_filtered_{false};
  std::vector<std::string> arm_joints_;
  std::vector<double> filtered_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  bool initialized_{false};
  std::vector<int> arm_idx_;
  std::vector<double> raw_arm_;
  int gripper_arm_idx_{-1};
  rclcpp::Time last_leader_stamp_{0, 0, RCL_ROS_TIME};

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    if (!initialized_ && !initialize_indices(*msg)) return;

    last_leader_stamp_ = this->now();

    // Cache targets
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      raw_arm_[i] = msg->position[arm_idx_[i]];
    }
    if (gripper_map_enabled_ && gripper_arm_idx_ >= 0) {
      raw_arm_[gripper_arm_idx_] = map_gripper(raw_arm_[gripper_arm_idx_]);
    }
  }

  bool initialize_indices(const sensor_msgs::msg::JointState &msg) {
    // Build name -> index map
    std::unordered_map<std::string, int> idx_map;
    idx_map.reserve(msg.name.size());
    for (size_t i = 0; i < msg.name.size(); i++)
      idx_map[msg.name[i]] = static_cast<int>(i);

    // Arm indexes
    arm_idx_.assign(arm_joints_.size(), -1);
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      auto it = idx_map.find(arm_joints_[i]);
      if (it == idx_map.end()) {
        RCLCPP_ERROR(this->get_logger(), "Leader arm joint '%s' not found", arm_joints_[i].c_str());
        return false;
      }
      arm_idx_[i] = it->second;
      if (arm_joints_[i] == gripper_joint_name_) {
        gripper_arm_idx_ = static_cast<int>(i);
      }
    }

    if (gripper_map_enabled_ && gripper_arm_idx_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Gripper remap enabled but joint '%s' is not in arm_joints",
                   gripper_joint_name_.c_str());
      return false;
    }

    initialized_ = true;
    RCLCPP_INFO(get_logger(), "Initialized: %zu arm joints", arm_joints_.size());
    return true;
  }

  double map_gripper(double leader_value) const {
    const double denominator = gripper_leader_close_ - gripper_leader_open_;
    if (std::abs(denominator) < 1e-6) {
      return leader_value;
    }
    const double ratio = std::clamp((leader_value - gripper_leader_open_) / denominator, 0.0, 1.0);
    const double mapped =
        gripper_follower_open_ + ratio * (gripper_follower_close_ - gripper_follower_open_);
    const auto bounds = std::minmax(gripper_follower_open_, gripper_follower_close_);
    return std::clamp(mapped, bounds.first, bounds.second);
  }

  void control_loop() {

    if (!initialized_) return;

    const auto now = this->now();

    // Leader data stale: do nothing (holds last command on follower)
    if ((now - last_leader_stamp_).seconds() > stale_timeout_s_) return;

    // LFP if used
    // if (!have_filtered_ || lpf_alpha_ >= 0.999) {
    //   filtered_ = raw_arm_;
    //   have_filtered_ = true;
    // } else {
    //   for (size_t i = 0; i < raw_arm_.size(); ++i) {
    //     filtered_[i] = lpf_alpha_ * raw_arm_[i] + (1.0 - lpf_alpha_) * filtered_[i];
    //   }
    // }

    // publish arm
    publish_arm(now);
  }

  void publish_arm(const rclcpp::Time &time) {
    if (arm_mode_ == "joint_trajectory") {
      trajectory_msgs::msg::JointTrajectory jt;
      jt.header.stamp = time;
      jt.joint_names = arm_joints_;

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = raw_arm_;

      const int sec = static_cast<int>(point_dt_s_);
      const int nsec = static_cast<int>((point_dt_s_ - sec) * 1e9);
      pt.time_from_start.sec = sec;
      pt.time_from_start.nanosec = nsec;
      jt.points.push_back(pt);
      trajectory_pub_->publish(jt);
    } else {
      std_msgs::msg::Float64MultiArray cmd;
      cmd.data = raw_arm_;
      forward_pub_->publish(cmd);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowerCommandRelay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
