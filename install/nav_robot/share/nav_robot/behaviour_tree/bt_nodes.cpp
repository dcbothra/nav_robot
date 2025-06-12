#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>
using namespace std::chrono_literals;


class BTContext
{
public:
  static rclcpp::Node::SharedPtr node()
  {
    static std::once_flag flag;
    std::call_once(flag, [] {
      node_ = rclcpp::Node::make_shared("bt_plugin_node");
      spin_thread_ = std::thread([] { rclcpp::spin(node_); });
    });
    return node_;
  }

  static void shutdown()
  {
    rclcpp::shutdown();
    if (spin_thread_.joinable()) spin_thread_.join();
  }

private:
  static rclcpp::Node::SharedPtr node_;
  static std::thread spin_thread_;
};

rclcpp::Node::SharedPtr BTContext::node_{};
std::thread BTContext::spin_thread_;

static geometry_msgs::msg::PoseStamped makePose(const YAML::Node& n)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = n["header"]["frame_id"].as<std::string>();
  p.pose.position.x = n["pose"]["position"]["x"].as<double>();
  p.pose.position.y = n["pose"]["position"]["y"].as<double>();
  p.pose.position.z = n["pose"]["position"]["z"].as<double>();
  p.pose.orientation.z = n["pose"]["orientation"]["z"].as<double>();
  p.pose.orientation.w = n["pose"]["orientation"]["w"].as<double>();
  return p;
}

class GetNextGoal : public BT::StatefulActionNode
{
public:
  GetNextGoal(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::StatefulActionNode(name, cfg)
  {
    node_ = BTContext::node();
    declareAndLoadPoses();
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal") };
  }

  BT::NodeStatus onStart() override
  {
    return tickOnce();
  }

  BT::NodeStatus onRunning() override
  {
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  BT::NodeStatus tickOnce()
  {
    bool charging = false;
    this->config().blackboard->get("charging_mode", charging);

    geometry_msgs::msg::PoseStamped pose;

    if (charging)
    {
      this->config().blackboard->get("saved_index", index_);
      pose = charging_pose_;
    }
    else
    {
      if (waypoints_.empty())
      {
        RCLCPP_ERROR(node_->get_logger(), "No waypoints loaded!");
        return BT::NodeStatus::FAILURE;
      }

      pose = waypoints_[index_];
      this->config().blackboard->set("saved_index", index_);
      index_ = (index_ + 1) % waypoints_.size();
    }

    pose.header.stamp = node_->now();
    setOutput("goal", pose);

    RCLCPP_INFO(node_->get_logger(), "GetNextGoal → (%.2f, %.2f)",
                pose.pose.position.x, pose.pose.position.y);
    return BT::NodeStatus::SUCCESS;
  }

  void declareAndLoadPoses()
  {
    std::string default_yaml =
      ament_index_cpp::get_package_share_directory("nav_robot") + "/config/poses.yaml";

    node_->declare_parameter("poses_file", default_yaml);
    auto yaml_path = node_->get_parameter("poses_file").as_string();

    try
    {
      YAML::Node root = YAML::LoadFile(yaml_path);
      for (const auto& wp : root["waypoints"])
        waypoints_.push_back(makePose(wp["pose"]));

      charging_pose_ = makePose(root["charging"][0]["pose"]);

      this->config().blackboard->set("charging_station", charging_pose_);
      this->config().blackboard->set("charging_mode", false);
      this->config().blackboard->set("saved_index", size_t(0));

      RCLCPP_INFO(node_->get_logger(), "Loaded %zu waypoints + charging pose from %s",
                  waypoints_.size(), yaml_path.c_str());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "YAML load error: %s", e.what());
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  geometry_msgs::msg::PoseStamped charging_pose_;
  size_t index_{0};
};

class DoPickDrop : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(rclcpp::get_logger("DoPickDrop"), "Simulating Pick/Drop...");
    std::this_thread::sleep_for(1s);
    return BT::NodeStatus::SUCCESS;
  }
};

class CheckBatteryLevel : public BT::ConditionNode
{
public:
    CheckBatteryLevel(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::ConditionNode(name, cfg)
    {
    node_ = BTContext::node();

    sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_status", 10,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        level_ = msg->percentage * 100.0;
        });
    }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    double threshold = 20.0;
    bool low = level_ < threshold;
    this->config().blackboard->set("charging_mode", low);

    RCLCPP_INFO(node_->get_logger(), "Battery: %.1f%% [%s]",
                level_, low ? "LOW → CHARGING" : "OK → CONTINUE");

    return low ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;
  double level_{100.0};
};

class SimulateCharging : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(rclcpp::get_logger("SimulateCharging"), "Charging for 10 seconds...");
    std::this_thread::sleep_for(10s);
    return BT::NodeStatus::SUCCESS;
  }
};

class ResumeLastTask : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    this->config().blackboard->set("charging_mode", false);
    RCLCPP_INFO(rclcpp::get_logger("ResumeLastTask"), "Resuming task from saved index.");
    return BT::NodeStatus::SUCCESS;
  }
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GetNextGoal>("GetNextGoal");
  factory.registerNodeType<DoPickDrop>("DoPickDrop");
  factory.registerNodeType<CheckBatteryLevel>("CheckBatteryLevel");
  factory.registerNodeType<SimulateCharging>("SimulateCharging");
  factory.registerNodeType<ResumeLastTask>("ResumeLastTask");
}
