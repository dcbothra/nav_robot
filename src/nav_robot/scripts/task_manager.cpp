// // #include <chrono>
// // #include <functional>
// // #include <memory>
// // #include <cmath>
// // #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/battery_state.hpp"
// #include "std_msgs/msgs/int32.hpp"


// // using namespace std::chrono_literals;

// /* This example creates a subclass of Node and uses std::bind() to register a
//  * member function as a callback from the timer. */

// class TaskManager : public rclcpp::Node
// {
// public:
//   TaskManager()
//   : Node("task_manager"), task_stage_(0), charging_(false)
//   {
//     task_pub_ = this->create_publisher<std_msgs::msg::Int32>("task_state", 10);
//     battery_sub_ = this->create_subscriber<sensor_msgs::msg::BatteryState>("battery_status", 10, std::bind(&TaskStateManager::batteryCallback, this, std::placeholders::_1));
//     timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MinimalPublisher::timer_callback, this));

//     RCLCPP_INFO(this->get_logger(), "Task State Manager started.");
//   }

// private:
//   int task_stage_;
//   bool charging_;
//   double battery_level_;

//   rclcpp::Publisher<std_msgs::msg::Int>::SharedPtr task_pub_;
//   rclcpp::Subscriber<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
//   rclcpp::TimerBase::SharedPtr timer_;

//   void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg){
//     battery_level_ = msg->percentage;

//     if (battery_level_ < 0.2 && !charging_){
//         charging_ = true;
//         RCLCPP_WARN(this->get_logger(), "battery low, pausing task at stage %d", task_stage_);
//     }
//     else if (battery_level_ > 0.99 && charging_){
//         charging_ = false;
//         RCLCPP_WARN(this->get_logger(), "battery charged, resuming task at stage %d", task_stage_);
//     }
//   }

//   void task_callback()
//   {
//     auto msg = sensor_msgs::msg::BatteryState();
//   }

//   void timer_callback()
//   {
//     auto message = std_msgs::msg::String();
//     message.data = "Hello, world! " + std::to_string(count_++);
//     RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//     publisher_->publish(message);
//   }
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

class TaskManager : public rclcpp::Node {
public:
    TaskManager() : Node("task_manager"), current_task_index_(0), charging_(false), battery_level_(1.0)
    {
        declare_parameter<std::string>("waypoints_file", "waypoints.yaml");
        auto file_path = get_parameter("waypoints_file").as_string();

        battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_status", 10,
            std::bind(&TaskManager::batteryCallback, this, std::placeholders::_1));

        goal_service_ = create_service<std_srvs::srv::Trigger>(
            "/get_next_goal",
            std::bind(&TaskManager::handleGoalRequest, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Task Manager node initialized.");
    }

private:
    std::vector<geometry_msgs::msg::PoseStamped> task_goals_;
    geometry_msgs::msg::PoseStamped charging_goal_;

    int current_task_index_;
    bool charging_;
    double battery_level_;

    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goal_service_;

    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        battery_level_ = msg->percentage;
        if (battery_level_ < 0.2 && !charging_) {
            charging_ = true;
            RCLCPP_WARN(get_logger(), "🔋 Battery low. Switching to charging mode.");
        } else if (charging_ && battery_level_ >= 0.99) {
            charging_ = false;
            RCLCPP_INFO(get_logger(), "✅ Fully charged. Resuming task.");
        }
    }

    void handleGoalRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        geometry_msgs::msg::PoseStamped next_goal;

        if (charging_) {
            next_goal = charging_goal_;
        } else {
            next_goal = task_goals_[current_task_index_];
            current_task_index_ = (current_task_index_ + 1) % task_goals_.size();
        }

        std::ostringstream ss;
        ss << next_goal.pose.position.x << ", " << next_goal.pose.position.y;
        response->success = true;
        response->message = ss.str();

        // Publish the goal as a latched topic or expose a getter for use in the BT node
        RCLCPP_INFO(get_logger(), "📍 Next goal: %s", ss.str().c_str());
    }

    bool loadWaypoints(const std::string& path)
    {
        YAML::Node yaml = YAML::LoadFile(path);
        for (const auto& wp : yaml["waypoints"]) {
            task_goals_.push_back(parsePoseStamped(wp["pose"]));
        }
        charging_goal_ = parsePoseStamped(yaml["charging"][0]["pose"]);
        return true;
    }

    geometry_msgs::msg::PoseStamped parsePoseStamped(const YAML::Node& node)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = node["header"]["frame_id"].as<std::string>();
        ps.pose.position.x = node["pose"]["position"]["x"].as<double>();
        ps.pose.position.y = node["pose"]["position"]["y"].as<double>();
        ps.pose.position.z = node["pose"]["position"]["z"].as<double>();
        ps.pose.orientation.z = node["pose"]["orientation"]["z"].as<double>();
        ps.pose.orientation.w = node["pose"]["orientation"]["w"].as<double>();
        return ps;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskManager>());
    rclcpp::shutdown();
    return 0;
}
