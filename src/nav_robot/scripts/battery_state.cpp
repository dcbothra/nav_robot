#include <chrono>
#include <cmath>
#include <memory>

// Including only those packages that we need.
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class BatterySimulator : public rclcpp::Node {
public:
    BatterySimulator()
    : Node("battery_simulator"), battery_level_(1.0), moving_(false)
    {
        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_status", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&BatterySimulator::odomCallback, this, _1));
        timer_ = this->create_wall_timer(1s, std::bind(&BatterySimulator::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "🔋 Battery simulator node started.");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double battery_level_;
    bool moving_;
    geometry_msgs::msg::Point last_position_;

    const double charging_x_ = -3.5;
    const double charging_y_ = 0.0;
    const double charging_radius_ = 0.5;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto current_position = msg->pose.pose.position;

        if (last_position_.x != 0 || last_position_.y != 0) {
            double dx = current_position.x - last_position_.x;
            double dy = current_position.y - last_position_.y;
            double distance = std::hypot(dx, dy);
            moving_ = distance > 0.001;
        }

        last_position_ = current_position;
    }

    bool isAtChargingStation(const geometry_msgs::msg::Point& pos)
    {
        double dx = pos.x - charging_x_;
        double dy = pos.y - charging_y_;
        return std::hypot(dx, dy) < charging_radius_;
    }

    void timerCallback()
    {
        // Drain for idle operation
        battery_level_ -= 0.0005;

        // Additional drain if moving
        if (moving_) {
            battery_level_ -= 0.002;
        }

        // Charge if at charging station
        if (isAtChargingStation(last_position_)) {
            battery_level_ += 0.01;
        }

        // Clamp battery level
        battery_level_ = std::clamp(battery_level_, 0.0, 1.0);

        // Publish battery state
        auto msg = sensor_msgs::msg::BatteryState();
        msg.percentage = battery_level_;
        msg.voltage = 12.0 * battery_level_;
        msg.present = true;
        msg.power_supply_status = isAtChargingStation(last_position_)
            ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
            : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

        battery_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "🔋 Battery: %.1f%%", battery_level_ * 100.0);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatterySimulator>());
    rclcpp::shutdown();
    return 0;
}
