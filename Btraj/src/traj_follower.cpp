#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class TrajFollower : public rclcpp::Node {
public:
    TrajFollower() : Node("traj_follower") {
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/traj_path", 10,
            [this](nav_msgs::msg::Path::SharedPtr msg) {
                path_ = msg->poses;
                idx_  = 0;
                RCLCPP_INFO(get_logger(), "Got path: %zu points", path_.size());
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                rx_ = msg->pose.pose.position.x;
                ry_ = msg->pose.pose.position.y;
                auto& q = msg->pose.pose.orientation;
                yaw_ = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                  1.0 - 2.0*(q.y*q.y + q.z*q.z));
            });

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajFollower::control_loop, this));
    }

private:
    void control_loop() {
        if (path_.empty() || idx_ >= path_.size()) return;

        double tx = path_[idx_].pose.position.x;
        double ty = path_[idx_].pose.position.y;
        double dx = tx - rx_;
        double dy = ty - ry_;
        double dist = std::hypot(dx, dy);

        if (dist < 0.3) { ++idx_; return; }

        double target_yaw = std::atan2(dy, dx);
        double err = target_yaw - yaw_;
        while (err >  M_PI) err -= 2.0 * M_PI;
        while (err < -M_PI) err += 2.0 * M_PI;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = (std::abs(err) < 0.6) ? 0.4 : 0.05;
        cmd.angular.z = 1.5 * err;
        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t idx_ = 0;
    double rx_ = 0.0, ry_ = 0.0, yaw_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajFollower>());
    rclcpp::shutdown();
    return 0;
}
