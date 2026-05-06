#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

/**
 *  发布 map -> odom 的静态偏移 (10, 10, 0)
 *  使用 ROS 2 的 StaticTransformBroadcaster:
 *    - 只发布一次（latched / transient_local QoS）
 *    - 任何后续订阅 /tf_static 的节点都会立即收到
 *    - 比 ROS1 的 100Hz 循环更节省资源
 */
class StaticTfBr : public rclcpp::Node
{
public:
    StaticTfBr() : rclcpp::Node("tf_broadcaster")
    {
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp    = this->now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id  = "odom";
        tf_msg.transform.translation.x = 10.0;
        tf_msg.transform.translation.y = 10.0;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x    = 0.0;
        tf_msg.transform.rotation.y    = 0.0;
        tf_msg.transform.rotation.z    = 0.0;
        tf_msg.transform.rotation.w    = 1.0;

        broadcaster_->sendTransform(tf_msg);
        RCLCPP_INFO(this->get_logger(),
                    "Published static transform map -> odom (10, 10, 0)");
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTfBr>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
