#include <rclcpp/rclcpp.hpp>
#include <bits/stdc++.h>
#include "Asearch.h"

using namespace std;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Astar_search::Asearch>();
    RCLCPP_WARN(node->get_logger(), "Astar program online!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
