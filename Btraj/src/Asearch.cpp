#include "Asearch.h"

using namespace std;
using namespace Astar_search;

// 对于栅格地图有 N 个像素点格数，相对应的索引 idx 应该是 0~N-1

Asearch::Asearch() : rclcpp::Node("astar")
{
    path.header.frame_id = "map";
    // 注：这里用 map 坐标系作为基准。tf_br 节点会广播 map -> odom 偏移 (10,10,0)，
    // 但所有点的计算/显示都在 map 坐标下进行。

    // ---- Publishers ----
    position_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/start", 1);
    node_pub     = this->create_publisher<visualization_msgs::msg::Marker>(
        "/pathNodes", 1);
    pathvis_pub  = this->create_publisher<visualization_msgs::msg::Marker>(
        "/path", 1);
    path_pub     = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/path_to_btraj", 1);

    // ---- Subscribers ----
    // /map 来自 nav2_map_server，发布 QoS 为 transient_local，因此订阅端必须匹配
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    Map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos,
        std::bind(&Asearch::setMap, this, std::placeholders::_1));

    start_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&Asearch::set_start, this, std::placeholders::_1));

    // ROS 2 RViz 的 "2D Goal Pose" 默认发到 /goal_pose
    goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1,
        std::bind(&Asearch::set_goal, this, std::placeholders::_1));
}

Asearch::~Asearch()
{
    if (gridmap) delete[] gridmap;
}

void Asearch::set_start(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial)
{
    start = new Node2D();
    start->x = initial->pose.pose.position.x;
    start->y = initial->pose.pose.position.y;
    start->parent = nullptr;

    geometry_msgs::msg::PoseStamped start_pub;
    start_pub.header.frame_id = "map";
    start_pub.header.stamp = this->now();
    start_pub.pose.position    = initial->pose.pose.position;
    start_pub.pose.orientation = initial->pose.pose.orientation;
    position_pub->publish(start_pub);

    transform2idx(start);
    if (is_collisionfree(start)) {
        get_start();
        get_s = true;
        if (get_s && get_g) {
            Node2D* p = findpath();
            trackback(p);
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid start!");
    }
}

void Asearch::get_start()
{
    RCLCPP_INFO(this->get_logger(), "get_start: %f, %f", start->x, start->y);
}

void Asearch::set_goal(const geometry_msgs::msg::PoseStamped::SharedPtr initial)
{
    goal = new Node2D();
    goal->x = initial->pose.position.x;
    goal->y = initial->pose.position.y;
    transform2idx(goal);
    if (is_collisionfree(goal)) {
        get_goal();
        get_g = true;
        if (get_s && get_g) {
            Node2D* p = findpath();
            trackback(p);
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid goal!");
    }
}

void Asearch::get_goal()
{
    RCLCPP_INFO(this->get_logger(), "get_goal: %f, %f", goal->x, goal->y);
}

void Asearch::setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr Map)
{
    map = Map;
    width_grid  = map->info.width;
    height_grid = map->info.height;
    resolution  = map->info.resolution;
    width  = static_cast<float>(width_grid)  * resolution;
    height = static_cast<float>(height_grid) * resolution;

    if (gridmap) delete[] gridmap;
    gridmap = new Node2D[width_grid * height_grid];
    for (int x = 0; x < width_grid; ++x) {
        for (int y = 0; y < height_grid; ++y) {
            gridmap[y*width_grid + x].is_Occupancy = (map->data[y*width_grid + x] ? true : false);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Map received: %d x %d, resolution %.3f",
                width_grid, height_grid, resolution);
}

void Asearch::reset_map()
{
    delete[] gridmap;
    gridmap = new Node2D[width_grid * height_grid];
    for (int x = 0; x < width_grid; ++x) {
        for (int y = 0; y < height_grid; ++y) {
            gridmap[y*width_grid + x].is_Occupancy = (map->data[y*width_grid + x] ? true : false);
        }
    }
}

bool Asearch::is_collisionfree(Node2D* node)
{
    bool bounding = (node->idx_x >= 0 && node->idx_x < width_grid &&
                     node->idx_y >= 0 && node->idx_y < height_grid);
    if (!bounding) return false;
    bool collisionfree = !gridmap[node->idx_y*width_grid + node->idx_x].is_Occupancy;
    return collisionfree;
}

void Asearch::transform2idx(Node2D* node)
{
    node->idx_x = node->x / resolution;
    node->idx_y = node->y / resolution;
}

float Asearch::Heuristics_cost(Node2D* s, Node2D* g)
{
    float distance = std::sqrt(std::pow((g->idx_x - s->idx_x) * resolution, 2) +
                               std::pow((g->idx_y - s->idx_y) * resolution, 2));
    return distance * 1.0001f;  // tie breaker
}

Node2D* Asearch::findpath()
{
    Openset.clear();
    reset_map();
    bool vis_nodes = true;

    visualization_msgs::msg::Marker pathNode;
    pathNode.header.frame_id = "map";
    pathNode.ns = "visnode";
    pathNode.id = 0;
    pathNode.header.stamp = this->now();
    pathNode.type   = visualization_msgs::msg::Marker::SPHERE_LIST;
    pathNode.action = visualization_msgs::msg::Marker::ADD;
    pathNode.scale.x = 0.5;
    pathNode.scale.y = 0.5;
    pathNode.scale.z = 0.5;
    pathNode.color.a = 1.0;
    pathNode.color.r = 0.0f;
    pathNode.color.g = 1.0f;
    pathNode.color.b = 0.0f;
    pathNode.lifetime = rclcpp::Duration(0, 0);

    int max_iter = 300000, iter = 0;
    start->h = Heuristics_cost(start, goal);
    Openset.push(start);
    while (!Openset.empty()) {
        Node2D* current = Openset.top();
        Openset.pop();

        if (current->idx_x == goal->idx_x && current->idx_y == goal->idx_y) {
            if (vis_nodes) node_pub->publish(pathNode);
            RCLCPP_WARN(this->get_logger(), "Find Goal!");
            return current;
        }
        if (vis_nodes) {
            pathNode.points.push_back(idx_INV(current->idx_x, current->idx_y));
        }
        gridmap[current->idx_y*width_grid + current->idx_x].state = -1;

        Node2D* succ = new Node2D();
        int temp_idxx = 0, temp_idxy = 0;
        for (int x = -1; x <= 1; ++x) {
            for (int y = -1; y <= 1; ++y) {
                if (x == 0 && y == 0) continue;
                temp_idxx = current->idx_x + x;
                temp_idxy = current->idx_y + y;
                if (temp_idxx < 0 || temp_idxx >= width_grid ||
                    temp_idxy < 0 || temp_idxy >= height_grid) continue;
                if (gridmap[temp_idxy*width_grid + temp_idxx].is_Occupancy ||
                    gridmap[temp_idxy*width_grid + temp_idxx].state == -1) continue;
                succ->state = 1;
                succ->idx_x = temp_idxx;
                succ->idx_y = temp_idxy;
                succ->g = current->g + std::sqrt(std::pow(x*resolution, 2) + std::pow(y*resolution, 2));
                succ->h = Heuristics_cost(succ, goal);
                succ->parent = current;
                succ->distance_cost = distance_cost(succ);
                if (gridmap[temp_idxy*width_grid + temp_idxx].state == 1 && is_collisionfree(succ)) {
                    if ((succ->g + succ->distance_cost) <
                        (gridmap[temp_idxy*width_grid + temp_idxx].g +
                         gridmap[temp_idxy*width_grid + temp_idxx].distance_cost))
                        gridmap[temp_idxy*width_grid + temp_idxx] = *succ;
                } else if (is_collisionfree(succ)) {
                    gridmap[temp_idxy*width_grid + temp_idxx] = *succ;
                    Openset.push(&gridmap[temp_idxy*width_grid + temp_idxx]);
                }
            }
        }
        ++iter;
        if (iter > max_iter) {
            RCLCPP_WARN(this->get_logger(), "Maximum iteration has been reached !");
            delete succ;
            return nullptr;
        }
        delete succ;
    }
    return nullptr;
}

void Asearch::trackback(Node2D* node)
{
    if (!node) return;

    visualization_msgs::msg::Marker Apath;
    geometry_msgs::msg::PoseArray path_to_inflate;
    geometry_msgs::msg::Pose temp;
    Apath.header.frame_id = "map";
    Apath.ns = "visApath";
    Apath.id = 0;
    Apath.header.stamp = this->now();
    Apath.type   = visualization_msgs::msg::Marker::SPHERE_LIST;
    Apath.action = visualization_msgs::msg::Marker::ADD;
    Apath.scale.x = 0.5;
    Apath.scale.y = 0.5;
    Apath.scale.z = 0.5;
    Apath.color.a = 1.0;
    Apath.color.r = 1.0f;
    Apath.color.g = 0.0f;
    Apath.color.b = 0.0f;
    Apath.lifetime = rclcpp::Duration(0, 0);
    while (node) {
        Apath.points.push_back(idx_INV(node->idx_x, node->idx_y));
        temp.position.x = node->idx_x;
        temp.position.y = node->idx_y;
        path_to_inflate.poses.push_back(temp);
        node = node->parent;
    }
    pathvis_pub->publish(Apath);
    path_pub->publish(path_to_inflate);
}

geometry_msgs::msg::Point Asearch::idx_INV(const int &x, const int &y)
{
    geometry_msgs::msg::Point pt;
    pt.x = x + resolution / 2;
    pt.y = y + resolution / 2;
    pt.z = 0;
    return pt;
}

float Asearch::distance_cost(Node2D* node)
{
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            if (x == 0 && y == 0) continue;
            int nx = node->idx_x + x, ny = node->idx_y + y;
            if (nx < 0 || nx >= width_grid || ny < 0 || ny >= height_grid) continue;
            if (gridmap[ny*width_grid + nx].is_Occupancy) {
                return 10.0f;
            }
        }
    }
    return 0.0f;
}
