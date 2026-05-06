#ifndef ASEARCH_H
#define ASEARCH_H

#include <rclcpp/rclcpp.hpp>
#include <bits/stdc++.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <boost/heap/binomial_heap.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace Astar_search
{
    struct Node2D
    {
        float x;
        float y;
        Node2D* parent=NULL;
        float g=0;
        float h=0;
        float distance_cost=0; // 与障碍物距离的代价，离障碍物近则 cost 大
        int idx_x=-1;
        int idx_y=-1;
        bool is_Occupancy=false;
        int state=0; // 0: 未扩展  1: 在 openlist 中  -1: 在 closedlist 中
        Node2D(){};
        Node2D(float X,float Y):x(X),y(Y),parent(NULL){}
        Node2D(const geometry_msgs::msg::PoseStamped::SharedPtr initial)
        {
            x=initial->pose.position.x;
            y=initial->pose.position.y;
            parent=NULL;
        }
        Node2D(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial)
        {
            x=initial->pose.pose.position.x;
            y=initial->pose.pose.position.y;
            parent=NULL;
        }
        ~Node2D(){}

        float getC(){return (g+h+distance_cost);}
    };

    struct CompareNodes {
        bool operator()(Node2D* lhs, Node2D* rhs) const {
            return lhs->getC() > rhs->getC();
        }
    };

    class Asearch : public rclcpp::Node
    {
    private:
        Node2D* start = nullptr;  // 类内的指针一定要用 new 初始化
        Node2D* goal = nullptr;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   position_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   pathvis_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr     path_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   node_pub;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr            Map_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr         goal_sub;
        nav_msgs::msg::Path path;
        Node2D* gridmap = nullptr;
        float width;       // map 实际大小 = map 栅格格数（像素）* 分辨率
        float height;
        int width_grid;    // map 栅格格数（像素）
        int height_grid;
        float resolution=1;
        bool get_s=false;
        bool get_g=false;
        nav_msgs::msg::OccupancyGrid::SharedPtr map;
        boost::heap::binomial_heap<Node2D*, boost::heap::compare<CompareNodes>> Openset;

    public:
        Asearch();
        ~Asearch();
        void set_start(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial);
        void set_goal(const geometry_msgs::msg::PoseStamped::SharedPtr initial);
        void get_start();
        void get_goal();
        void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
        bool is_collisionfree(Node2D* node);
        void transform2idx(Node2D* node);
        void reset_map();
        float Heuristics_cost(Node2D* start,Node2D* goal);
        void trackback(Node2D* node);
        Node2D* findpath();
        geometry_msgs::msg::Point idx_INV(const int &x,const int & y);
        float distance_cost(Node2D* node);
    };

}

#endif
