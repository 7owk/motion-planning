#ifndef B_TRAJ
#define B_TRAJ

#include <rclcpp/rclcpp.hpp>
#include <bits/stdc++.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <Eigen/Eigen>
#include <qpOASES.hpp>

using namespace std;
using namespace Eigen;
using namespace qpOASES;

namespace Bezier
{
/**
 *  bounding_box
 *  P1--------P2
 *   |        |
 *   |        |
 *   |        |
 *  P3--------P4
**/
struct bounding_box
{
    Vector3d center;
    Vector3i P1;
    Vector3i P2;
    Vector3i P3;
    Vector3i P4;
    bounding_box(){};
    ~bounding_box(){};
    bounding_box(const geometry_msgs::msg::Point& pt, const int& resolution){
        P1(0) = pt.x;
        P1(1) = pt.y;
        P1(2) = 0;
        P2 = P1;
        P3 = P1;
        P4 = P1;
        center(0) = pt.x + resolution / 2.0;
        center(1) = pt.y + resolution / 2.0;
        center(2) = 0;
    }
};

class bezier : public rclcpp::Node
{
private:
    float width;       // map 实际大小 = map 栅格格数（像素）* 分辨率
    float height;
    int   width_grid;  // map 栅格格数（像素）
    int   height_grid;
    float resolution = 1;
    Vector3d start;
    Vector3d goal;
    Vector3d start_v;
    Vector3d goal_v;
    Vector3d start_a;
    Vector3d goal_a;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr            Map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr           inflate_path;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr         goal_sub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr       box_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr            traj_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr            center_pub;

    nav_msgs::msg::OccupancyGrid::SharedPtr map;
    bounding_box box_last;
    vector<bounding_box> box_list;
    visualization_msgs::msg::MarkerArray box_list_vis;

    int max_inflate_iter = 1000;
    int traj_order = 6;
    vector<MatrixXd> MQM_list;
    vector<double> times;
    MatrixXd Aeq;
    MatrixXd Beq;
    MatrixXd Aieq;
    MatrixXd uBieq;
    MatrixXd lBieq;
    bool is_x = true;  // true: 优化 x 轴 / false: 优化 y 轴
    double vx_max;
    double vx_min;
    double vy_max;
    double vy_min;
    double ax_max;
    double ax_min;
    double ay_max;
    double ay_min;

public:
    bezier();
    ~bezier();
    void set_start(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial);
    void set_goal(const geometry_msgs::msg::PoseStamped::SharedPtr initial);
    void inflate(const geometry_msgs::msg::PoseArray::SharedPtr path);
    void inflate_box(bounding_box& box);
    void set_map(const nav_msgs::msg::OccupancyGrid::SharedPtr Map);
    bool is_contain(const bounding_box& box_last, const bounding_box& box_now);
    bool is_in_box(const geometry_msgs::msg::Point& pt);
    void visual_box(const vector<bounding_box>& box_list);
    geometry_msgs::msg::Point trans2pt(const Vector3i& p);
    int  delete_box(const bounding_box& box_last, const bounding_box& box_now,
                    const geometry_msgs::msg::PoseArray::SharedPtr path);
    void simplify_box();
    bool is_Overlap(const bounding_box& box_old, const bounding_box& box_now);
    void Trajectory_Generation();
    void vistraj(real_t* xopt, real_t* yopt);
    void set_MQMlist();
    void set_AeqBeq();
    void set_AieqBieq();
    MatrixXd get_M();
    void Time_allocate();
    geometry_msgs::msg::Point getBezierPos(const int& seg, const double& t,
                                           real_t* xopt, real_t* yopt);
    double Bernstein_base(const int& i, const double& t, const double& time_i);
    double factorial(const int& n);
    void get_Overlap_center(vector<Vector3d>& pt_list);
    void vis_center(const vector<Vector3d>& pt_list);
};
}

#endif
