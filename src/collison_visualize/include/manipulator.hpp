#ifndef __H_MANIPULATOR_H__
#define __H_MANIPULATOR_H__
// Manipulator需要的头文件
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
// Visualizer额外需要的头文件
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
// fcl库头文件
#include <fcl/math/constants.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>

class Link {
   public:
    using T = Eigen::Matrix4d;
    // D表示转轴z方向，A表示转轴x方向
    // 第i个link，如果为D则是局部坐标系为z_{i-1}轴，如果为A则是局部坐标系为x_{i}轴
    enum class Type { D, A };
    Link() = default;
    explicit Link(double radius, double length, Type type);
    Eigen::Vector3d GetCenterGlobal(const T& T_l_g) const;
    Eigen::Quaterniond GetOrientGlobal(const T& T_l_g) const;
    inline double GetRadius() const { return radius_; }
    inline double GetLength() const { return length_; }

   private:
    double radius_;
    double length_;
    Eigen::Vector3d center_local_;
    Eigen::Quaterniond orient_local_;
};

class Manipulator {
   public:
    using DHParamStruct = Eigen::Vector4d;
    using T = Eigen::Matrix4d;
    Manipulator() {
        // 6个link下标到6，所以用7防止索引超
        int cols = 7, rows = 7;
        T_table_ = std::vector<std::vector<Eigen::Matrix4d>>(
            rows,
            std::vector<Eigen::Matrix4d>(cols, Eigen::Matrix4d::Identity()));
    }
    void SetDHParamTable(
        const std::vector<Manipulator::DHParamStruct>& dh_params_table_in);
    std::vector<Manipulator::DHParamStruct> GetDHParamTable();
    void ForwardKinematics(const std::vector<double>& q_in);
    Manipulator::T GetT(int a, int b);
    // 返回real link的数量，DH表数目
    int GetLinkNum();
    void UpdateLink();

    // Link数组，一个real Link由一个A和D的Link组成
    std::vector<Link> link_d_lst_;
    std::vector<Link> link_a_lst_;

   private:
    // 传统DH表:theta, d, a, alpha, 转轴分别是x_i和z_{i-1}角度采用弧度制
    std::vector<DHParamStruct> dh_params_table_;
    // 形如T_table[a][b]表示从a坐标系到b坐标系的变换矩阵
    std::vector<std::vector<T>> T_table_;
};

class CollisionDetector {
   public:
    using T = Eigen::Matrix4d;
    // 定义顶点类型
    using Vertex = std::array<float, 3>;  // [x, y, z]
    // 定义三角形类型
    using Triangle = std::array<Vertex, 3>;  // 三个点组成的三角形
    CollisionDetector() = default;

    bool LoadObsSTLFile(std::string stl_file_path);
    // true代表会撞到
    bool CheckCollision(const CollisionDetector::T& T_l_g, const Link& link);

   private:
    std::vector<Triangle> obs_triangles_;
    std::shared_ptr<fcl::BVHModel<fcl::kIOS<double>>> mesh_geometry_ptr_;
    std::shared_ptr<fcl::CollisionObjectd> obs_stl_collison_obj_ptr_;
};

class Visualizer {
   public:
    // 圆柱体的颜色属性r, g, b, a
    using Color = Eigen::Matrix<double, 4, 1>;
    using T = Eigen::Matrix4d;
    Visualizer() = default;
    Visualizer(std::shared_ptr<Manipulator> manipulator_ptr,
               std::string obs_stl_file_path);
    void Update(const T& T_i_0, const T& T_pre_0, int i);
    void GetMarkerLst(std::vector<visualization_msgs::msg::Marker>& marker_lst);
    void GetObsMarker(visualization_msgs::msg::Marker& marker_obs);

   private:
    std::shared_ptr<Manipulator> manipulator_ptr_;
    std::vector<visualization_msgs::msg::Marker> marker_lst_d_;
    std::vector<visualization_msgs::msg::Marker> marker_lst_a_;
    visualization_msgs::msg::Marker marker_obs_;
    std::vector<Color> color_lst_;
};
#endif  //__H_MANIPULATOR_H__