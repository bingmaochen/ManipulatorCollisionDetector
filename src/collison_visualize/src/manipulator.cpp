// Manipulator需要的头文件
#include "manipulator.hpp"
// Visualizer需要的额外头文件

void Manipulator::SetDHParamTable(
    const std::vector<Manipulator::DHParamStruct>& dh_params_table_in) {
    // 初始化H参数表
    dh_params_table_ = dh_params_table_in;
    int joint_num = dh_params_table_.size();

    // 初始化T变换矩阵
    std::vector<double> q(joint_num);
    T_table_.clear();
    ForwardKinematics(q);

    // 初始化link数组
    link_d_lst_.resize(joint_num);
    link_a_lst_.resize(joint_num);
    double radius = 0.05;
    for (int i = 0; i < joint_num; i++) {
        link_d_lst_[i] = Link(radius, dh_params_table_[i][1], Link::Type::D);
        link_a_lst_[i] = Link(radius, dh_params_table_[i][2], Link::Type::A);
    }
    return;
}

std::vector<Manipulator::DHParamStruct> Manipulator::GetDHParamTable() {
    return dh_params_table_;
}

void Manipulator::ForwardKinematics(const std::vector<double>& q_in) {
    int joint_num = dh_params_table_.size();
    // T_{i}_{i-1}表示i坐标系在i-1坐标系下的姿态，同时也表示i到i-1坐标系的变换矩阵
    for (int i = 1; i <= joint_num; i++) {
        auto transform1 =
            Eigen::Translation3d(0, 0, dh_params_table_[i - 1][1]) *
            Eigen::AngleAxisd(q_in[i - 1], Eigen::Vector3d::UnitZ());
        auto transfomr2 =
            Eigen::Translation3d(dh_params_table_[i - 1][2], 0, 0) *
            Eigen::AngleAxisd(dh_params_table_[i - 1][3],
                              Eigen::Vector3d::UnitX());
        T_table_[i][i - 1] = transform1.matrix() * transfomr2.matrix();
    }
}

Manipulator::T Manipulator::GetT(int a, int b) {
    int T_num = dh_params_table_.size();
    if (a < 0 || a > T_num || b < 0 || b > T_num) {
        std::cout << "a is" << a << ", b is" << b << std::endl;
        throw std::invalid_argument("GetT: a or b out of range");
    }
    T res = T::Identity();
    if (a < b) {
        for (int i = b + 1; i <= a; i++) {
            res = res * T_table_[i][i - 1];
        }
        res = res.inverse();
    } else if (a > b) {
        for (int i = b + 1; i <= a; i++) {
            res = res * T_table_[i][i - 1];
        }
    } else {
        res = T::Identity();
    }
    return res;
}

int Manipulator::GetLinkNum() {
    return dh_params_table_.size();
}

// void Manipulator::UpdateLink() {
//     int joint_num = GetLinkNum();
//     for (int i = 1; i <= joint_num; i++) {
//         auto T_i_0 = GetT(i, 0);
//         auto T_pre_0 = GetT(i - 1, 0);
//         link_a_lst_[i - 1].GetOrientGlobal(const T& T_l_g)
//     }
// }

Link::Link(double radius, double length, Type type) {
    radius_ = radius, length_ = length + radius;
    if (type == Type::A) {
        center_local_ = {-(length) / 2, 0, 0};
        orient_local_ = Eigen::Quaterniond::FromTwoVectors(
            Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ());
    } else if (type == Type::D) {
        center_local_ = {0, 0, (length) / 2};
        orient_local_ = Eigen::Quaterniond::Identity();
    } else {
        throw std::invalid_argument("Link: type not supported");
    }
}

Eigen::Vector3d Link::GetCenterGlobal(const T& T_l_g) const {
    if (!T_l_g.matrix().allFinite())
        throw std::runtime_error("Transform contains NaN/Inf");
    Eigen::Affine3d affine = static_cast<Eigen::Affine3d>(T_l_g);
    return affine * center_local_;
}

Eigen::Quaterniond Link::GetOrientGlobal(const T& T_l_g) const {
    if (!T_l_g.matrix().allFinite())
        throw std::runtime_error("Transform contains NaN/Inf");
    Eigen::Affine3d affine = static_cast<Eigen::Affine3d>(T_l_g);
    Eigen::Matrix3d R = affine.rotation();
    Eigen::Quaterniond q = static_cast<Eigen::Quaterniond>(R);
    Eigen::Quaterniond orient = q * orient_local_;
    orient.normalize();
    return orient;
}

bool CollisionDetector::LoadObsSTLFile(std::string stl_file_path) {
    std::ifstream file(stl_file_path, std::ios::binary | std::ios::in);
    if (!file.is_open()) {
        std::cerr << "ERROR: Failed to open STL file: " << stl_file_path
                  << std::endl;
        return false;
    }
    // 跳过80字节的文件头
    file.seekg(80, std::ios::beg);
    // 读取三角形数量
    uint32_t num_triangles;
    file.read(reinterpret_cast<char*>(&num_triangles), sizeof(num_triangles));

    // 简单大小验证
    file.seekg(0, std::ios::end);
    if (file.tellg() != 84 + std::streamoff(num_triangles * 50)) {
        std::cerr << "ERROR: STL file size doesn't match triangle count"
                  << std::endl;
        return false;
    }
    file.seekg(84, std::ios::beg);  // 回到数据开始位置
    obs_triangles_.clear();
    obs_triangles_.reserve(num_triangles);
    for (uint32_t i = 0; i < num_triangles; ++i) {
        // 跳过法线向量（12字节）
        file.seekg(12, std::ios::cur);

        // 读取三个顶点（每个顶点12字节，共36字节）
        Triangle triangle;
        for (int j = 0; j < 3; ++j) {
            file.read(reinterpret_cast<char*>(&triangle[j]), sizeof(Vertex));
        }

        obs_triangles_.push_back(std::move(triangle));

        // 跳过属性字节（2字节）
        file.seekg(2, std::ios::cur);
    }
    std::cout << "Loaded " << obs_triangles_.size() << " triangles from STL"
              << std::endl;

    mesh_geometry_ptr_ = std::make_shared<fcl::BVHModel<fcl::kIOS<double>>>();
    mesh_geometry_ptr_->beginModel();
    for (const auto& triangle : obs_triangles_) {
        Eigen::Vector3d p0 = {triangle[0][0], triangle[0][1], triangle[0][2]};
        Eigen::Vector3d p1 = {triangle[1][0], triangle[1][1], triangle[1][2]};
        Eigen::Vector3d p2 = {triangle[2][0], triangle[2][1], triangle[2][2]};
        mesh_geometry_ptr_->addTriangle(p0, p1, p2);
    }
    mesh_geometry_ptr_->endModel();
    obs_stl_collison_obj_ptr_ =
        std::make_shared<fcl::CollisionObjectd>(mesh_geometry_ptr_);
    return true;
}

bool CollisionDetector::CheckCollision(const CollisionDetector::T& T_l_g,
                                       const Link& link) {
    // 圆柱体link
    auto cylinder_geometry = std::make_shared<fcl::Cylinder<double>>(
        link.GetRadius(), link.GetLength());
    Eigen::Vector3d T = link.GetCenterGlobal(T_l_g);
    Eigen::Matrix3d R = link.GetOrientGlobal(T_l_g).toRotationMatrix();
    fcl::Transform3d pose = fcl::Transform3d::Identity();
    pose.linear() = R;
    pose.translation() = T;
    fcl::CollisionObject<double> cylinder(cylinder_geometry, pose);

    // 静态障碍物
    // 采用kIOS多平面离散定向多面体
    // 记载stl时已经初始过了

    // 碰撞检测
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    fcl::collide(&cylinder, obs_stl_collison_obj_ptr_.get(), request, result);

    bool is_collision = true;
    if (result.isCollision()) {
        is_collision = true;
        std::cout << "Collision detected!" << std::endl;
    } else {
        is_collision = false;
        std::cout << "No collision detected." << std::endl;
    }

    // 距离检测
    fcl::DistanceRequestd requestd;
    fcl::DistanceResultd resultd;
    fcl::distance(&cylinder, obs_stl_collison_obj_ptr_.get(), requestd,
                  resultd);
    std::cout << "min_distance:" << resultd.min_distance << std::endl;
    return is_collision;
}

Visualizer::Visualizer(std::shared_ptr<Manipulator> manipulator_ptr,
                       std::string obs_stl_file_path) {
    /*****************************link*****************************/
    manipulator_ptr_ = manipulator_ptr;

    int JointNum = manipulator_ptr_->GetLinkNum();
    marker_lst_d_.resize(JointNum);
    marker_lst_a_.resize(JointNum);
    color_lst_.resize(JointNum);

    color_lst_[0] = {1.0, 0.0, 0.0, 1.0};
    color_lst_[1] = {0.0, 1.0, 0.0, 1.0};
    color_lst_[2] = {0.0, 0.0, 1.0, 1.0};
    color_lst_[3] = {0.0, 0.0, 0.0, 1.0};
    color_lst_[4] = {0.0, 0.0, 0.0, 1.0};
    color_lst_[5] = {0.5, 0.5, 0.5, 1.0};

    for (int i = 0; i < JointNum; i++) {
        // 起点在i-1坐标系，轴向为z_{i-1}轴长度为d
        auto& marker_d = marker_lst_d_[i];
        const auto& link_d = manipulator_ptr_->link_d_lst_[i];
        marker_d.scale.x = marker_d.scale.y = link_d.GetRadius();
        marker_d.scale.z = link_d.GetLength();
        marker_d.color.r = color_lst_[i][0];
        marker_d.color.g = color_lst_[i][1];
        marker_d.color.b = color_lst_[i][2];
        marker_d.color.a = color_lst_[i][3];
        // 终点在i坐标系，轴向为x_{i}轴长度为a
        auto& marker_a = marker_lst_a_[i];
        const auto& link_a = manipulator_ptr_->link_a_lst_[i];
        marker_a.scale.x = marker_a.scale.y = link_a.GetRadius();
        marker_a.scale.z = link_a.GetLength();
        marker_a.color.r = color_lst_[i][0];
        marker_a.color.g = color_lst_[i][1];
        marker_a.color.b = color_lst_[i][2];
        marker_a.color.a = color_lst_[i][3];
    }
    /*****************************obs*****************************/
    marker_obs_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_obs_.mesh_resource = "file://" + obs_stl_file_path;
    marker_obs_.pose.position.x = 0, marker_obs_.pose.position.y = 0,
    marker_obs_.pose.position.z = 0;
    marker_obs_.pose.orientation.x = 0, marker_obs_.pose.orientation.y = 0,
    marker_obs_.pose.orientation.z = 0, marker_obs_.pose.orientation.w = 1;
    marker_obs_.scale.x = 1, marker_obs_.scale.y = 1, marker_obs_.scale.z = 1;
    marker_obs_.mesh_use_embedded_materials = true;
    marker_obs_.color.r = 0.8f;
    marker_obs_.color.g = 0.2f;
    marker_obs_.color.b = 0.2f;
    marker_obs_.color.a = 1.0f;
}

void Visualizer::Update(const T& T_i_0, const T& T_pre_0, int i) {
    try {
        /***************************更新marker_d*********************************/
        auto& marker_d = marker_lst_d_[i - 1];
        // 位置
        auto center_d =
            manipulator_ptr_->link_d_lst_[i - 1].GetCenterGlobal(T_pre_0);
        marker_d.pose.position.x = center_d[0],
        marker_d.pose.position.y = center_d[1],
        marker_d.pose.position.z = center_d[2];
        // 朝向
        auto orient_d =
            manipulator_ptr_->link_d_lst_[i - 1].GetOrientGlobal(T_pre_0);
        marker_d.pose.orientation.x = orient_d.x(),
        marker_d.pose.orientation.y = orient_d.y(),
        marker_d.pose.orientation.z = orient_d.z(),
        marker_d.pose.orientation.w = orient_d.w();
        /***************************更新marker_a*********************************/
        auto& marker_a = marker_lst_a_[i - 1];
        // 位置
        auto center_a =
            manipulator_ptr_->link_a_lst_[i - 1].GetCenterGlobal(T_i_0);
        marker_a.pose.position.x = center_a[0],
        marker_a.pose.position.y = center_a[1],
        marker_a.pose.position.z = center_a[2];
        // 朝向
        auto orient_a =
            manipulator_ptr_->link_a_lst_[i - 1].GetOrientGlobal(T_i_0);
        marker_a.pose.orientation.x = orient_a.x(),
        marker_a.pose.orientation.y = orient_a.y(),
        marker_a.pose.orientation.z = orient_a.z(),
        marker_a.pose.orientation.w = orient_a.w();
    } catch (const std::exception& e) {
        std::cout << "Error in Update: " << e.what() << std::endl;
    }
}

void Visualizer::GetMarkerLst(
    std::vector<visualization_msgs::msg::Marker>& marker_lst) {
    marker_lst.clear();
    marker_lst.reserve(marker_lst_d_.size() + marker_lst_a_.size());

    auto itd = marker_lst_d_.begin();
    auto ita = marker_lst_a_.begin();

    while (itd != marker_lst_d_.end() && ita != marker_lst_a_.end()) {
        marker_lst.push_back(*itd);  // 先添加来自 marker_lst_d_ 的元素
        marker_lst.push_back(*ita);  // 再添加来自 marker_lst_a_ 的元素
        ++itd;
        ++ita;
    }

    marker_lst.insert(marker_lst.end(), itd, marker_lst_d_.end());
    marker_lst.insert(marker_lst.end(), ita, marker_lst_a_.end());
}

void Visualizer::GetObsMarker(visualization_msgs::msg::Marker& marker_obs) {
    marker_obs = marker_obs_;
}