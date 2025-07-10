#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "manipulator.hpp"

using namespace std::chrono_literals;

class MainNode : public rclcpp::Node {
   public:
    using PubType =
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
    MainNode();

   private:
    void FlushPub();
    void Show();
    void SubCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // 机械臂相关
    std::shared_ptr<Manipulator> manipulator_ptr_;
    // 可视化相关
    std::shared_ptr<Visualizer> visualizer_ptr_;
    // 碰撞检测相关
    std::shared_ptr<CollisionDetector> collision_detector_ptr_;

    // ros2相关
    rclcpp::TimerBase::SharedPtr timer_;
    std::string pub_topic_name_ = "/visualization_marker_array";
    std::string subs_topic_name_ = "/joint_states";
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        pub_;  // 发送可视化markder
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        sub_;  // 接收机械臂关节角度值, 单位度
};

MainNode::MainNode() : Node("MainNode") {
    // 初始化DH参数
    // 传统DH表:theta, d, a, alpha, 转轴分别是x_i和z_{i-1}角度采用弧度制
    std::vector<Manipulator::DHParamStruct> dh_params_table;
    dh_params_table.resize(6);
    dh_params_table[0] = {0.0, 0.1295, 0.0, M_PI / 2};
    dh_params_table[1] = {0.0, 0.1195, 0.414, M_PI};
    dh_params_table[2] = {0.0, 0.1195, 0.264, M_PI / 2};
    dh_params_table[3] = {0.0, 0.0, 0.0, 0};
    dh_params_table[4] = {0.0, 0.0, 0.0, -M_PI / 2};
    dh_params_table[5] = {0.0, 0.0, 0.0, 0};

    manipulator_ptr_ = std::make_shared<Manipulator>();
    manipulator_ptr_->SetDHParamTable(dh_params_table);

    std::string obs_stl_file_path =
        "/home/test/桌面/collison_detection/src/collison_visualize/stl/"
        "obs_m.STL";
    visualizer_ptr_ =
        std::make_shared<Visualizer>(manipulator_ptr_, obs_stl_file_path);

    collision_detector_ptr_ = std::make_shared<CollisionDetector>();
    collision_detector_ptr_->LoadObsSTLFile(obs_stl_file_path);

    // 创建可视化关节话题发布者
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        pub_topic_name_, 10);

    // 创建机械臂关节角度话题接收者
    sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        subs_topic_name_, 10,
        std::bind(&MainNode::SubCallback, this, std::placeholders::_1));

    // 正运动学解算
    manipulator_ptr_->ForwardKinematics({0, 0, 0, 0, 0, 0});

    // 更新visualizer
    for (int i = 1; i <= manipulator_ptr_->GetLinkNum(); i++) {
        auto T_i_0 = manipulator_ptr_->GetT(i, 0);
        auto T_pre_0 = manipulator_ptr_->GetT(i - 1, 0);
        visualizer_ptr_->Update(T_i_0, T_pre_0, i);
    }

    // 创建定时器，定时器内定时刷新发布话题
    timer_ =
        this->create_wall_timer(500ms, std::bind(&MainNode::FlushPub, this));
}

void MainNode::FlushPub() {
    // link关节
    std::vector<visualization_msgs::msg::Marker> marker_lst;
    visualizer_ptr_->GetMarkerLst(marker_lst);
    auto marker_array = visualization_msgs::msg::MarkerArray();
    int id = 0;
    for (auto& marker : marker_lst) {
        marker.header.frame_id = "base_link";
        marker.ns = "marker_array";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker_array.markers.push_back(marker);
    }
    // obstacle障碍物
    visualization_msgs::msg::Marker marker_obs;
    visualizer_ptr_->GetObsMarker(marker_obs);
    marker_obs.header.frame_id = "base_link";
    marker_obs.ns = "obs", marker_obs.id = 0;
    marker_obs.action = visualization_msgs::msg::Marker::ADD;
    marker_array.markers.push_back(marker_obs);

    pub_->publish(marker_array);
}

void MainNode::SubCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    const std::vector<double>& doubleArray = msg->data;
    // 打印数组内容
    std::ostringstream oss;
    oss << "收到数组如下 [ ";
    for (double val : doubleArray) {
        oss << std::fixed << std::setprecision(6) << val << " ";
    }
    oss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

    // 只取前3个关节
    if (doubleArray.size() < 3) {
        RCLCPP_ERROR(this->get_logger(), "数组长度小于3");
        return;
    }
    std::vector<double> q_lst = {
        doubleArray[0], doubleArray[1], doubleArray[2], 0, 0, 0};
    for (auto& q : q_lst) {
        q = q * M_PI / 180.0;  // 角度转弧度
    }

    // 正运动学解算
    manipulator_ptr_->ForwardKinematics(q_lst);

    // 更新visualizer
    for (int i = 1; i <= manipulator_ptr_->GetLinkNum(); i++) {
        auto T_i_0 = manipulator_ptr_->GetT(i, 0);
        auto T_pre_0 = manipulator_ptr_->GetT(i - 1, 0);
        visualizer_ptr_->Update(T_i_0, T_pre_0, i);
    }

    // 碰撞检测
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 1; i <= manipulator_ptr_->GetLinkNum(); i++) {
        // 基座和障碍物不用判断，因为可能固连
        if (i == 1)
            continue;
        auto T_i_0 = manipulator_ptr_->GetT(i, 0);
        auto T_pre_0 = manipulator_ptr_->GetT(i - 1, 0);
        if (collision_detector_ptr_->CheckCollision(
                T_pre_0, manipulator_ptr_->link_d_lst_[i - 1])) {
            RCLCPP_ERROR(this->get_logger(), "第%i个link, type D, 碰撞发生", i);
        }
        if (collision_detector_ptr_->CheckCollision(
                T_i_0, manipulator_ptr_->link_a_lst_[i - 1])) {
            RCLCPP_ERROR(this->get_logger(), "第%i个link, type A, 碰撞发生", i);
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    RCLCPP_INFO(this->get_logger(), "碰撞检测耗时: %f ms",
                duration.count() / 1e6);
    FlushPub();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainNode>());
    rclcpp::shutdown();
    return 0;
}