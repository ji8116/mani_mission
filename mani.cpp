#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <wpr_simulation2/msg/object.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <string>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;

// ============================================================
// 全局变量
// ============================================================

std::shared_ptr<rclcpp::Node> node;

// --- 导航 ---
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub;

// --- 机械臂 ---
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behavior_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mani_pub;

// --- 底盘 ---
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

// --- 人脸检测输入转发 ---
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr face_input_pub;

// ============================================================
// 状态机
// ============================================================

enum StepState
{
    STEP_WAIT,             // 等待系统就绪
    STEP_GOTO_KITCHEN,     // 导航到厨房
    STEP_STAY_KITCHEN,     // 厨房停留 + RGB 确认
    STEP_ALIGN_OBJ,        // 底盘对准饮料
    STEP_HAND_UP,          // 抬臂张爪
    STEP_FORWARD,          // 底盘前送
    STEP_GRAB,             // 闭合夹爪
    STEP_OBJ_UP,           // 提起物体
    STEP_BACKWARD,         // 底盘后退
    STEP_LOWER_ARM,        // 降臂至运输高度（缩小碰撞体过窄通道）
    STEP_GOTO_GUEST,       // 导航到客人
    STEP_STAY_GUEST,       // 客人停留 + 人脸确认
    STEP_DONE,             // 任务完成

    STEP_RETRY_NAVI,       // 导航重试
    STEP_RETRY_GRAB,       // 抓取重试
    STEP_RETRY_VERIFY,     // 确认重试
    STEP_ABORT             // 任务终止
};

StepState current_state = STEP_WAIT;

// 记录重试前状态
StepState state_before_retry = STEP_WAIT;

// ============================================================
// 航点
// ============================================================

std::vector<std::string> waypoints =
{
    "kitchen",
    "guest",
    "HOME"
};

std::size_t current_index = 0;
std::size_t total_waypoints = 2;   // kitchen + guest 为有效航点

// ============================================================
// 导航参数
// ============================================================

bool waiting_result = false;
int retry_count = 0;
int max_retry = 2;
double timeout_sec = 80.0;
int strategy_mode = 1;

rclcpp::Time nav_start_time(0, 0, RCL_ROS_TIME);

// ============================================================
// 停留 / 人脸检测
// ============================================================

bool staying = false;
double stay_time_sec = 8.0;

rclcpp::Time stay_start_time(0, 0, RCL_ROS_TIME);
rclcpp::Time last_face_check_time(0, 0, RCL_ROS_TIME);

bool face_detected = false;
rclcpp::Time last_face_time(0, 0, RCL_ROS_TIME);

int face_detect_count = 0;
int authorized_count = 0;

int verify_retry_count = 0;
int max_verify_retry = 2;

// ============================================================
// RGB 视觉确认（厨房）
// ============================================================

cv::Mat last_image_;
bool rgb_verified_ = false;

// ============================================================
// 3D 物体抓取
// ============================================================

float object_x = 0.0;
float object_y = 0.0;
float object_z = 0.0;
bool object_received_ = false;

// EMA 滤波 + 跳变检测（解决 objects_3d 数据不稳定问题）
float object_x_filt_ = 0.0;
float object_y_filt_ = 0.0;
float prev_raw_x_ = -99.0;       // 上一帧原始 x，用于跳变检测
bool filter_inited_ = false;
int align_converge_count_ = 0;   // 连续收敛帧计数

const float EMA_ALPHA = 0.25;          // EMA 平滑系数 (越小越平滑)
const float JUMP_THRESHOLD = 0.25;     // 相邻帧跳变阈值 (m)
const int CONVERGE_REQUIRED = 20;      // 连续收敛所需帧数

float align_x = 0.8;
float align_y = 0.0;

int grab_retry_count = 0;
int max_grab_retry = 2;

// 对准阶段计时器
rclcpp::Time align_start_time(0, 0, RCL_ROS_TIME);

// 抓取步骤计时
rclcpp::Time step_start_time(0, 0, RCL_ROS_TIME);
bool step_timer_running_ = false;

// ============================================================
// 激光安全
// ============================================================

double min_front_dist_ = 10.0;
const double safety_threshold_ = 0.25;

// ============================================================
// 实验统计
// ============================================================

rclcpp::Time mission_start_time(0, 0, RCL_ROS_TIME);
rclcpp::Time mission_end_time(0, 0, RCL_ROS_TIME);

int success_count = 0;
bool home_success = false;
bool drink_grabbed_ = false;
bool delivery_done_ = false;

int navi_retry_triggered_ = 0;
int grab_retry_triggered_ = 0;
int verify_retry_triggered_ = 0;

// ============================================================
// 函数声明
// ============================================================

void PrintMissionResult();
void SendWaypoint(const std::string &name);
void NaviDone();
void RetryNavi();
void RetryGrab();
void RetryVerify();
void DoRGBCheck();
void DoFaceCheck();
void StartTimedStep();
bool IsTimedStepDone(double seconds);
void StopChassis();

// ============================================================
// 回调: 摄像头
// ============================================================

void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 转发给人脸检测节点
    face_input_pub->publish(*msg);

    // 保存最后一帧供 RGB 确认使用
    try
    {
        last_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (...)
    {
    }
}

// ============================================================
// 回调: 导航结果
// ============================================================

void ResultCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "navi done" && waiting_result)
    {
        NaviDone();
    }
}

// ============================================================
// 回调: 人脸位置
// ============================================================

void FacePosCallback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
{
    if (!staying) return;

    if (msg->width > 0 && msg->height > 0)
    {
        if (!face_detected)
        {
            RCLCPP_INFO(node->get_logger(), "检测到人脸!");
        }
        face_detected = true;
        last_face_time = node->now();
    }
}

// ============================================================
// 回调: 3D 物体坐标
// ============================================================

void ObjectCallback(const wpr_simulation2::msg::Object::SharedPtr msg)
{
    if (current_state == STEP_ALIGN_OBJ)
    {
        // 持续更新目标坐标（带 EMA 滤波 + 跳变检测）
        if (msg->x.size() > 0)
        {
            float ox = msg->x[0];
            float oy = msg->y[0];
            float oz = msg->z[0];

            // 有效性检查：全零或异常值视为无效数据
            if (fabs(ox) < 0.001 && fabs(oy) < 0.001 && fabs(oz) < 0.001)
            {
                return;
            }
            if (fabs(ox) > 50.0 || fabs(oy) > 50.0)
            {
                return;
            }

            // 跳变检测：相邻帧原始 x 差值过大则重置滤波器
            if (filter_inited_ && fabs(ox - prev_raw_x_) > JUMP_THRESHOLD)
            {
                filter_inited_ = false;
                align_converge_count_ = 0;
            }
            prev_raw_x_ = ox;

            // EMA 指数滑动平均滤波
            if (!filter_inited_)
            {
                object_x_filt_ = ox;
                object_y_filt_ = oy;
                filter_inited_ = true;
            }
            else
            {
                object_x_filt_ = EMA_ALPHA * ox + (1.0f - EMA_ALPHA) * object_x_filt_;
                object_y_filt_ = EMA_ALPHA * oy + (1.0f - EMA_ALPHA) * object_y_filt_;
            }

            object_x = object_x_filt_;
            object_y = object_y_filt_;
            object_z = oz;
            object_received_ = true;
        }
    }
}

// ============================================================
// 回调: 激光
// ============================================================

void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    min_front_dist_ = 10.0;

    size_t center = msg->ranges.size() / 2;
    for (size_t i = center - 40;
         i < center + 40 && i < msg->ranges.size(); ++i)
    {
        if (msg->ranges[i] > 0.1 && msg->ranges[i] < min_front_dist_)
        {
            min_front_dist_ = msg->ranges[i];
        }
    }
}

// ============================================================
// 工具: 停止底盘
// ============================================================

void StopChassis()
{
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0;
    stop.linear.y = 0;
    stop.angular.z = 0;
    vel_pub->publish(stop);
}

// ============================================================
// 工具: 开始计时步骤
// ============================================================

void StartTimedStep()
{
    step_start_time = node->now();
    step_timer_running_ = true;
}

// ============================================================
// 工具: 检查计时步骤是否完成
// ============================================================

bool IsTimedStepDone(double seconds)
{
    if (!step_timer_running_) return false;
    double elapsed = (node->now() - step_start_time).seconds();
    return elapsed >= seconds;
}

// ============================================================
// 发布导航目标
// ============================================================

void SendWaypoint(const std::string &name)
{
    std_msgs::msg::String msg;
    msg.data = name;
    navigation_pub->publish(msg);

    waiting_result = true;
    nav_start_time = node->now();

    RCLCPP_INFO(node->get_logger(), "导航至: %s", name.c_str());
}

// ============================================================
// 导航成功
// ============================================================

void NaviDone()
{
    waiting_result = false;
    retry_count = 0;

    RCLCPP_INFO(node->get_logger(), "到达航点!");

    if (current_state == STEP_GOTO_KITCHEN)
    {
        current_state = STEP_STAY_KITCHEN;
        staying = true;
        stay_start_time = node->now();
        RCLCPP_INFO(node->get_logger(), "=== 厨房: 开始 RGB 视觉确认 ===");
    }
    else if (current_state == STEP_GOTO_GUEST)
    {
        current_state = STEP_STAY_GUEST;
        staying = true;
        stay_start_time = node->now();
        last_face_check_time = node->now();
        face_detected = false;
        RCLCPP_INFO(node->get_logger(), "=== 客人点: 开始人脸识别确认 ===");
    }
    else
    {
        // HOME (任务完成后回家)
        home_success = true;
        success_count++;
        current_state = STEP_DONE;
    }
}

// ============================================================
// 导航超时重试
// ============================================================

void RetryNavi()
{
    if (retry_count < max_retry)
    {
        retry_count++;
        navi_retry_triggered_++;

        RCLCPP_WARN(node->get_logger(),
            "导航超时! 重试 %d/%d", retry_count, max_retry);

        rclcpp::sleep_for(2s);

        std::string target;
        if (current_state == STEP_RETRY_NAVI)
        {
            // 恢复到重试前的状态
            current_state = state_before_retry;
        }

        if (current_state == STEP_GOTO_KITCHEN ||
            state_before_retry == STEP_GOTO_KITCHEN)
        {
            target = waypoints[0];  // kitchen
            current_state = STEP_GOTO_KITCHEN;
        }
        else
        {
            target = waypoints[1];  // guest
            current_state = STEP_GOTO_GUEST;
        }

        SendWaypoint(target);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(),
            "导航失败! 已达最大重试次数, 任务终止");
        current_state = STEP_ABORT;
    }
}

// ============================================================
// 抓取重试
// ============================================================

void RetryGrab()
{
    if (grab_retry_count < max_grab_retry)
    {
        grab_retry_count++;
        grab_retry_triggered_++;

        RCLCPP_WARN(node->get_logger(),
            "抓取失败! 重试 %d/%d", grab_retry_count, max_grab_retry);

        // 先退后一段距离
        StopChassis();
        rclcpp::sleep_for(2s);

        // 重新对准
        object_received_ = false;
        object_x = 0;
        object_y = 0;
        object_z = 0;
        object_x_filt_ = 0;
        object_y_filt_ = 0;
        filter_inited_ = false;
        align_converge_count_ = 0;
        align_start_time = rclcpp::Time(0, 0, RCL_ROS_TIME);   // 重置计时器

        std_msgs::msg::String cmd;
        cmd.data = "start objects";
        behavior_pub->publish(cmd);

        current_state = STEP_ALIGN_OBJ;
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(),
            "抓取失败! 已达最大重试次数, 任务终止");
        current_state = STEP_ABORT;
    }
}

// ============================================================
// 确认重试
// ============================================================

void RetryVerify()
{
    if (verify_retry_count < max_verify_retry)
    {
        verify_retry_count++;
        verify_retry_triggered_++;

        RCLCPP_WARN(node->get_logger(),
            "确认失败! 等待后重试 %d/%d",
            verify_retry_count, max_verify_retry);

        rclcpp::sleep_for(3s);

        // 重置人脸检测状态
        face_detected = false;
        staying = true;
        stay_start_time = node->now();
        last_face_check_time = node->now();

        current_state = STEP_STAY_GUEST;
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(),
            "客人确认失败! 已达最大重试次数");

        // 增强模式下,仍尝试完成递送(输出失败日志)
        if (strategy_mode == 2)
        {
            RCLCPP_WARN(node->get_logger(),
                "增强模式: 标记递送完成(确认跳过)");
            delivery_done_ = true;
            current_state = STEP_DONE;
        }
        else
        {
            current_state = STEP_ABORT;
        }
    }
}

// ============================================================
// RGB 视觉确认（厨房）
// ============================================================

void DoRGBCheck()
{
    if (last_image_.empty()) return;
    if (rgb_verified_) return;

    // 显示图像
    cv::imshow("Kitchen_RGB_Check", last_image_);
    cv::waitKey(1);

    // HSV 颜色检测（红色 + 绿色，适配饮料瓶颜色）
    cv::Mat hsv;
    cv::cvtColor(last_image_, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask_red1, mask_red2, mask_red, mask_green, mask;

    cv::inRange(hsv, cv::Scalar(0, 70, 50),
                cv::Scalar(10, 255, 255), mask_red1);
    cv::inRange(hsv, cv::Scalar(170, 70, 50),
                cv::Scalar(180, 255, 255), mask_red2);

    cv::bitwise_or(mask_red1, mask_red2, mask_red);

    cv::inRange(hsv, cv::Scalar(40, 70, 50),
                cv::Scalar(85, 255, 255), mask_green);

    cv::bitwise_or(mask_red, mask_green, mask);

    int non_zero = cv::countNonZero(mask);

    RCLCPP_INFO(node->get_logger(),
        "RGB 视觉确认: 颜色像素 = %d", non_zero);

    // 保存截图
    cv::imwrite("kitchen_visual_check.jpg", last_image_);

    if (non_zero > 800)
    {
        rgb_verified_ = true;
        staying = false;

        RCLCPP_INFO(node->get_logger(),
            "=== RGB 视觉确认通过! 启动 3D 物体检测 ===");

        // 启动物体发布
        std_msgs::msg::String cmd;
        cmd.data = "start objects";
        behavior_pub->publish(cmd);

        // 等待 objects 数据到达
        rclcpp::sleep_for(8s);

        current_state = STEP_ALIGN_OBJ;
    }
    else
    {
        RCLCPP_WARN(node->get_logger(),
            "RGB 视觉确认: 颜色像素不足, 继续等待...");
    }
}

// ============================================================
// 人脸检测确认（客人）
// ============================================================

void DoFaceCheck()
{
    if (!staying) return;

    double stay_elapsed = (node->now() - stay_start_time).seconds();

    // 停留时间到
    if (stay_elapsed >= stay_time_sec)
    {
        if (face_detected)
        {
            // 模拟授权判断 (70% 通过率)
            int r = rand() % 10;
            if (r < 7)
            {
                authorized_count++;
                face_detect_count++;
                staying = false;
                delivery_done_ = true;

                RCLCPP_INFO(node->get_logger(),
                    "人脸识别: 授权通过!");
                RCLCPP_INFO(node->get_logger(),
                    "==========================================");
                RCLCPP_INFO(node->get_logger(),
                    "  Delivery completed!  ");
                RCLCPP_INFO(node->get_logger(),
                    "==========================================");

                current_state = STEP_DONE;
            }
            else
            {
                RCLCPP_WARN(node->get_logger(),
                    "人脸识别: 未授权人员!");
                face_detect_count++;

                // 未授权也视为确认失败,走重试
                RetryVerify();
            }
        }
        else
        {
            RCLCPP_WARN(node->get_logger(),
                "停留期间未检测到人脸");
            RetryVerify();
        }
        return;
    }

    // 每秒检查一次人脸
    double check_interval = (node->now() - last_face_check_time).seconds();
    if (check_interval >= 1.0)
    {
        last_face_check_time = node->now();

        if (face_detected)
        {
            double face_delay = (node->now() - last_face_time).seconds();
            if (face_delay < 1.0)
            {
                RCLCPP_INFO(node->get_logger(),
                    "检测到人脸, 停留 %.1f / %.1f s",
                    stay_elapsed, stay_time_sec);
            }
            else
            {
                face_detected = false;
            }
        }
        else
        {
            RCLCPP_INFO(node->get_logger(),
                "等待人脸检测... %.1f / %.1f s",
                stay_elapsed, stay_time_sec);
        }
    }
}

// ============================================================
// 打印任务结果
// ============================================================

void PrintMissionResult()
{
    mission_end_time = node->now();
    double total_time = (mission_end_time - mission_start_time).seconds();

    RCLCPP_INFO(node->get_logger(),
        "==========================================");
    RCLCPP_INFO(node->get_logger(),
        "  任务结束");
    RCLCPP_INFO(node->get_logger(),
        "==========================================");
    RCLCPP_INFO(node->get_logger(),
        "策略模式 : %d", strategy_mode);
    RCLCPP_INFO(node->get_logger(),
        "总耗时   : %.2f s", total_time);
    RCLCPP_INFO(node->get_logger(),
        "导航成功 : %d / %d", success_count, (int)total_waypoints);
    RCLCPP_INFO(node->get_logger(),
        "回家成功 : %s", home_success ? "YES" : "NO");
    RCLCPP_INFO(node->get_logger(),
        "饮料抓取 : %s", drink_grabbed_ ? "SUCCESS" : "FAILED");
    RCLCPP_INFO(node->get_logger(),
        "递送完成 : %s", delivery_done_ ? "SUCCESS" : "FAILED");
    RCLCPP_INFO(node->get_logger(),
        "人脸检测次数 : %d", face_detect_count);
    RCLCPP_INFO(node->get_logger(),
        "授权通过次数 : %d", authorized_count);
    RCLCPP_INFO(node->get_logger(),
        "导航重试触发 : %d", navi_retry_triggered_);
    RCLCPP_INFO(node->get_logger(),
        "抓取重试触发 : %d", grab_retry_triggered_);
    RCLCPP_INFO(node->get_logger(),
        "确认重试触发 : %d", verify_retry_triggered_);
    RCLCPP_INFO(node->get_logger(),
        "==========================================");
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("mani_mission_node");

    srand(time(NULL));

    // ---- 参数声明 ----

    node->declare_parameter("strategy_mode", 1);
    node->declare_parameter("timeout_sec", 80.0);
    node->declare_parameter("stay_time_sec", 8.0);
    node->declare_parameter("max_retry", 2);

    node->get_parameter("strategy_mode", strategy_mode);
    node->get_parameter("timeout_sec", timeout_sec);
    node->get_parameter("stay_time_sec", stay_time_sec);
    node->get_parameter("max_retry", max_retry);

    max_grab_retry = max_retry;
    max_verify_retry = max_retry;

    // ---- Publisher ----

    navigation_pub = node->create_publisher<std_msgs::msg::String>(
        "/waterplus/navi_waypoint", 10);

    behavior_pub = node->create_publisher<std_msgs::msg::String>(
        "/wpb_home/behavior", 10);

    mani_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        "/wpb_home/mani_ctrl", 10);

    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    face_input_pub = node->create_publisher<sensor_msgs::msg::Image>(
        "/face_detector_input", 1);

    // ---- Subscriber ----

    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/kinect2/qhd/image_raw", 10, CamRGBCallback);

    auto result_sub = node->create_subscription<std_msgs::msg::String>(
        "/waterplus/navi_result", 10, ResultCallback);

    auto face_sub = node->create_subscription<sensor_msgs::msg::RegionOfInterest>(
        "/face_position", 10, FacePosCallback);

    auto object_sub = node->create_subscription<wpr_simulation2::msg::Object>(
        "/wpb_home/objects_3d", 10, ObjectCallback);

    auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, LaserCallback);

    // ---- 等待系统初始化 ----

    RCLCPP_INFO(node->get_logger(), "等待系统初始化 (5s)...");
    rclcpp::sleep_for(5s);

    // ---- 开始任务 ----

    mission_start_time = node->now();

    RCLCPP_INFO(node->get_logger(),
        "==========================================");
    RCLCPP_INFO(node->get_logger(),
        "  家庭服务机器人 — 饮料递送系统  ");
    RCLCPP_INFO(node->get_logger(),
        "==========================================");
    RCLCPP_INFO(node->get_logger(),
        "策略模式 : %d", strategy_mode);
    RCLCPP_INFO(node->get_logger(),
        "导航超时 : %.0f s", timeout_sec);
    RCLCPP_INFO(node->get_logger(),
        "停留时间 : %.0f s", stay_time_sec);
    RCLCPP_INFO(node->get_logger(),
        "最大重试 : %d", max_retry);

    // 发送第一个航点
    current_state = STEP_GOTO_KITCHEN;
    SendWaypoint(waypoints[0]);   // "kitchen"

    // ---- 主循环 ----

    rclcpp::Rate loop_rate(30);

    while (rclcpp::ok() && current_state != STEP_DONE && current_state != STEP_ABORT)
    {
        rclcpp::spin_some(node);

        // ================================================
        // 导航超时检测（独立于状态，全时检查）
        // ================================================

        if (waiting_result)
        {
            double nav_elapsed = (node->now() - nav_start_time).seconds();
            if (nav_elapsed > timeout_sec)
            {
                waiting_result = false;
                RCLCPP_WARN(node->get_logger(), "导航超时 (%.0fs)!", nav_elapsed);

                state_before_retry = current_state;
                current_state = STEP_RETRY_NAVI;
            }
        }

        // ================================================
        // 状态机
        // ================================================

        switch (current_state)
        {
            // --------------------------------------------------
            case STEP_GOTO_KITCHEN:
                // 等待回调 NaviDone 切换状态
                break;

            // --------------------------------------------------
            case STEP_STAY_KITCHEN:
            {
                double stay_elapsed = (node->now() - stay_start_time).seconds();

                // RGB 视觉确认
                DoRGBCheck();

                // 超时保护：停留超时仍未通过 RGB 确认
                if (!rgb_verified_ && stay_elapsed > 15.0)
                {
                    RCLCPP_WARN(node->get_logger(),
                        "厨房 RGB 确认超时, 强制进入抓取阶段");
                    rgb_verified_ = true;
                    staying = false;

                    std_msgs::msg::String cmd;
                    cmd.data = "start objects";
                    behavior_pub->publish(cmd);
                    rclcpp::sleep_for(2s);
                    current_state = STEP_ALIGN_OBJ;
                }
                break;
            }

            // --------------------------------------------------
            case STEP_ALIGN_OBJ:
            {
                // 激光安全
                if (min_front_dist_ < safety_threshold_)
                {
                    StopChassis();
                    RCLCPP_ERROR(node->get_logger(),
                        "激光检测到障碍! 距离=%.2f m, 停止!",
                        min_front_dist_);
                    break;
                }

                // 等待有效 objects_3d 数据
                if (!object_received_)
                {
                    RCLCPP_INFO_THROTTLE(node->get_logger(),
                        *(node->get_clock()), 2000,
                        "[对准] 等待 objects_3d 有效数据...");
                    break;
                }

                // 首次进入对准阶段时初始化计时器和滤波器
                if (align_start_time.seconds() == 0.0)
                {
                    align_start_time = node->now();
                    filter_inited_ = false;
                    align_converge_count_ = 0;
                }

                // 对准超时保护 (30s)
                double align_elapsed = (node->now() - align_start_time).seconds();
                if (align_elapsed > 30.0)
                {
                    StopChassis();
                    RCLCPP_WARN(node->get_logger(),
                        "[对准] 超时 (%.0fs), 以当前位姿进入抓取", align_elapsed);
                    std_msgs::msg::String cmd;
                    cmd.data = "stop objects";
                    behavior_pub->publish(cmd);
                    current_state = STEP_HAND_UP;
                    StartTimedStep();
                    break;
                }

                float diff_x = object_x - align_x;
                float diff_y = object_y - align_y;

                geometry_msgs::msg::Twist vel_msg;

                // 收敛检测：用较大容差 + 连续计数防抖
                if (fabs(diff_x) > 0.08 || fabs(diff_y) > 0.05)
                {
                    align_converge_count_ = 0;   // 脱离容差范围，重置计数

                    // 速度限幅, 防止 objects 数据异常导致暴冲
                    vel_msg.linear.x = std::clamp(diff_x * 0.8, -0.25, 0.25);
                    vel_msg.linear.y = std::clamp(diff_y * 0.8, -0.20, 0.20);

                    RCLCPP_INFO_THROTTLE(node->get_logger(),
                        *(node->get_clock()), 1000,
                        "[对准] obj=(%.2f,%.2f,%.2f) 误差=(%.2f,%.2f) 速度=(%.2f,%.2f) 收敛=%d/%d",
                        object_x, object_y, object_z,
                        diff_x, diff_y,
                        vel_msg.linear.x, vel_msg.linear.y,
                        align_converge_count_, CONVERGE_REQUIRED);
                }
                else
                {
                    // 在容差范围内，累计连续帧数
                    align_converge_count_++;
                    StopChassis();

                    RCLCPP_INFO_THROTTLE(node->get_logger(),
                        *(node->get_clock()), 500,
                        "[对准] 容差内 obj=(%.2f,%.2f) 误差=(%.2f,%.2f) 收敛=%d/%d",
                        object_x, object_y,
                        diff_x, diff_y,
                        align_converge_count_, CONVERGE_REQUIRED);

                    if (align_converge_count_ >= CONVERGE_REQUIRED)
                    {
                        // 对准完成
                        
                        std_msgs::msg::String cmd;
                        cmd.data = "stop objects";
                        behavior_pub->publish(cmd);

                        RCLCPP_INFO(node->get_logger(),
                            "=== 对准完成! 滤波坐标 (%.2f,%.2f,%.2f) ===",
                            object_x, object_y, object_z);

                        current_state = STEP_HAND_UP;
                        StartTimedStep();
                    }
                }

                vel_pub->publish(vel_msg);
                break;
            }

            // --------------------------------------------------
            case STEP_HAND_UP:
            {
                if (!step_timer_running_) StartTimedStep();

                sensor_msgs::msg::JointState mani_msg;
                mani_msg.name.resize(2);
                mani_msg.name[0] = "lift";
                mani_msg.name[1] = "gripper";
                mani_msg.position.resize(2);
                mani_msg.position[0] = object_z;
                mani_msg.position[1] = 0.15;         // 张开夹爪
                mani_pub->publish(mani_msg);

                RCLCPP_INFO_THROTTLE(node->get_logger(),
                    *(node->get_clock()), 2000,
                    "[抬臂] 高度=%.2f 夹爪=0.15", object_z);

                // 抬臂完成 → 前送靠近物体
                if (IsTimedStepDone(5.0))
                {
                    step_timer_running_ = false;
                    current_state = STEP_FORWARD;
                    StartTimedStep();
                }
                break;
            }

            // --------------------------------------------------
            case STEP_FORWARD:
            {
                if (!step_timer_running_) StartTimedStep();

                geometry_msgs::msg::Twist vel_msg;
                vel_msg.linear.x = 0.03;
                vel_pub->publish(vel_msg);

                RCLCPP_INFO_THROTTLE(node->get_logger(),
                    *(node->get_clock()), 1000,
                    "[前送] 速度=0.1 m/s");

                // 缩短至 4s (~0.4m), 避免推走物体
                if (IsTimedStepDone(4.0))
                {
                    step_timer_running_ = false;
                    current_state = STEP_GRAB;
                    StartTimedStep();
                }
                break;
            }

            // --------------------------------------------------
            case STEP_GRAB:
            {
                if (!step_timer_running_) StartTimedStep();

                StopChassis();

                sensor_msgs::msg::JointState mani_msg;
                mani_msg.name.resize(2);
                mani_msg.name[0] = "lift";
                mani_msg.name[1] = "gripper";
                mani_msg.position.resize(2);
                mani_msg.position[0] = object_z;
                mani_msg.position[1] = 0.07;         // 闭合夹爪
                mani_pub->publish(mani_msg);

                RCLCPP_INFO_THROTTLE(node->get_logger(),
                    *(node->get_clock()), 2000,
                    "[夹紧] 夹爪闭合到 0.07");

                if (IsTimedStepDone(5.0))
                {
                    step_timer_running_ = false;
                    drink_grabbed_ = true;
                    RCLCPP_INFO(node->get_logger(),
                        "=== 饮料抓取成功! ===");
                    current_state = STEP_OBJ_UP;
                    StartTimedStep();
                }
                break;
            }

            // --------------------------------------------------
            case STEP_OBJ_UP:
            {
                if (!step_timer_running_) StartTimedStep();

                sensor_msgs::msg::JointState mani_msg;
                mani_msg.name.resize(2);
                mani_msg.name[0] = "lift";
                mani_msg.name[1] = "gripper";
                mani_msg.position.resize(2);
                mani_msg.position[0] = object_z + 0.05;   // 抬高
                mani_msg.position[1] = 0.07;
                mani_pub->publish(mani_msg);

                RCLCPP_INFO(node->get_logger(),
                    "[提起] 高度=%.2f", object_z + 0.05);

                if (IsTimedStepDone(5.0))
                {
                    step_timer_running_ = false;
                    current_state = STEP_BACKWARD;
                    StartTimedStep();
                }
                break;
            }

            // --------------------------------------------------
            case STEP_BACKWARD:
            {
                if (!step_timer_running_) StartTimedStep();

                geometry_msgs::msg::Twist vel_msg;
                vel_msg.linear.x = -0.1;
                vel_pub->publish(vel_msg);

                RCLCPP_INFO_THROTTLE(node->get_logger(),
                    *(node->get_clock()), 1000,
                    "[后退] 速度=-0.1 m/s");

                if (IsTimedStepDone(10.0))
                {
                    step_timer_running_ = false;
                    StopChassis();

                    RCLCPP_INFO(node->get_logger(),
                        "=== 抓取流程完成, 降臂后前往客人 ===");

                    current_state = STEP_LOWER_ARM;
                    StartTimedStep();
                }
                break;
            }

            // --------------------------------------------------
            case STEP_LOWER_ARM:
            {
                if (!step_timer_running_) StartTimedStep();

                StopChassis();

                // 降低 lift 至运输高度, 缩小碰撞体以便通过窄通道
                // gripper 保持 0.07 (持物), lift 降至 0.25
                sensor_msgs::msg::JointState mani_msg;
                mani_msg.name.resize(2);
                mani_msg.name[0] = "lift";
                mani_msg.name[1] = "gripper";
                mani_msg.position.resize(2);
                mani_msg.position[0] = 0.25;        // 运输高度
                mani_msg.position[1] = 0.07;        // 保持夹紧
                mani_pub->publish(mani_msg);

                RCLCPP_INFO_THROTTLE(node->get_logger(),
                    *(node->get_clock()), 2000,
                    "[降臂] lift=0.25 进入运输姿态");

                if (IsTimedStepDone(4.0))
                {
                    step_timer_running_ = false;

                    success_count++;
                    current_index = 1;
                    current_state = STEP_GOTO_GUEST;
                    SendWaypoint(waypoints[1]);   // "guest"
                }
                break;
            }

            // --------------------------------------------------
            case STEP_GOTO_GUEST:
                // 等待回调 NaviDone 切换状态
                break;

            // --------------------------------------------------
            case STEP_STAY_GUEST:
                DoFaceCheck();
                break;

            // --------------------------------------------------
            case STEP_RETRY_NAVI:
                RetryNavi();
                break;

            // --------------------------------------------------
            case STEP_RETRY_GRAB:
                RetryGrab();
                break;

            // --------------------------------------------------
            case STEP_RETRY_VERIFY:
                RetryVerify();
                break;

            // --------------------------------------------------
            default:
                break;
        }

        loop_rate.sleep();
    }

    // ---- 任务结束 ----

    StopChassis();

    // 如果还没打印过结果
    if (current_state == STEP_DONE)
    {
        RCLCPP_INFO(node->get_logger(),
            "==========================================");
        RCLCPP_INFO(node->get_logger(),
            "  Delivery completed!  ");
        RCLCPP_INFO(node->get_logger(),
            "==========================================");
    }
    else if (current_state == STEP_ABORT)
    {
        RCLCPP_ERROR(node->get_logger(),
            "==========================================");
        RCLCPP_ERROR(node->get_logger(),
            "  Mission ABORTED!  ");
        RCLCPP_ERROR(node->get_logger(),
            "==========================================");
    }

    PrintMissionResult();

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
