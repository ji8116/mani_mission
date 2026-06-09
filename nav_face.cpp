#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <vector>
#include <string>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;

// 全局变量

std::shared_ptr<rclcpp::Node> node;

// 导航目标发布
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub;

// 人脸检测输入图像发布
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub;

// 定时器
rclcpp::TimerBase::SharedPtr timer;

void PrintMissionResult();

// 航点

std::vector<std::string> waypoints =
{
    "A",
    "B",
    "C",
    "HOME"
};

std::vector<std::string> skipped_points;

std::size_t current_index = 0;

// 导航状态

bool waiting_result = false;

bool retry_skipped_mode = false;

int retry_count = 0;

int max_retry = 2;

double timeout_sec = 80.0;

double stay_time_sec = 5.0;

int strategy_mode = 1;

rclcpp::Time start_time(0, 0, RCL_ROS_TIME);

// 停留状态

bool staying = false;

rclcpp::Time stay_start_time(0, 0, RCL_ROS_TIME);

rclcpp::Time last_face_check_time(0, 0, RCL_ROS_TIME);

// 人脸检测

bool face_detected = false;

rclcpp::Time last_face_time(0, 0, RCL_ROS_TIME);

// 实验统计
rclcpp::Time mission_start_time(0, 0, RCL_ROS_TIME);

rclcpp::Time mission_end_time(0, 0, RCL_ROS_TIME);

double total_time = 0.0;

int success_count = 0;

int total_waypoints = 4;

int face_detect_count = 0;

int authorized_count = 0;

bool has_skip = false;

// HOME是否成功
bool home_success = false;

// 失败航点
std::string failed_waypoint = "";

// 跳过统计
int skipped_count = 0;

// 补访成功统计
int revisit_success_count = 0;

// 状态机

enum StepState
{
    STEP_INIT,

    STEP_GOTO_A,
    STEP_STAY_A,

    STEP_GOTO_B,
    STEP_STAY_B,

    STEP_GOTO_C,
    STEP_STAY_C,

    STEP_GOTO_HOME,

    STEP_RETRY,
    STEP_SKIP,

    STEP_DONE
};

StepState current_state = STEP_INIT;

// 摄像头回调

void CamRGBCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 转发给人脸检测节点
    frame_pub->publish(*msg);
}

// 发布导航目标

void SendWaypoint(const std::string &name)
{
    std_msgs::msg::String msg;

    msg.data = name;

    navigation_pub->publish(msg);

    waiting_result = true;

    start_time = node->now();

    RCLCPP_INFO(
        node->get_logger(),
        "Navigating to waypoint %s",
        name.c_str());
}

// 发送下一个航点

void SendNextWaypoint()
{
    if(current_index < waypoints.size())
    {
        std::string target = waypoints[current_index];

        if(target == "A")
        {
            current_state = STEP_GOTO_A;
        }
        else if(target == "B")
        {
            current_state = STEP_GOTO_B;
        }
        else if(target == "C")
        {
            current_state = STEP_GOTO_C;
        }
        else if(target == "HOME")
        {
            current_state = STEP_GOTO_HOME;
        }

        SendWaypoint(target);
    }
    else
    {
        // 策略2：补访失败点

        if(strategy_mode == 2 &&
           !retry_skipped_mode &&
           !skipped_points.empty())
        {
            retry_skipped_mode = true;

            RCLCPP_WARN(
                node->get_logger(),
                "Retry skipped waypoints...");

            waypoints = skipped_points;

            skipped_points.clear();

            current_index = 0;

            SendNextWaypoint();

            return;
        }
    
        // 任务结束

        current_state = STEP_DONE;

        PrintMissionResult();

        rclcpp::shutdown();
    }
}

//结果打印
void PrintMissionResult()
{
    mission_end_time = node->now();

    total_time =
        (mission_end_time -
         mission_start_time).seconds();

    RCLCPP_INFO(
        node->get_logger(),
        "====================================");

    RCLCPP_INFO(
        node->get_logger(),
        "Mission Finished");

    RCLCPP_INFO(
        node->get_logger(),
        "Strategy Mode : %d",
        strategy_mode);

    RCLCPP_INFO(
        node->get_logger(),
        "Total Time : %.2f s",
        total_time);

    RCLCPP_INFO(
        node->get_logger(),
        "Success Waypoints : %d / %d",
        success_count,
        total_waypoints);

    RCLCPP_INFO(
        node->get_logger(),
        "Return HOME : %s",
        home_success ? "SUCCESS" : "FAILED");

    RCLCPP_INFO(
        node->get_logger(),
        "Face Detect Count : %d",
        face_detect_count);

    RCLCPP_INFO(
        node->get_logger(),
        "Authorized Count : %d",
        authorized_count);

    RCLCPP_INFO(
        node->get_logger(),
        "Skipped Waypoints : %d",
        skipped_count);

    RCLCPP_INFO(
        node->get_logger(),
        "Revisit Success Count : %d",
        revisit_success_count);

    if(success_count == total_waypoints)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "All waypoints visited successfully");
    }
    else
    {
        RCLCPP_WARN(
            node->get_logger(),
            "Task incomplete: waypoint %s failed after retries",
            failed_waypoint.c_str());
    }

    RCLCPP_INFO(
        node->get_logger(),
        "====================================");
}

// 导航成功

void NaviDone()
{
    waiting_result = false;

    retry_count = 0;

    success_count++;

    // 补访成功统计
    if(retry_skipped_mode)
    {
        revisit_success_count++;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Arrived at waypoint %s",
        waypoints[current_index].c_str());


    // A/B/C 停留


    if(current_state == STEP_GOTO_A)
    {
        current_state = STEP_STAY_A;
    }
    else if(current_state == STEP_GOTO_B)
    {
        current_state = STEP_STAY_B;
    }
    else if(current_state == STEP_GOTO_C)
    {
        current_state = STEP_STAY_C;
    }
    else
    {
        // HOME成功
        home_success = true;

        current_index++;

        SendNextWaypoint();

        return;
    }

    staying = true;

    stay_start_time = node->now();

    last_face_check_time = node->now();

    face_detected = false;

    RCLCPP_INFO(
        node->get_logger(),
        "Start face recognition...");
}

// 策略1：阻塞重试

void RetryCurrentWaypoint()
{
    if(retry_count < max_retry)
    {
        retry_count++;

        RCLCPP_WARN(
            node->get_logger(),
            "Timeout! Navigation to waypoint %s failed, retry %d/%d",
            waypoints[current_index].c_str(),
            retry_count,
            max_retry);

        rclcpp::sleep_for(2s);

        SendWaypoint(waypoints[current_index]);
    }
    else
    {
        failed_waypoint =
            waypoints[current_index];

        RCLCPP_ERROR(
            node->get_logger(),
            "Task aborted due to navigation failure at waypoint %s",
            waypoints[current_index].c_str());

        current_state = STEP_DONE;

        PrintMissionResult();

        rclcpp::shutdown();
    }
}

// 策略2：跳过并补访

void SkipWaypoint()
{
    current_state = STEP_SKIP;

    has_skip = true;

    skipped_count++;

    RCLCPP_WARN(
        node->get_logger(),
        "Skipping waypoint %s, will revisit later",
        waypoints[current_index].c_str());

    skipped_points.push_back(
        waypoints[current_index]);

    retry_count = 0;

    current_index++;

    SendNextWaypoint();
}

// 导航结果回调

void ResultCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
    if(msg->data == "navi done" &&
       waiting_result)
    {
        NaviDone();
    }
}

// 人脸位置回调

void FacePosCallback(
    const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
{
    // 只有停留阶段才做人脸检测
    if(!staying)
    {
        return;
    }

    if(msg->width > 0 &&
       msg->height > 0)
    {
        if(!face_detected)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Face detected!");
        }

        face_detected = true;

        last_face_time = node->now();
    }
}

// 停留阶段处理

void HandleStayState()
{
    if(!staying)
    {
        return;
    }

    double stay_duration =
        (node->now() -
         stay_start_time).seconds();

    // 停留结束

    if(stay_duration >= stay_time_sec)
    {
        staying = false;

        RCLCPP_INFO(
            node->get_logger(),
            "Stay finished");

        current_index++;

        SendNextWaypoint();

        return;
    }

    // 每秒检测一次

    double check_interval =
        (node->now() -
         last_face_check_time).seconds();

    if(check_interval >= 1.0)
    {
        last_face_check_time = node->now();

        double face_delay =
            (node->now() -
             last_face_time).seconds();

        // 最近1秒检测到人脸

        if(face_detected &&
           face_delay < 1.0)
        {
            face_detect_count++;

            int r = rand() % 10;

            // 70%授权
            if(r < 7)
            {
                authorized_count++;

                RCLCPP_INFO(
                    node->get_logger(),
                    "Face recognition: authorized");
            }
            else
            {
                RCLCPP_WARN(
                    node->get_logger(),
                    "WARNING: unauthorized face!");
            }
        }
        else
        {
            RCLCPP_INFO(
                node->get_logger(),
                "No face detected");
        }

        face_detected = false;
    }
}

// 超时检测

void TimerCheck()
{
    // 导航超时检测

    if(waiting_result)
    {
        double duration =
            (node->now() -
             start_time).seconds();

        if(duration > timeout_sec)
        {
            waiting_result = false;

            if(strategy_mode == 1)
            {
                RetryCurrentWaypoint();
            }
            else if(strategy_mode == 2)
            {
                SkipWaypoint();
            }
        }
    }

    // 停留阶段处理

    HandleStayState();
}

// 主函数

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>(
        "waypoint_navigation_node");

    srand(time(NULL));

    // 参数

    node->declare_parameter(
        "strategy_mode",
        1);

    node->declare_parameter(
        "timeout_sec",
        80.0);

    node->declare_parameter(
        "stay_time_sec",
        5.0);

    node->declare_parameter(
        "max_retry",
        2);

    node->get_parameter(
        "strategy_mode",
        strategy_mode);

    node->get_parameter(
        "timeout_sec",
        timeout_sec);

    node->get_parameter(
        "stay_time_sec",
        stay_time_sec);

    node->get_parameter(
        "max_retry",
        max_retry);

    // 导航Publisher

    navigation_pub =
        node->create_publisher<
        std_msgs::msg::String>(
            "/waterplus/navi_waypoint",
            10);

    // 人脸检测输入Publisher

    frame_pub =
        node->create_publisher<
        sensor_msgs::msg::Image>(
            "/face_detector_input",
            1);

    // 摄像头订阅

    auto rgb_sub =
        node->create_subscription<
        sensor_msgs::msg::Image>(
            "/kinect2/qhd/image_raw",
            1,
            CamRGBCallback);

    // 导航结果订阅

    auto result_sub =
        node->create_subscription<
        std_msgs::msg::String>(
            "/waterplus/navi_result",
            10,
            ResultCallback);

    // 人脸位置订阅

    auto face_sub =
        node->create_subscription<
        sensor_msgs::msg::RegionOfInterest>(
            "/face_position",
            10,
            FacePosCallback);

    // 定时器

    timer =
        node->create_wall_timer(
            100ms,
            TimerCheck);

    // 等待导航系统初始化

    rclcpp::sleep_for(5s);

    // 开始任务

    mission_start_time = node->now();

    RCLCPP_INFO(
        node->get_logger(),
        "Mission Start!");

    RCLCPP_INFO(
        node->get_logger(),
        "Strategy Mode : %d",
        strategy_mode);

    SendNextWaypoint();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}