#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/search/kdtree.h>

#include <Eigen/Dense>

#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;


// 全局变量


std::shared_ptr<rclcpp::Node> node;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

rclcpp::TimerBase::SharedPtr timer;

std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


// 状态机


enum StepState
{
    STEP_INIT,

    STEP_GOTO_TABLE_A,
    STEP_STAY_A,
    STEP_RGB_CHECK_A,
    STEP_PC_DETECT_A,
    STEP_ALIGN_A,

    STEP_GOTO_TABLE_B,
    STEP_STAY_B,
    STEP_RGB_CHECK_B,
    STEP_PC_DETECT_B,
    STEP_ALIGN_B,

    STEP_GOTO_HOME,

    STEP_ABORT,
    STEP_DONE
};

StepState current_state = STEP_INIT;


// 航点


std::vector<std::string> waypoints =
{
    "TABLE_A",
    "TABLE_B",
    "HOME"
};

std::size_t current_index = 0;


// 导航控制


bool waiting_result = false;

int retry_count = 0;
int max_retry = 2;

double timeout_sec = 80.0;
double stay_time_sec = 5.0;

bool staying = false;

rclcpp::Time start_time;
rclcpp::Time stay_start_time;

rclcpp::Time mission_start_time;
rclcpp::Time mission_end_time;


// 实验统计


double total_time = 0.0;

int success_count = 0;
int total_waypoints = 3;

bool home_success = false;

double nav_time_A = 0.0;
double nav_time_B = 0.0;

double align_time_A = 0.0;
double align_time_B = 0.0;

std::vector<std::string> failed_waypoints;

double plane_height_A = 0.0;
double plane_height_B = 0.0;

int object_count_A = 0;
int object_count_B = 0;

pcl::PointXYZ target_A;
pcl::PointXYZ target_B;

rclcpp::Time align_start_time;


// RGB检测


cv::Mat last_image_;

bool visual_check_passed_ = false;


// 点云检测


double plane_height_ = 0.0;

int object_count_ = 0;

pcl::PointXYZ target_centroid_;

bool target_found_ = false;

// 【新增】目标锁定
bool target_locked_ = false;


// 微调超时


double max_align_time_ = 8.0;


// 激光安全


double min_front_dist_ = 10.0;

const double safety_threshold_ = 0.6;


// 航向锁定


double target_yaw_ = 0.0;


// 函数声明


void PrintMissionResult();

void SendWaypoint(const std::string &name);

void SendNextWaypoint();

void NaviDone();

void RetryCurrentWaypoint();

void ResultCallback(const std_msgs::msg::String::SharedPtr msg);

void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg);

void HandleStayState();

void TimerCheck();

void doRGBCheck();

void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

void doMicroAdjustment();

void finishMission();


// 打印任务结果


void PrintMissionResult()
{
    mission_end_time = node->now();

    total_time =
        (mission_end_time - mission_start_time).seconds();

    RCLCPP_INFO(node->get_logger(),
                "====================================");

    RCLCPP_INFO(node->get_logger(),
                "Mission Finished");

    RCLCPP_INFO(node->get_logger(),
                "Total Time : %.2f s",
                total_time);

    RCLCPP_INFO(node->get_logger(),
                "Success Waypoints : %d / %d",
                success_count,
                total_waypoints);

    RCLCPP_INFO(node->get_logger(),
                "Return HOME : %s",
                home_success ? "SUCCESS" : "FAILED");

    // 导航时间
    RCLCPP_INFO(node->get_logger(),
                "TABLE_A nav time : %.2f s",
                nav_time_A);

    RCLCPP_INFO(node->get_logger(),
                "TABLE_B nav time : %.2f s",
                nav_time_B);

    // 微调时间
    RCLCPP_INFO(node->get_logger(),
                "TABLE_A align time : %.2f s",
                align_time_A);

    RCLCPP_INFO(node->get_logger(),
                "TABLE_B align time : %.2f s",
                align_time_B);

    // 桌面高度
    RCLCPP_INFO(node->get_logger(),
                "TABLE_A plane height : %.3f m",
                plane_height_A);

    RCLCPP_INFO(node->get_logger(),
                "TABLE_B plane height : %.3f m",
                plane_height_B);

    // 物体数量
    RCLCPP_INFO(node->get_logger(),
                "TABLE_A object count : %d",
                object_count_A);

    RCLCPP_INFO(node->get_logger(),
                "TABLE_B object count : %d",
                object_count_B);

    // 目标坐标
    RCLCPP_INFO(node->get_logger(),
                "TABLE_A target : (%.3f, %.3f, %.3f)",
                target_A.x,
                target_A.y,
                target_A.z);

    RCLCPP_INFO(node->get_logger(),
                "TABLE_B target : (%.3f, %.3f, %.3f)",
                target_B.x,
                target_B.y,
                target_B.z);

    // 失败航点
    if (!failed_waypoints.empty())
    {
        std::string failed_list;

        for (const auto &wp : failed_waypoints)
        {
            failed_list += wp + " ";
        }

        RCLCPP_INFO(node->get_logger(),
                    "Failed waypoints : %s",
                    failed_list.c_str());
    }
    else
    {
        RCLCPP_INFO(node->get_logger(),
                    "Failed waypoints : NONE");
    }

    RCLCPP_INFO(node->get_logger(),
                "====================================");
}


// 发布航点


void SendWaypoint(const std::string &name)
{
    std_msgs::msg::String msg;

    msg.data = name;

    navigation_pub->publish(msg);

    waiting_result = true;

    start_time = node->now();

    RCLCPP_INFO(node->get_logger(),
                "Navigating to %s",
                name.c_str());
}


// 下一航点


void SendNextWaypoint()
{
    if (current_index >= waypoints.size())
    {
        current_state = STEP_DONE;

        PrintMissionResult();

        finishMission();

        return;
    }

    std::string target = waypoints[current_index];

    if (target == "TABLE_A")
    {
        current_state = STEP_GOTO_TABLE_A;
    }
    else if (target == "TABLE_B")
    {
        current_state = STEP_GOTO_TABLE_B;
    }
    else if (target == "HOME")
    {
        current_state = STEP_GOTO_HOME;
    }

    SendWaypoint(target);
}


// 导航成功


void NaviDone()
{
    waiting_result = false;

    retry_count = 0;

    success_count++;

    double nav_time =
        (node->now() - start_time).seconds();

    if (current_state == STEP_GOTO_TABLE_A)
    {
        nav_time_A = nav_time;
    }

    if (current_state == STEP_GOTO_TABLE_B)
    {
        nav_time_B = nav_time;
    }

    RCLCPP_INFO(node->get_logger(),
                "Arrived at %s",
                waypoints[current_index].c_str());

    if (current_state == STEP_GOTO_TABLE_A)
    {
        current_state = STEP_STAY_A;
    }
    else if (current_state == STEP_GOTO_TABLE_B)
    {
        current_state = STEP_STAY_B;
    }
    else if (current_state == STEP_GOTO_HOME)
    {
        home_success = true;

        current_index++;

        SendNextWaypoint();

        return;
    }

    staying = true;

    stay_start_time = node->now();
}


// 重试


void RetryCurrentWaypoint()
{
    if (retry_count < max_retry)
    {
        retry_count++;

        RCLCPP_WARN(node->get_logger(),
                    "Timeout! Retry %d/%d",
                    retry_count,
                    max_retry);

        SendWaypoint(waypoints[current_index]);
    }
    else
    {
        failed_waypoints.push_back(
            waypoints[current_index]);

        RCLCPP_ERROR(node->get_logger(),
                     "Navigation failed at %s",
                     waypoints[current_index].c_str());

        current_state = STEP_ABORT;

        PrintMissionResult();

        finishMission();// ==================================================
    }
}


// 导航结果回调


void ResultCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "navi done" &&
        waiting_result)
    {
        NaviDone();
    }
}


// RGB图像回调


void CamRGBCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        last_image_ =
            cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (...)
    {
    }
}


// 停留状态


void HandleStayState()
{
    if (!staying)
    {
        return;
    }

    if ((node->now() - stay_start_time).seconds()
        >= stay_time_sec)
    {
        staying = false;

        if (current_state == STEP_STAY_A)
        {
            current_state = STEP_RGB_CHECK_A;
        }
        else if (current_state == STEP_STAY_B)
        {
            current_state = STEP_RGB_CHECK_B;
        }
    }
}


// RGB视觉验证


void doRGBCheck()
{
    if (last_image_.empty())
    {
        return;
    }

    std::string table_name =
        (current_state == STEP_RGB_CHECK_A)
        ? "TABLE_A"
        : "TABLE_B";

    cv::imshow(table_name, last_image_);

    cv::waitKey(1);

    cv::Mat hsv;

    cv::cvtColor(last_image_,
                 hsv,
                 cv::COLOR_BGR2HSV);

    cv::Mat mask_red1;
    cv::Mat mask_red2;
    cv::Mat mask_red;
    cv::Mat mask_green;
    cv::Mat mask;

    cv::inRange(hsv,
                cv::Scalar(0, 70, 50),
                cv::Scalar(10, 255, 255),
                mask_red1);

    cv::inRange(hsv,
                cv::Scalar(170, 70, 50),
                cv::Scalar(180, 255, 255),
                mask_red2);

    cv::bitwise_or(mask_red1,
                   mask_red2,
                   mask_red);

    cv::inRange(hsv,
                cv::Scalar(40, 70, 50),
                cv::Scalar(85, 255, 255),
                mask_green);

    cv::bitwise_or(mask_red,
                   mask_green,
                   mask);

    int non_zero = cv::countNonZero(mask);

    visual_check_passed_ = non_zero > 1200;

    RCLCPP_INFO(node->get_logger(),
                "Visual check at %s done | pixels=%d",
                table_name.c_str(),
                non_zero);

    cv::imwrite(table_name + "_visual_check.jpg",
                last_image_);

    if (visual_check_passed_)
    {
        RCLCPP_INFO(node->get_logger(),
                    "RGB verification SUCCESS");

        if (current_state == STEP_RGB_CHECK_A)
        {
            current_state = STEP_PC_DETECT_A;
        }
        else
        {
            current_state = STEP_PC_DETECT_B;
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(),
                     "RGB verification FAILED");

        failed_waypoints.push_back(
            waypoints[current_index]);
    
        current_index++;

        SendNextWaypoint();
    }
}


// 激光回调


void laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    min_front_dist_ = 10.0;

    size_t center = msg->ranges.size() / 2;

    for (size_t i = center - 40;
         i < center + 40 &&
         i < msg->ranges.size();
         ++i)
    {
        if (msg->ranges[i] > 0.1 &&
            msg->ranges[i] < min_front_dist_)
        {
            min_front_dist_ = msg->ranges[i];
        }
    }
}


// 点云回调


void pcCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (current_state == STEP_PC_DETECT_A ||
        current_state == STEP_PC_DETECT_B)
    {
        processPointCloud(msg);
    }
}


// 点云处理


void processPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
    
    // 目标锁定后不再更新
    

    if (target_locked_)
    {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromROSMsg(*msg, *cloud);

    geometry_msgs::msg::TransformStamped transform;

    try
    {
        transform =
            tf_buffer_->lookupTransform(
                "base_footprint",
                msg->header.frame_id,
                msg->header.stamp,
                rclcpp::Duration::from_seconds(1.0));
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(node->get_logger(),
                    "TF failed: %s",
                    e.what());

        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(
        new pcl::PointCloud<pcl::PointXYZ>());

    Eigen::Matrix4f tf_matrix = Eigen::Matrix4f::Identity();

    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);

    tf2::Matrix3x3 mat(q);

    tf_matrix(0,0) = mat[0][0];
    tf_matrix(0,1) = mat[0][1];
    tf_matrix(0,2) = mat[0][2];

    tf_matrix(1,0) = mat[1][0];
    tf_matrix(1,1) = mat[1][1];
    tf_matrix(1,2) = mat[1][2];

    tf_matrix(2,0) = mat[2][0];
    tf_matrix(2,1) = mat[2][1];
    tf_matrix(2,2) = mat[2][2];

    tf_matrix(0,3) = transform.transform.translation.x;
    tf_matrix(1,3) = transform.transform.translation.y;
    tf_matrix(2,3) = transform.transform.translation.z;

    pcl::transformPointCloud(
        *cloud,
        *cloud_base,
        tf_matrix);

    
    // PassThrough
    

    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(cloud_base);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.6, 1.8);
    pass.filter(*cloud_base);

    pass.setInputCloud(cloud_base);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.8, 0.8);
    pass.filter(*cloud_base);

    pass.setInputCloud(cloud_base);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.3, 1.4);
    pass.filter(*cloud_base);

    
    // 平面分割
    

    pcl::ModelCoefficients::Ptr coeff(
        new pcl::ModelCoefficients);

    pcl::PointIndices::Ptr inliers(
        new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);

    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setDistanceThreshold(0.02);

    seg.setInputCloud(cloud_base);

    seg.segment(*inliers, *coeff);

    if (inliers->indices.empty())
    {
        RCLCPP_WARN(node->get_logger(),
                    "No table plane detected");

        return;
    }

    plane_height_ =
        -coeff->values[3] / coeff->values[2];

    
    // 删除桌面点
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(cloud_base);

    extract.setIndices(inliers);

    extract.setNegative(true);

    extract.filter(*object_cloud);

    
    // 保留桌面上方点
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr upper_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());

    for (auto &p : object_cloud->points)
    {
        if (p.z > plane_height_ + 0.02)
        {
            upper_cloud->points.push_back(p);
        }
    }

    upper_cloud->width =
        upper_cloud->points.size();

    upper_cloud->height = 1;

    upper_cloud->is_dense = true;

    if (upper_cloud->points.empty())
    {
        RCLCPP_WARN(node->get_logger(),
                    "No object points");

        return;
    }

    
    // 欧几里得聚类
    

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud(upper_cloud);

    std::vector<pcl::PointIndices> clusters;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance(0.05);   //

    ec.setMinClusterSize(50);

    ec.setMaxClusterSize(10000);

    ec.setSearchMethod(tree);

    ec.setInputCloud(upper_cloud);

    ec.extract(clusters);

    object_count_ = clusters.size();

    if (clusters.empty())
    {
        RCLCPP_WARN(node->get_logger(),
                    "No object clusters");

        return;
    }

    
    // 最近目标
    

    target_found_ = false;

    double min_x = 999.0;

    for (auto &cluster : clusters)
    {
        Eigen::Vector4f centroid;

        pcl::compute3DCentroid(
            *upper_cloud,
            cluster.indices,
            centroid);

        if (centroid[0] < min_x)
        {
            min_x = centroid[0];

            target_centroid_.x = centroid[0];
            target_centroid_.y = centroid[1];
            target_centroid_.z = centroid[2];

            target_found_ = true;
        }
    }

    if (!target_found_)
    {
        return;
    }

    std::string table =
        (current_state == STEP_PC_DETECT_A)
        ? "TABLE_A"
        : "TABLE_B";

    RCLCPP_INFO(node->get_logger(),
                "========== 3D DETECTION ==========");

    RCLCPP_INFO(node->get_logger(),
                "Table : %s",
                table.c_str());

    RCLCPP_INFO(node->get_logger(),
                "Plane height : %.3f m",
                plane_height_);

    RCLCPP_INFO(node->get_logger(),
                "Object count : %d",
                object_count_);

    RCLCPP_INFO(node->get_logger(),
                "Target centroid : (%.3f, %.3f, %.3f)",
                target_centroid_.x,
                target_centroid_.y,
                target_centroid_.z);

    if (current_state == STEP_PC_DETECT_A)
    {
        plane_height_A = plane_height_;
        object_count_A = object_count_;
        target_A = target_centroid_;
    }
    else
    {
        plane_height_B = plane_height_;
        object_count_B = object_count_;
        target_B = target_centroid_;
    }    

    
    // 记录航向
    

    try
    {
        auto tf_map =
            tf_buffer_->lookupTransform(
                "map",
                "base_footprint",
                tf2::TimePointZero);

        tf2::Quaternion q(
            tf_map.transform.rotation.x,
            tf_map.transform.rotation.y,
            tf_map.transform.rotation.z,
            tf_map.transform.rotation.w);

        target_yaw_ = tf2::getYaw(q);
    }
    catch (...)
    {
    }

    
    // 锁定目标
    

    target_locked_ = true;

    align_start_time = node->now();

    if (current_state == STEP_PC_DETECT_A)
    {
        current_state = STEP_ALIGN_A;
    }
    else
    {
        current_state = STEP_ALIGN_B;
    }
}


// 微调


void doMicroAdjustment()
{
    if (!target_found_)
    {
        return;
    }

    geometry_msgs::msg::Twist cmd;

    double err_x = target_centroid_.x - 0.45;

    double err_y = target_centroid_.y;

    cmd.linear.x =
        std::clamp(err_x * 0.4,
                   -0.15,
                   0.15);

    cmd.angular.z =
        std::clamp(-err_y * 1.8,
                   -0.5,
                   0.5);

    
    // IMU航向锁定
    

    try
    {
        auto tf =
            tf_buffer_->lookupTransform(
                "map",
                "base_footprint",
                tf2::TimePointZero);

        tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);

        double current_yaw =
            tf2::getYaw(q);

        double yaw_err =
            target_yaw_ - current_yaw;

        cmd.angular.z +=
            std::clamp(yaw_err * 1.2,
                       -0.4,
                       0.4);
    }
    catch (...)
    {
    }

    
    // 激光安全
    

    if (min_front_dist_ < safety_threshold_)
    {
        cmd.linear.x = 0.0;

        cmd.angular.z = 0.0;

        RCLCPP_ERROR(node->get_logger(),
                     "Obstacle too close! STOP!");

        cmd_vel_pub_->publish(cmd);

        return;
    }

    cmd_vel_pub_->publish(cmd);

    
    // 微调超时
    

    double align_elapsed =
        (node->now() - align_start_time).seconds();

    if (align_elapsed > max_align_time_)
    {
        geometry_msgs::msg::Twist stop;

        cmd_vel_pub_->publish(stop);

        target_locked_ = false;

        RCLCPP_WARN(node->get_logger(),
                    "Align timeout. Force continue.");

        failed_waypoints.push_back(
            waypoints[current_index]);

        current_index++;

        SendNextWaypoint();

        return;
    }

    
    // 收敛判断
    

    if (std::abs(err_x) < 0.08 &&
        std::abs(err_y) < 0.08)
    {
        cmd.linear.x = 0.0;

        cmd.angular.z = 0.0;

        cmd_vel_pub_->publish(cmd);

        double align_time =
            (node->now() - align_start_time).seconds();

        if (current_state == STEP_ALIGN_A)
        {
            align_time_A = align_time;
        }
        else
        {
            align_time_B = align_time;
        }

        target_locked_ = false;

        RCLCPP_INFO(node->get_logger(),
                    "Ready for grasping !");

        current_index++;

        SendNextWaypoint();

        return;
    }
}


// 定时器


void TimerCheck()
{
    // 导航超时
    if (waiting_result)
    {
        if ((node->now() - start_time).seconds()
            > timeout_sec)
        {
            waiting_result = false;

            RetryCurrentWaypoint();
        }
    }

    // 停留
    HandleStayState();

    // RGB
    if (current_state == STEP_RGB_CHECK_A ||
        current_state == STEP_RGB_CHECK_B)
    {
        doRGBCheck();
    }

    // 微调
    else if (current_state == STEP_ALIGN_A ||
             current_state == STEP_ALIGN_B)
    {
        doMicroAdjustment();
    }
}


// 结束任务


void finishMission()
{
    RCLCPP_INFO(node->get_logger(),
                "=== 桌面巡检任务结束 ===");

    cv::destroyAllWindows();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}


// 主函数


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    node =
        std::make_shared<rclcpp::Node>("pc_work");

    // Publisher
    navigation_pub =
        node->create_publisher<std_msgs::msg::String>(
            "/waterplus/navi_waypoint",
            10);

    cmd_vel_pub_ =
        node->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10);

    // Subscriber
    rgb_sub_ =
        node->create_subscription<
            sensor_msgs::msg::Image>(
                "/kinect2/qhd/image_raw",
                10,
                CamRGBCallback);

    pc_sub_ =
        node->create_subscription<
            sensor_msgs::msg::PointCloud2>(
                "/kinect2/sd/points",
                10,
                pcCallback);

    laser_sub_ =
        node->create_subscription<
            sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                laserCallback);

    auto result_sub =
        node->create_subscription<
            std_msgs::msg::String>(
                "/waterplus/navi_result",
                10,
                ResultCallback);

    // TF
    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(
            node->get_clock());

    tf_listener_ =
        std::make_shared<
            tf2_ros::TransformListener>(
                *tf_buffer_);

    // Timer
    timer =
        node->create_wall_timer(
            200ms,
            TimerCheck);

    rclcpp::sleep_for(5s);

    mission_start_time = node->now();

    RCLCPP_INFO(node->get_logger(),
                "=== 桌面巡检机器人启动 ===");

    SendNextWaypoint();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}