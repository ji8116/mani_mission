import os

from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ---- 地图与导航参数 ----

    map_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'maps',
        'map.yaml'
    )

    nav_param_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'config',
        'nav2_params.yaml'
    )

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    # ---- nav2 导航 ----

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_launch_dir, '/bringup_launch.py']
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': nav_param_file
        }.items(),
    )

    # ---- RVIZ2 ----

    rviz_file = os.path.join(
        get_package_share_directory('wp_map_tools'),
        'rviz',
        'navi.rviz'
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file]
    )

    # ---- 航点编辑器 ----

    wp_edit_cmd = Node(
        package='wp_map_tools',
        executable='wp_edit_node',
        name='wp_edit_node'
    )

    # ---- 航点导航服务器 ----

    wp_navi_server_cmd = Node(
        package='wp_map_tools',
        executable='wp_navi_server',
        name='wp_navi_server'
    )

    # ---- 人脸检测器 ----

    face_detector_cmd = Node(
        package='wpr_simulation2',
        executable='face_detector.py',
        name='face_detector'
    )

    # ---- 3D 物体发布器 (objects_3d) ----
    # 为机械臂抓取提供 /wpb_home/objects_3d 数据
    # 注意：如果 objects_publisher 随 Gazebo 仿真自动启动，可注释掉此项

    objects_publisher_cmd = Node(
        package='wpr_simulation2',
        executable='objects_publisher',
        name='objects_publisher_node'
    )

    # ---- 组合 Launch ----
    # Gazebo 仿真 : 手动启动
    # mani 任务节点 : 手动 ros2 run

    ld = LaunchDescription()

    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(wp_edit_cmd)
    ld.add_action(wp_navi_server_cmd)
    ld.add_action(face_detector_cmd)
    ld.add_action(objects_publisher_cmd)

    return ld
