# Copyright (c) 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    loopback_sim_dir = get_package_share_directory('nav2_loopback_sim')
    coverage_demo_dir = get_package_share_directory('opennav_coverage_demo')
    # sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    desc_dir = get_package_share_directory('nav2_minimal_tb4_description')

    param_file_path = os.path.join(coverage_demo_dir, 'demo_params.yaml')

    map_yaml_file = os.path.join(nav2_bringup_dir, 'maps', 'depot.yaml')
    # map_yaml_file = os.path.join(nav2_bringup_dir, 'maps', 'courtyard_benches.yaml')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # world = os.path.join(coverage_demo_dir, 'blank.world')
    # # world = os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')
    # robot_sdf = os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')

    sdf = os.path.join(desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro')
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': Command(['xacro', ' ', sdf])}
                ],
        remappings=remappings,
        )


    pose = {
        'x': LaunchConfiguration('x_pose', default='-2.00'),
        'y': LaunchConfiguration('y_pose', default='-0.50'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }

    # start the simulation
    loopback_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loopback_sim_dir, 'loopback_simulation.launch.py')),
        launch_arguments={
            'params_file': param_file_path,
            'scan_frame_id': 'rplidar_link',
        }.items(),
    )

    # start the visualization
    rviz_config = os.path.join(coverage_demo_dir, 'opennav_coverage_demo.rviz')
    # print("rviz_config:", rviz_config)
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '', 'rviz_config': rviz_config}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coverage_demo_dir, 'bringup_launch.py')),
        launch_arguments={'params_file': param_file_path}.items())

    static_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.0', '0.0', '0', '0', '0',
            'base_footprint', 'base_link']
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=param_file_path,
            root_key='',
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # world->odom transform, no localization. For visualization & controller transform
    fake_localization_cmd = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

    start_map_server = GroupAction(
        actions=[
            SetParameter('use_sim_time', True),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map_server',
                output='screen',
                parameters=[
                    configured_params,
                    {'autostart': True}, {'node_names': ['map_server']}],
            ),
        ]
    )


    # start the demo task
    demo_cmd = Node(
        package='opennav_coverage_demo',
        executable='demo_coverage',
        emulate_tty=True,
        output='screen')

    ld = LaunchDescription()

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(static_publisher_cmd)
    ld.add_action(start_map_server)
    ld.add_action(loopback_sim_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(fake_localization_cmd)
    ld.add_action(demo_cmd)

    return ld
