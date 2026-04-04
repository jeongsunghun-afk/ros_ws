"""
Motorcortex + 시뮬레이터 통합 런치 파일

실행:
  ros2 launch motorcortex_bridge motorcortex_sim.launch.py

옵션:
  mcx_url:=wss://192.168.2.100    (Motorcortex 서버 주소)
  mcx_cert:=/path/to/cert.crt     (인증서 경로, 선택)
  mcx_login:=admin
  mcx_password:=vectioneer
  mcx_mode:=sim                   (기본: 시뮬레이션 모드)
                                   sim  — Simulator/targetPosition, 인코더 1,048,576
                                   prod — hostInJointAdditivePosition1, 인코더 4,096
  traj_file:=/home/jsh/leg_sim/trajectory_jump.txt
  traj_dt:=0.02                   (waypoint 간격 [s], 기본 50Hz)
  use_rviz:=true                  (true: RViz2 실행 / false: Isaac Sim 연결 모드)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_description = get_package_share_directory('CM_HL_v8')
    urdf_file = os.path.join(pkg_description, 'urdf', 'CM_HL_v11.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── 런치 인자 ────────────────────────────────────────────────────────────
    declare_mcx_url = DeclareLaunchArgument(
        'mcx_url', default_value='wss://192.168.2.100',
        description='Motorcortex 서버 WebSocket URL'
    )
    declare_mcx_cert = DeclareLaunchArgument(
        'mcx_cert', default_value='',
        description='TLS 인증서 파일 경로 (없으면 빈 값)'
    )
    declare_mcx_login = DeclareLaunchArgument(
        'mcx_login', default_value='admin',
        description='Motorcortex 로그인 ID'
    )
    declare_mcx_password = DeclareLaunchArgument(
        'mcx_password', default_value='vectioneer',
        description='Motorcortex 비밀번호'
    )
    declare_mcx_mode = DeclareLaunchArgument(
        'mcx_mode', default_value='sim',
        description="sim=시뮬레이션(기본) / prod=실제 모터 제어"
    )
    declare_traj_file = DeclareLaunchArgument(
        'traj_file', default_value='/home/jsh/leg_sim/trajectory_jump.txt',
        description='궤적 파일 경로'
    )
    declare_traj_dt = DeclareLaunchArgument(
        'traj_dt', default_value='0.02',
        description='waypoint 간격 [s] (기본 0.02 = 50Hz)'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='true: RViz2 실행 / false: Isaac Sim 연결 모드 (RViz2 미실행)'
    )

    # ── PYTHONPATH: motorcortex venv ─────────────────────────────────────────
    mcx_venv_site_packages = '/home/jsh/mcx-client-app-template/.venv/lib/python3.10/site-packages'
    existing_pythonpath = os.environ.get('PYTHONPATH', '')
    combined_pythonpath = (
        f"{mcx_venv_site_packages}:{existing_pythonpath}"
        if existing_pythonpath else mcx_venv_site_packages
    )

    # ── robot_state_publisher ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # ── joint_state_bridge ───────────────────────────────────────────────────
    joint_state_bridge = Node(
        package='motorcortex_bridge',
        executable='joint_state_bridge',
        name='motorcortex_joint_state_bridge',
        output='screen',
        additional_env={'PYTHONPATH': combined_pythonpath},
        parameters=[{
            'mcx_url':      LaunchConfiguration('mcx_url'),
            'mcx_cert':     LaunchConfiguration('mcx_cert'),
            'mcx_login':    LaunchConfiguration('mcx_login'),
            'mcx_password': LaunchConfiguration('mcx_password'),
            'mcx_mode':     LaunchConfiguration('mcx_mode'),
            'traj_file':    LaunchConfiguration('traj_file'),
            'traj_dt':      LaunchConfiguration('traj_dt'),
        }],
    )

    # ── RViz2 (use_rviz:=true 일 때만 실행) ─────────────────────────────────
    rviz_config = os.path.join(
        get_package_share_directory('motorcortex_bridge'), 'config', 'motorcortex.rviz'
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        declare_mcx_url,
        declare_mcx_cert,
        declare_mcx_login,
        declare_mcx_password,
        declare_mcx_mode,
        declare_traj_file,
        declare_traj_dt,
        declare_use_rviz,
        robot_state_publisher,
        joint_state_bridge,
        rviz2,
    ])
