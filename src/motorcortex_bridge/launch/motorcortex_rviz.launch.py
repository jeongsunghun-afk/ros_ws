"""
Motorcortex + RViz2 통합 런치 파일

실행:
  ros2 launch motorcortex_bridge motorcortex_rviz.launch.py

옵션:
  mcx_url:=wss://192.168.2.100    (Motorcortex 서버 주소)
  mcx_cert:=/path/to/cert.crt     (인증서 경로, 선택)
  mcx_login:=admin
  mcx_password:=vectioneer
  use_actual:=true                 (true=actual 위치, false=target 위치)
  use_v2:=false                    (true=v2 CSP 궤적 제어, false=v1 읽기 전용)
  enable_csp:=true                 (v2 전용: CSP 궤적 활성화 여부)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_description = get_package_share_directory('CM_HL_v8')
    urdf_file = os.path.join(pkg_description, 'urdf', 'CM_HL_v11.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # 런치 인자 선언
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
    declare_use_actual = DeclareLaunchArgument(
        'use_actual', default_value='true',
        description='true=실제 위치(actual), false=목표 위치(target)'
    )
    declare_use_v2 = DeclareLaunchArgument(
        'use_v2', default_value='false',
        description='true=v2 CSP 궤적 제어 노드, false=v1 읽기 전용 노드'
    )
    declare_enable_csp = DeclareLaunchArgument(
        'enable_csp', default_value='true',
        description='v2 전용: CSP 궤적 제어 활성화 여부'
    )

    # robot_state_publisher: URDF 로드 및 TF 퍼블리시
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # PYTHONPATH: motorcortex venv site-packages 추가 (motorcortex 모듈 탐색용)
    mcx_venv_site_packages = '/home/jsh/mcx-client-app-template/.venv/lib/python3.10/site-packages'
    existing_pythonpath = os.environ.get('PYTHONPATH', '')
    combined_pythonpath = (
        f"{mcx_venv_site_packages}:{existing_pythonpath}"
        if existing_pythonpath else mcx_venv_site_packages
    )

    # v1 노드: 읽기 전용 (use_v2:=false 일 때)
    motorcortex_bridge_v1 = Node(
        package='motorcortex_bridge',
        executable='joint_state_bridge',
        name='motorcortex_joint_state_bridge',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_v2')),
        additional_env={'PYTHONPATH': combined_pythonpath},
        parameters=[{
            'mcx_url':      LaunchConfiguration('mcx_url'),
            'mcx_cert':     LaunchConfiguration('mcx_cert'),
            'mcx_login':    LaunchConfiguration('mcx_login'),
            'mcx_password': LaunchConfiguration('mcx_password'),
            'use_actual':   LaunchConfiguration('use_actual'),
            'publish_rate': 50.0,
        }],
    )

    # v2 노드: CSP 궤적 제어 (use_v2:=true 일 때)
    motorcortex_bridge_v2 = Node(
        package='motorcortex_bridge',
        executable='joint_state_bridge_v2',
        name='motorcortex_joint_state_bridge_v2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_v2')),
        additional_env={'PYTHONPATH': combined_pythonpath},
        parameters=[{
            'mcx_url':      LaunchConfiguration('mcx_url'),
            'mcx_cert':     LaunchConfiguration('mcx_cert'),
            'mcx_login':    LaunchConfiguration('mcx_login'),
            'mcx_password': LaunchConfiguration('mcx_password'),
            'publish_rate': 50.0,
            'enable_csp':   LaunchConfiguration('enable_csp'),
        }],
    )

    # RViz2 (WSL2 OpenGL 충돌 방지를 위해 소프트웨어 렌더링 사용)
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
    )

    return LaunchDescription([
        declare_mcx_url,
        declare_mcx_cert,
        declare_mcx_login,
        declare_mcx_password,
        declare_use_actual,
        declare_use_v2,
        declare_enable_csp,
        robot_state_publisher,
        motorcortex_bridge_v1,
        motorcortex_bridge_v2,
        rviz2,
    ])
