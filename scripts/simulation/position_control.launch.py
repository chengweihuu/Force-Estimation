from pathlib import Path
import xml.etree.ElementTree as E
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,Shutdown,ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    directory=Path(LaunchConfiguration('config_dir').perform(context)); params=str(directory/'controllers.yaml')
    robot=E.fromstring((directory/'robot.urdf').read_text())
    for param in robot.findall('.//hardware/param'):
        if param.get('name') in ('mujoco_model','pids_config_file'):
            param.text=str((directory/param.text).resolve())
    return [ExecuteProcess(cmd=[sys.executable,str(Path(__file__).with_name("equivalent_current.py")),"--config",str(directory/"equivalent_current.json")],output="screen",on_exit=Shutdown()),Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':E.tostring(robot,encoding='unicode'),'use_sim_time':True}],output='screen'),Node(package='mujoco_ros2_control',executable='ros2_control_node',parameters=[{'use_sim_time':True},params],output='screen',on_exit=Shutdown()),Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','joint_trajectory_controller','--activate-as-group','--param-file',params],output='screen')]
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_dir'),OpaqueFunction(function=setup)])
