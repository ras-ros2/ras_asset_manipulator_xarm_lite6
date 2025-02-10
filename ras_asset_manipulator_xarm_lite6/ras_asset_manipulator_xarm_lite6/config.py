asset_name = 'xarm_lite6'
import os
from ras_resource_lib.types.manipulator.config import ManipulatorCfg
from trajectory_msgs.msg import JointTrajectory
from launch.launch_context import LaunchContext
from launch.logging import get_logger
from ament_index_python.packages import get_package_share_directory
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

class XarmLite6Cfg(ManipulatorCfg):
    def __init__(self,moveit_config:dict,sim_actions:list,real_actions:list):
        #things to do before initialization and assignment
        super().__init__(
            label=asset_name,
            movegroup_name="lite6",
            moveit_config=moveit_config,
            sim_launch_actions=sim_actions,
            real_launch_actions=real_actions
            # launch_actions=launch_actions,
        )

    def execute_trajectory(self,trajectory: JointTrajectory):
        #do something with the trajectory
        pass

def generate_sim_launch_actions():

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            context=LaunchContext(),
            controllers_name="controllers",
            robot_type="lite",
            robot_ip="192.168.1.111",
            dof=6,
            hw_ns="lite",
            prefix="",
            ros2_control_plugin="uf_robot_hardware/UFRobotSystemHardware",
        )
        .robot_description()
        .trajectory_execution(file_path="config/lite6/controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--log-level", "debug"],
    )


    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("xarm_controller_sim"),
        "config",
        "lite6_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path, moveit_config.robot_description],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lite6_traj_controller", "-c", "/controller_manager"],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lite6_traj_controller", "-c", "/controller_manager"],
    )
    return [
        run_move_group_node,
        ros2_control_node,
        arm_controller_spawner,
    ]
    
def generate_real_launch_actions():
    import os
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch.launch_context import LaunchContext
    from launch.logging import get_logger
    from ament_index_python.packages import get_package_share_directory
    from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


    def generate_launch_actions():

        # Load the robot configuration
        moveit_config = (
            MoveItConfigsBuilder(
                context=LaunchContext(),
                controllers_name="controllers",
                robot_type="lite",
                robot_ip="192.168.1.111",
                dof=6,
                hw_ns="lite",
                prefix="",
                ros2_control_plugin="uf_robot_hardware/UFRobotSystemHardware",
            )
            .robot_description()
            .trajectory_execution(file_path="config/lite6/controllers.yaml")
            .planning_scene_monitor(
                publish_robot_description=True, publish_robot_description_semantic=True
            )
            .to_moveit_configs()
        )
        gripper_node = Node(
            package="ras_asset_manipulator_xarm_lite6",
            executable="gripper.py",
            name="xarm_gripper",
            output="both",
        )

        # Start the actual move_group node/action server
        run_move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
            arguments=["--log-level", "debug"],
        )

        # RViz
        rviz_config_file = (
            get_package_share_directory("xarm_moveit_config") + "/rviz/moveit.rviz"
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        )

        # Static TF
        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        )

        # Publish TF
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[moveit_config.robot_description],
        )

        # ros2_control using FakeSystem as hardware
        ros2_controllers_path = os.path.join(
            get_package_share_directory("xarm_controller"),
            "config",
            "lite6_controllers.yaml",
        )

        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[ros2_controllers_path, moveit_config.robot_description],
            remappings=[
                ("/controller_manager/robot_description", "/robot_description"),
            ],
            output="both",
        )

        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
        )

        arm_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["lite6_traj_controller", "-c", "/controller_manager"],
        )

        return [
                rviz_node,
                gripper_node,
                static_tf,
                robot_state_publisher,
                run_move_group_node,
                ros2_control_node,
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
            ]
    return generate_launch_actions()


def generate_configuration():
    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            context=LaunchContext(),
            controllers_name="fake_controllers",
            robot_type="lite",
            add_gripper="true",
            dof=6,
            hw_ns="lite",
            prefix="",
            ros2_control_plugin="ign_ros2_control/IgnitionSystem",
            kinematics_params_filename="lite6_default_kinematics.yaml"
        )
        .robot_description()
        .trajectory_execution(file_path="config/lite6/fake_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    return XarmLite6Cfg(moveit_config,
                        sim_actions=generate_sim_launch_actions(),
                        real_actions=generate_real_launch_actions())
