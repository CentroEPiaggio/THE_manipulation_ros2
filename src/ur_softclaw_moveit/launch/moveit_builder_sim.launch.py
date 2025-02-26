import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
def generate_launch_description():
    
    ld = LaunchDescription()

    # declare launch configuration 
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # delcalre launch arguments and add them to LaunchDescription
    ld.add_action(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur10e"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )

    ld.add_action(
       DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    ld.add_action(
       DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz to monitor or plan the robot motion.",
        )
    )



    # get ur param files
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )

    

    moveit_config_builder = MoveItConfigsBuilder(package_name="ur_softclaw_moveit", robot_name="ur_softclaw")
        
    # set robot description
    moveit_config_builder.robot_description(
        file_path="urdf/ur_softclaw.urdf.xacro",
        mappings= {
            "robot_ip": "xxx.yyy.zzz.www",
            "joint_limit_params": joint_limit_params,
            "kinematics_params": kinematics_params,
            "physical_params": physical_params,
            "visual_params": visual_params,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "name": "ur_softclaw",
            "ur_type": ur_type,
            "script_filename": "ros_control.urscript",
            "input_recipe_filename": "rtde_input_recipe.txt",
            "output_recipe_filename": "rtde_output_recipe.txt",
            "prefix": prefix,
            "sim_ignition": "true"
        }
    )
    #set robot description semantic
    moveit_config_builder.robot_description_semantic(
        file_path="srdf/ur_softclaw.srdf.xacro",
        mappings= {
            "prefix": prefix,
            "name": "ur_softclaw",
            "simulation": "true"
        }
    )

    # set moveit configs
    moveit_config_builder.robot_description_kinematics()

    moveit_config_builder.joint_limits()

    moveit_config_builder.trajectory_execution(
        file_path="config/ur_softclaw_controllers_sim.yaml",
        moveit_manage_controllers=False
    )

    moveit_config_builder.planning_scene_monitor(
        publish_robot_description=True,
        publish_robot_description_semantic=True
    )

    moveit_config_builder.planning_pipelines(
        default_planning_pipeline="ompl",
        pipelines = ["ompl"]
    )

    moveit_config = moveit_config_builder.to_moveit_configs()

    # declare and add move_group node and rviz node to launch description

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time}
            ],
           
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_softclaw_moveit"), "rviz", "view_robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
        
    )

    ld.add_action(run_move_group_node)
    ld.add_action(rviz_node)
    # ld.add_action(robot_state_publisher)

    return ld