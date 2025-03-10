from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
def generate_launch_description():
    ld = LaunchDescription()
    
    # planner config
    visualize_trajectory = LaunchConfiguration("visualize_trajectory")
    gui_debug = LaunchConfiguration("gui_debug")
    #robot config
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    # degine launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            "visualize_trajectory",
            default_value="true",
            description="Visualize trajectory in RViz."
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "gui_debug",
            default_value="true",
            description="Enable GUI debug mode."
        )
    )
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
    #get ur param files
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
    #define Moveit Config Builfrt
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
            "prefix": "",
            "sim_ignition": "true"
        }
    )
    #set robot description semantic
    moveit_config_builder.robot_description_semantic(
        file_path="srdf/ur_softclaw.srdf.xacro",
        mappings= {
            "prefix": "",
            "name": "ur_softclaw"
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

    # Define move_group node including ExecuteTaskSolutionCapability
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            ],
            
        )
    
    # #start rsp node
    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[moveit_config.robot_description],
    # )
    # ld.add_action(robot_state_publisher_node)
    # Define rviz node
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("the_mtc_planner"), "rviz", "ext_calib_visual_conf.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
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
                "use_sim_time": True,
            },
        ],
    )

    # add planner node 
    planner_node = Node(
        package="the_mtc_planner",
        executable="extr_calibration_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": True,
                "visualize_trajectory": visualize_trajectory,
                "gui_debug": gui_debug,
                "camera_frame_position": [1.0, 0.0, 1.0],
                "camera_frame_orientation": [0.0, 0.7071068, 0, -0.7071068],
                "workspace_dimensions_camera_frame": [0.5, 0.5, 0.5],
                "table_dimensions": [4.0, 1.5, 0.1],
                "default_eef_ik": "aruco_frame"
            },
        ],
    )

    ld.add_action(
        TimerAction(
            period=3.0,
            actions=[run_move_group_node,rviz_node,
            TimerAction(
                period=0.5,
                actions=[planner_node]
            )]
        )
    )
    #simulation launch
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur5_softclaw_gz"), "/launch", "/ur_softclaw_gz.launch.py"]  
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "false",
        }.items(),
    )    



    ld.add_action(sim_launch)
    return ld