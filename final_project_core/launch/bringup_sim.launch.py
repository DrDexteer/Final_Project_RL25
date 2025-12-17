from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    # -----------------------
    # Declare launch arguments
    # -----------------------
    launch_mission_arg = DeclareLaunchArgument(
        'launch_mission',
        default_value='true',
        description='Start  coordinator + iiwa gesture.'
    )

    gz_world_arg = DeclareLaunchArgument(
        'gz_world',
        default_value='final_project_world',
        description='Gazebo world name (must match /world/<name>).'
    )

    iiwa_spawn_x_arg = DeclareLaunchArgument('iiwa_spawn_x', default_value='1.0')
    iiwa_spawn_y_arg = DeclareLaunchArgument('iiwa_spawn_y', default_value='0.0')
    iiwa_spawn_z_arg = DeclareLaunchArgument('iiwa_spawn_z', default_value='0.10')

    iiwa_command_interface_arg = DeclareLaunchArgument(
        'iiwa_command_interface',
        default_value='position',
        description='iiwa command interface [position|velocity|effort]'
    )

    iiwa_robot_controller_arg = DeclareLaunchArgument(
        'iiwa_robot_controller',
        default_value='iiwa_arm_trajectory_controller',
        description='iiwa controller name to spawn (e.g., velocity_controller)'
    )

    # -----------------------
    # ArUco detector arguments
    # -----------------------
    aruco_enable_arg = DeclareLaunchArgument(
        'aruco_enable',
        default_value='true',
        description='Start aruco_ros single detector'
    )
    aruco_id_arg = DeclareLaunchArgument(
        'aruco_id',
        default_value='201',
        description='Aruco marker id to detect'
    )
    aruco_size_arg = DeclareLaunchArgument(
        'aruco_size',
        default_value='0.10',
        description='Aruco marker size in meters'
    )

    # -----------------------
    # LaunchConfigurations
    # -----------------------
    launch_mission = LaunchConfiguration('launch_mission')
    gz_world = LaunchConfiguration('gz_world')

    iiwa_spawn_x = LaunchConfiguration('iiwa_spawn_x')
    iiwa_spawn_y = LaunchConfiguration('iiwa_spawn_y')
    iiwa_spawn_z = LaunchConfiguration('iiwa_spawn_z')

    iiwa_command_interface = LaunchConfiguration('iiwa_command_interface')
    iiwa_robot_controller = LaunchConfiguration('iiwa_robot_controller')

    aruco_enable = LaunchConfiguration('aruco_enable')
    aruco_id = LaunchConfiguration('aruco_id')
    aruco_size = LaunchConfiguration('aruco_size')

    # -----------------------
    # Include fra2mo gazebo/world
    # -----------------------
    fra2mo_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros2_fra2mo'), 'launch', 'gazebo_fra2mo.launch.py'])
        ),
    )

    # -----------------------
    # Include iiwa (spawn only, Gazebo già avviato)
    # -----------------------
    iiwa_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('iiwa_bringup'), 'launch', 'iiwa.launch.py'])
        ),
        launch_arguments={
            'use_sim': 'true',
            'launch_gazebo': 'false',

            # IMPORTANT: world name deve combaciare con quello del gazebo già avviato
            'gz_world': gz_world,

            # spawn pose
            'spawn_x': iiwa_spawn_x,
            'spawn_y': iiwa_spawn_y,
            'spawn_z': iiwa_spawn_z,

            # control interface + controller
            'command_interface': iiwa_command_interface,
            'robot_controller': iiwa_robot_controller,
        }.items()
    )

    # ----------------------
    from launch_ros.parameter_descriptions import ParameterValue

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': ParameterValue(aruco_size, value_type=float),
            'marker_id': ParameterValue(aruco_id, value_type=int),
            'reference_frame': '',
            'camera_frame': 'camera_optical_frame',
            'marker_frame': 'aruco_marker_frame',
            'corner_refinement': 'LINES',
        }],
        remappings=[
            ('/image', '/iiwa/videocamera'),
            ('/camera_info', '/iiwa/videocamera_info'),
        ],
        condition=IfCondition(aruco_enable),
    )



    # -----------------------
    # Mission nodes
    # -----------------------
    params_file = PathJoinSubstitution([
        FindPackageShare('final_project_core'),
        'config',
        'final_project_params.yaml'
    ])

    coordinator = Node(
        package='final_project_core',
        executable='coordinator',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(launch_mission),
    )

    iiwa_gesture = Node(
        package='final_project_core',
        executable='iiwa_gesture',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(launch_mission),
    )




    

    # -----------------------
    # Return LaunchDescription
    # -----------------------
    return LaunchDescription([
        # args
        launch_mission_arg,
        gz_world_arg,
        iiwa_spawn_x_arg,
        iiwa_spawn_y_arg,
        iiwa_spawn_z_arg,
        iiwa_command_interface_arg,
        iiwa_robot_controller_arg,

        aruco_enable_arg,
        aruco_id_arg,
        aruco_size_arg,

        # system
        fra2mo_gazebo,
        iiwa_spawn,

        # perception
        aruco_single,

        # mission
        coordinator,
        iiwa_gesture,
    ])
