import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    ld = LaunchDescription()

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'robot'
    file_subpath = 'urdf/robot.urdf.xacro'  
    slam_toolbox_path = [os.path.join(get_package_share_directory('slam_toolbox'), 'launch'),'/online_async_launch.py']

     # Set ignition resource path (to be able to render meshes)
    resource_paths = [os.path.join(get_package_prefix(pkg_name), 'share')]
    # Add any existing declared resources
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        resource_paths.insert(0, os.environ['IGN_GAZEBO_RESOURCE_PATH'])

   # Concatenate a path seperator (':') between all paths and update the environment variable
    ign_resource_path_update = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',value=[os.pathsep.join(resource_paths)])

    # Start SLAM Toolbox with default parameters

    # SLAM for front LiDAR
    slam_left_tof = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_path),
        launch_arguments={
            'slam_params_file': os.path.join(
                get_package_share_directory(pkg_name), 'config/left_tof.yaml'),
            'use_sim_time': 'true'
        }.items()
    )

    # SLAM for rear LiDAR
    slam_right_tof = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_path),
        launch_arguments={
            'slam_params_file': os.path.join(
                get_package_share_directory(pkg_name), 'config/right_tof.yaml'),
            'use_sim_time': 'true'
        }.items()
    )

    # Use xacro to process the URDF file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    map_merger = Node(
        package='robot',
        executable='map_merger',
        output='screen'
    )

    # Bridge for communication between ROS and Gazebo
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock' + '@rosgraph_msgs/msg/Clock' + '[' + 'ignition.msgs.Clock',
            '/model/robot/odometry' + '@nav_msgs/msg/Odometry' + '[' + 'ignition.msgs.Odometry',
            '/model/robot/scan' + '@sensor_msgs/msg/LaserScan' + '[' + 'ignition.msgs.LaserScan',
            '/model/robot/tf' + '@tf2_msgs/msg/TFMessage' + '[' + 'ignition.msgs.Pose_V',
            '/model/robot/imu' + '@sensor_msgs/msg/Imu' + '[' + 'ignition.msgs.IMU',
            '/model/robot/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'ignition.msgs.Model',
            '/model/robot/depth_camera/image_raw' + '@sensor_msgs/msg/Image' + '[' + 'ignition.msgs.Image',
            '/model/robot/depth_camera/camera_info' + '@sensor_msgs/msg/CameraInfo' + '[' + 'ignition.msgs.CameraInfo',
        ],
        remappings=[
            ('/model/robot/odometry', '/odom'),
            ('/model/robot/scan', '/scan'),
            ('/model/robot/tf', '/tf'),
            ('/model/robot/imu', '/imu_raw'),
            ('/model/robot/joint_state', 'joint_states'),
        ],
        output='screen'
    )
    
# Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path_update)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(slam_left_tof)
    ld.add_action(slam_right_tof)
    ld.add_action(node_ros_gz_bridge)
    ld.add_action(map_merger)
    return ld
