import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from pathlib import Path
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction, SetLaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='diablo_description').find('diablo_description')
    default_model_path = os.path.join(pkg_share, 'src/description/diablo_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    
    #To use the turtlebot world, you have to: 
    #export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models
    #after installing:
    #sudo apt install ros-<ros2-distro>-turtlebot3-gazebo
    world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'diablo', '-topic', 'robot_description'],
        output='screen'
    )
    rtabmap_slam = Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
            parameters= [{
		        'frame_id': 'base_link',
          	    'approx_sync': True, #For external odom (odometry filtered) it should be true
                'publish_tf':True,
                'subscribe_odom_info':False, #external odom used
          	    'wait_imu_to_init': True,
          	    'wait_for_transform': 0.2,
          	    'use_sim_time': LaunchConfiguration('use_sim_time'),
                'subscribe_scan_cloud': True,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'RGBD/MarkerDetection': 'False',
                'Marker/CornerRefinementMethod': '1', #1: Subpixel
                'Marker/Dictionary': '3', #DICT_ARUCO_4X4_1000=3
                'Marker/Length': '0.2',
                'Marker/MaxDepthError': '0.01',
                'Marker/MaxRange': '10',
                'Marker/MinRange': '0',
                'Optimizer/Strategy': '1',
            	'Optimizer/Robust':'True',
	        	'Optimizer/Iterations':'10',
          	    'RGBD/OptimizeMaxError':'0',
          	    #'RGBD/OptimizeFromGraphEnd':'true',
                #'Rtabmap/StartNewMapOnLoopClosure':'true'  
          	    'Rtabmap/DetectionRate':'1',
                'cloud_noise_filtering_radius':'0.1',
                'cloud_noise_filtering_min_neighbors':'5',
                #For a planer map (like a apartment)
                'Reg/Force3DoF': "true",
                'Grid/FromDepth': 'True',
                'Grid/RayTracing': 'True',
		    }],
            remappings=[
                        ('imu', '/diablo/imu'),
                        ('rgb/image', '/depth_camera/image_raw'),
                        ('rgb/camera_info', '/depth_camera/camera_info'),
                        ('depth/image', '/depth_camera/depth/image_raw'),
                        ('scan_cloud', '/velodyne_points'),
                        ('/odom','/diablo/odom'),
            ],
            arguments=['-d', "--ros-args", "--log-level", "info"])
 
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        #rviz_node,
        rtabmap_slam
    ])
