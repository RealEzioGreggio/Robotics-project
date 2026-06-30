import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node

#Questo file contiene i launch file da lanciare per far funzionare il robot reale. sull'hardware fisico è imprtante lanciare
#sia l'interfaccia hardware che il joystick teleop se lo si vuole far funzionare.

#Ok ora questo file da ESATTAMENTE le stesse del sim. tranne per gazebo ovviamente che è stato sostituito dall'interfaccia hardware.
#Non sono sicuro sia una buona idea lanciare tutta sta roba sull'hardware. SLAM  e localization posso anche farle dal mio cmputer e lascio
#interfaccia hardware, controller(joystick e non diff_drive controller di ros_control), IMU(ancora non funziona..), LaserScanner. Il resto lo faccio dal mio computer

#Questo launch file lancia gia l'IMU.
def generate_launch_description():

    #Con questo parametro decido se lanciare direttamente SLAM o solo localization
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_slam = LaunchConfiguration("use_slam")

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_noisy_controller": "False",
        }.items()
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False",
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition = UnlessCondition(use_slam) #Avvi questo solo se use_slam==False
    )


    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition = IfCondition(use_slam)
    )

    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop.py",
        output="screen"
    )

    laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        output="screen",
        parameters=[os.path.join(
            get_package_share_directory("bumperbot_bringup"),
            "config",
            "rplidar_a1.yaml"
        )]
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("navigation"),
            "launch",
            "navigation.launch.py"
        )
    )

    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        controller,
        joystick,
        laser_driver,
        #safety_stop,
        localization,
        slam,
        navigation
    ])