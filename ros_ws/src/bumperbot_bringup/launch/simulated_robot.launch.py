import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node

#Questo file contiene tutto il necessario per la simulazione del robot.
#Attualmente lancia la descrizione del robot con ros_control e tutti i plugin necessari, il controller del robot, NON lancia la local localization(ekf con imu,
# per odometria ti basta il launch fle del controller), i joystick(con twist_mux e il joystick la tastiera lanciale a parte key_teleop) ed infine la global loc.
# Puoi dargli diversi argomenti fra cui world_name questo ti permette di cambiare la mappa(è un argomento del file launche che lancia gazebo bump.description gazebo.launch.py)
# Puoi decidere anche se lanciare direttamente slam o solo localization con amcl con use_slam.
#Lancia anche la safety_stop in bumperbot_utils. 



def generate_launch_description():

    #Con questo parametro decido se lanciare direttamente SLAM o solo localization
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )


    use_slam = LaunchConfiguration("use_slam")

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        )
    )

    rviz_config_slam = os.path.join(
        get_package_share_directory("bumperbot_description"),
        "rviz",
        "slam.rviz"
    )

    rviz_config_amcl = os.path.join(
        get_package_share_directory("bumperbot_description"),
        "rviz",
        "amcl.rviz"
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_noisy_controller": "True",
        }.items()
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True",
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

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("navigation"),
            "launch",
            "navigation.launch.py"
        )
    )

    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop.py",
        output="screen"
    )

    rviz_node_slam = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_slam],
        condition=IfCondition(LaunchConfiguration("use_slam"))
    )

    rviz_node_amcl = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_amcl],
        condition=UnlessCondition(LaunchConfiguration("use_slam"))
    )

    return LaunchDescription([

        use_slam_arg,
        gazebo,
        controller,
        joystick,
        #safety_stop,
        localization,
        slam,
        navigation,
        rviz_node_slam,
        rviz_node_amcl

    ])