from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue 
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

#Questo launch file è per la maggior parte un copia incolla di gazebo.launch.py solo che lo avvi se non vuoi il robot simulato
#Avvia le interfaccie per il robot reale
def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("bumperbot_description"),
                    "urdf",
                    "bumperbot.urdf.xacro",
                ),
                " is_simulation:=False"
            ]
        ),
        value_type=str,
    )

    #Rende disponibile il modello del tuo robot urdf in un topic
    robot_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        parameters=[{"robot_description": robot_description}]
    )

    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description" : robot_description,
             "use_sim_time": False},
                os.path.join(
                get_package_share_directory("bumperbot_controller"), "config", "bumperbot_controllers.yaml"
            )
        ]
    )

    imu_node = Node(
        package="bumperbot_firmware",
        executable="IMU.py"
    )


    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        imu_node
    ])