from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument




def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("mapping"),
            "config",
            "slam_toolbox.yaml"
        )
    )


    slam_config = LaunchConfiguration("slam_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    lifecycle_nodes = ["map_saver_server"]


    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time}
        ]
    )

    #Avvio anche un altro nodo che mi consente di salvare la mappa. è un nodo lifecycle quindi avvia il manager per gestirlo 
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default", "0.196"}, #Gli autori del pacchetto consigliano di inserire sto valore
            {"occupied_thresh_default", "0.65"}
        ]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True} #Appena avvi sto nodo, tutti i nodi lifecycle che gli passi gli configura e gli attiva e gli terra attivi
        ]
    )


    return LaunchDescription([

        use_sim_time_arg,
        slam_config_arg,
        nav2_map_saver,
        slam_toolbox,
        nav2_lifecycle_manager,


    ])