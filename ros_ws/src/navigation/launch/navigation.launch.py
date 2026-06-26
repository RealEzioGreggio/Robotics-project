from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    #Solita roba: quando lanci questo file puoi decidere se mettere questo argoemnto true o false
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value = "true"
    )
    #Mi prendo i lvalore a runtime della variabile use_sim_time
    use_sim_time = LaunchConfiguration("use_sim_time")

    #Quando avvii il lifecycle node manager in argomento vuole un parametro che prende la LISTA dei NOMI dei nodi che deve gestire
    #Quindi con questo launch file assicurati prima di lanciare i nodi controller_server planner_server smoother_server e poi 
    #infine avvia il lifecycle node manager con in argoemnto questa lista di nodi. In tal modo se li gestisce lui e cercera di tenerli 
    #sempre nello stato active
    lifecycle_nodes = ["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("navigation"),
                "config",
                "controller_server.yaml"
            ),
            {"use_sim_time": use_sim_time} #Cosi il use_sim time che passi quando lanci questo launch file sovrascrive il valore di default nel file yaml
        ]
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("navigation"),
                "config",
                "planner_server.yaml"
            ),
            {"use_sim_time": use_sim_time} #Cosi il use_sim time che passi quando lanci questo launch file sovrascrive il valore di default nel file yaml
        ]
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("navigation"),
                "config",
                "smoother_server.yaml"
            ),
            {"use_sim_time": use_sim_time} #Cosi il use_sim time che passi quando lanci questo launch file sovrascrive il valore di default nel file yaml
        ]
    )

    #Aggiunto molto dopo. Avvio anche il BT_navigator lo imposto per far usare ai suoi 2 plugin(nav_to_pose e nav_through_poses) lo stesso BT che ho creato con Groot2
    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("navigation"),
                "config",
                "bt_navigator.yaml"
            ),
            {"use_sim_time": use_sim_time} #Cosi il use_sim time che passi quando lanci questo launch file sovrascrive il valore di default nel file yaml
        ]
    )

    nav2_behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("navigation"),
                "config",
                "behaviors_server.yaml"
            ),
            {"use_sim_time": use_sim_time} #Cosi il use_sim time che passi quando lanci questo launch file sovrascrive il valore di default nel file yaml
        ]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation", #Questo è il secondo che avvio (DEVONO AVERE NOMI UNICI!!)
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes}, #Ricorda che deve essere una lista
            {"use_sim_time": use_sim_time},
            {"autostart": True} #Appena si avvia coonfigrua da solo i nodi e li mette su active
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        #L'ordine di avvio è importante: prima i nodi poi il lifecycle
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_bt_navigator,
        nav2_behaviors,
        nav2_lifecycle_manager
    ])