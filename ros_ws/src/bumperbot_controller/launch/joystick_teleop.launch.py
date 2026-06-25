from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
#Ogni volta che usi un file yaml importa ste 2 librerie che ti servono solo per trovare il percorso del file di config.

def generate_launch_description():

    #Creo una variabili che mi da accesso al pacchetto(Piuttosto che stare a riscrivere ogni volta get_package...)
    bumperbot_controller_pkg = get_package_share_directory("bumperbot_controller")

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    #Voglio lanciare un nodo che (grazie alla magia nera) riesce a leggere i dati del controller, ovviamente lo devi configurare
    #coi parametri perchè gli devi dire che controller sta usando ecc... Tuttavia uso un file yaml non gli elenco direttamente.
    #Qusto nodo è il driver del controller e pubblica i suoi dati su ros (da USB a ros) pubblica su
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    #Questo nodo si iscrivera nel topic in cui pubblica il driver(nodo di sopra) e scrive in bumperbot_controller/cmd_vel che
    #saranno letti dal simple_controller(quello mio non quello di ros_control)
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_teleop.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    #Qui configuro il twist_mux lo lancio da un comodissimo launch file e gli dico che deve pubblicare im cmd_vel_unstamped
    #ricoda che hai il nodo twist che si sottoscrive a quel topic e ci aggiunge la parte stamp pubblicando un StampedTwist
    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py",
        ), 
        launch_arguments={
            "cmd_vel_out": "bumperbot_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    ) 

    twist_node = Node(
        package="bumperbot_controller",
        executable="twist_relay.py",
        name="twist",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )


    return LaunchDescription([
        use_sim_time_arg,
        joy_node,
        joy_teleop,
        twist_mux_launch,
        twist_node
    ])