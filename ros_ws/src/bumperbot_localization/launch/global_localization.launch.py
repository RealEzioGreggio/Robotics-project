from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

#Questo nodo lancia il map server e l'amcl. il map_server pubblica una mappa gia esistente. l'amcl invece risolve il problema della global localization.
#Usa un odometry motion system(fatto bene non come quelli che faccio io), la mappa dell'ambiente(messa a disposizione da map_server), e il sensor model, esso è
#responsabile di confrontare la lettura del sensore con ciascuna posa generata e verificare la coerenza dei risultati assegnando un peso a ciascuna posa.
#Infine hai anche il resampling: le pose con un peso basso vengono eliminate.


def generate_launch_description():

    #Dichiaro due argomenti per quando lanci il lauch file. uno per dire che tempo deve utilizzare e l'altro per capire che
    #mappa deve caricare
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "config",
            "amcl.yaml"
        )
    )

    #Leggo i valori a runtime degli argomenti decisi dall'utente così gli posso usare.
    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")

    #In questa lista inserisci tutti i nodi lifecycle che vuoi che vengano gestiti dal lifecycle_manager
    lifecycle_nodes = ["map_server", "amcl"]

    map_path = PathJoinSubstitution([

        get_package_share_directory("mapping"), #pkg
        "maps", #Cartella di tutte le mappe
        map_name, #cartella della mappa che l'utente vuole utilizzare
        "map.yaml" #Contiene i metadata di come il map_server deve leggere i dati della mappa
    ])


    #Avvio direttamnte il nodo map_server, ricorda che è un lifecycle node quindi se non gli dici niente rimane in 
    # unconfigured: 
    nav2_map_server = Node(
        #Aggiungi nel packagexml la sua exec_depend
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        #I suoi parametri sono due dictionary: mappa e simulazione del tempo
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time},
        ]
    )

    #Avvio e configuro il nodo amcl che mi permette di risolvere la global localization. Pure lui è un lifecycle node. DAllo in pasto al manager
    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_config,
                    {"use_sim_time": use_sim_time}]
    )

    #aggiungi la execdeend nel xml
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},#Lista dei lifecycle node che vuoi far gestire al manager
            {"use_sim_time": use_sim_time},
            {"autostart": True} #appena si avvia cerchera di tenere tutti i nodi in active state
        ]
    )
    
    return LaunchDescription([
        
        map_name_arg,
        use_sim_time_arg,
        amcl_config_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager
    ])

