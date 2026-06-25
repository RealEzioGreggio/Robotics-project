from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription#-> Aggiunto questo.
import os #
from os import pathsep
from pathlib import Path # Ti converte i percorsi 
from ament_index_python.packages import get_package_share_directory 
from launch_ros.parameter_descriptions import ParameterValue 
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression #Python expression la usi per usare le estensioni world
from launch.launch_description_sources import PythonLaunchDescriptionSource #Questa la usi per dire che l'altro launch file che vuoi lanciare da questo è anchesso scritto in py

#Prima di tutto ci serve il robot_state_publisher node che legge l'urdf e i joint e pubblica le trasformate -> CopiaIncolla




def generate_launch_description():

    bumperbot_description = get_package_share_directory("bumperbot_description")
    #Lancio il comando environ dal os che mi restituisce una variabile d'ambiente, gestita automaticamente, e contiene la
    #distro di ros. ros_distro è una string e dipende dalla distribuzione di ros che hai lanciato.
    #In base a tale valore setta la variabile is_ignition to true or false. Questo è stato necessario perchè
    #ho deciso di utilizzare il plugin di ros_control sia con jazzy che con humble
    ros_distro = os.environ["ROS_DISTRO"]
    #Devi trovare un modo per collegare questa var is_ignition a quella presente in bumperbot.urdf.xacro. lo faccio poco piu sotto
    is_ignition = "True" if ros_distro == "humble" else "false"

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                bumperbot_description, "urdf", "bumperbot.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    #Ho inserito una cartella contenente diversi world in cui far spownare il robot. usa i modelli contenunti in models e anche 
    # le foto contenute in photos. Installa queste 3 cartelle nel CMake
    world_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    #Launch configuration("world_name") prende a runtime il valore  di world_name che gli vai ad assegnare. Dopodiche world Path glielo passi al launch file
    #di gazebo come argomento.
    world_path = PathJoinSubstitution([
            bumperbot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(bumperbot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("bumperbot_description"), 'models')


    #Qui setto un parametro(il modello del robot) che se lo aspetta in urdf quindi lo converto e inoltre setto is_ignition presente
    #in quel file con is_ignition descritto poco piu sopra. Il risultato è una stringa
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]), value_type=str)

    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    #Aggiungo una variabile d'ambiente che contiene la directory del modello del nostro robot
    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    #Per avviare gazebo si esegue un launch file. tale launch file lo avvii con un'altro launch file. Il servizio è offerto da
    #IncludeLaunchDescription class. Poi usi PythonLaunchDescriptionSource per dire che il file da lanciare è in py e in 
    #input si prende un vettore che contiene la directory del file lauch che vuoi lanciare. Gli passi anche degli argomenti 
    #Per settare il verbosity level di gazebo sul console. L'output lo vedi sul terminale. il -r fa partire immediatamente la
    #simulazione dopo che si è avviato, l'ultimo argomento è il mondo in cui fai partire la simulazione

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )
    
    #Ora devi far spawnare il robot nella simulazione #Non prendi il robot dal file URDF ovviamnte ma ti prendi le trasformate
    # fatte e pubblicate da robot_state_publisher nel topic robot_description. gazebo si inscrive la. 
    #Infine gli passi anche un nome che sarà visualizzato per il tuo robot nella simulazione

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output = "screen", # Così vedo l'output dallo stesso terminale in cui lo sto lanciando
        arguments=["-topic", "robot_description",
                    "-name", "bumperbot",
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.065",

                    "-R", "0.0",   # roll
                    "-P", "0.0",   # pitch
                    "-Y", "0.0"   # yaw
        ] 
    )

    #Questo nodo mi consente di creare un bridge fra i topic di gz e ros così posso vedere cosa pubblicano i sensori simulati
    gz_ros2_bridge = Node(
        package="ros_gz_bridge", #Aggiungi dipendenza in xml
        executable="parameter_bridge",
        #voglio convertire il messaggio pubblicato in /imu(topic di gazebo) di tipo gz.msgs.IMU  in sensor_msgs/msg/Imu pubblicato in ros
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" #Così ti trovi /scan in ros
        ],
        #Col remapping cambi il nome del topic: Da /imu(gz) in /imu/ros(ros)
        remappings=[
            ("/imu", "/imu/ros")
        ]
    )

    #Qui metti tutte le istruzioni che hai creato
    return LaunchDescription([
        model_arg,
        world_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])

#Non scordarti di modificare il xml per le nuovissime dipendenze