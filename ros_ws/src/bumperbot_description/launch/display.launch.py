from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os #Ti serve per interagire col OS, la usi per specificare directory
from ament_index_python.packages import get_package_share_directory #La usi per trovare la directory di un package che hai creato
from launch_ros.parameter_descriptions import ParameterValue # Ti permette di creare una variabile che funge da parametro per un nodo
from launch.substitutions import Command, LaunchConfiguration#Ti permette di eseguire comandi??

#Qui è dove mi sono messa a piangere


#La prima cosa che voglio fare è lanciare il robot_state_publisher node e in argomento ci voglio mettere il mio robbottino

#Voglio andare a definire una lista di istruzioni da eseguire(launch description)
def generate_launch_description():
    
    #
    model_arg = DeclareLaunchArgument(
        name="model",
        #Nel caso model non viene specificato uso il modello che voglio io.
        #
        default_value=os.path.join(get_package_share_directory("bumperbot_description"), "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    #Questa è la parte piu importante: vado a definire un parametro che contiene il percorso del file xacro del mio robot
    #Tuttavia c'è da convertirlo in URDF quindi lanciamo il comando xacro sul percorso dell'URDF file ()
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    #Volgio avviare questo nodo
    robot_state_publisher = Node(
        package="robot_state_publisher", #Gli passi il pacchetto del nodo 
        executable="robot_state_publisher", # e anche l'eseguibile
        parameters=[{"robot_description": robot_description}]#Qui assegni anche i parametri che vuoi passargli quando lo avvii
                                                            #Il robot_description è un argomento di questo launch file che poi
                                                            #sara assegnato al parametro di robot_state_publisher
    )

    #Secondo nodo che voglio avviare
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    #Ultimo nodo da avviare è quello di Rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",#L'output lo voglio vedere nel terminale
        arguments=["-d", os.path.join(get_package_share_directory("bumperbot_description"), "rviz", "rviz_conf.rviz")] #File di configurazione rviz salvato in precedenza
    )


    #Voglio che qeusta funzione restituisca un oggetto di tipo LaunchDescription, il suo costruttore prende in ingresso una
    #lista di istruzioni (quelle che vuoi vengano eseguite). Tutto quello che hai creato lo metti qui dentro cosi viene
    #effettivamente eseguito
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])


#Devi installare questo file vai nel Cmake e aggiungi anche le dipendenze
#Sta merda funziona 