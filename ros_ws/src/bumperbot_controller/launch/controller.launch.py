from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction #Opaque mi serve per fare la somma fra 2 argomenti
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

#In questo file di launch voglio avviare i 2 nodi che mi servono per la simulazione. In pratica sono riuscito a simulare
#il robot in gazebo aggiungendo i parametri necessari(inerzie attriti...) e per la simulazuine devo trovare un modo per 
#far muovere i giunti: in pratica devo simulare l'hardware (sia nel ricevere i comandi che nel leggere lo stato) ho fatto cio
#implementando nella descrizione del robot un plugin(ros_control) con tale libreria sono stato in grado di implementare 
#l'hardware(in pratica gli ho detto che i giunti della right and left wheel sono i miei attuatori) e il controllore,
#ci puo interagire attraverso 2 interfacce. L'algoritmo di controllo(che ancora non esiste) interagisce con tali interfacce
#attraverso un'ulteriore interfaccia che è quella del controller manager


#Fatto cio voglio che questo launch avvia anche il nodo che manda i comandi al controller(di ros_controller) per far muovere
#il robot(Controllore definito nel controller manager)

#Ho aggiunto anche degli argomenti di errore nella misura del raggio e della distanza delle ruote del robot. Puoi passare
#l'errore come argomento, poi questo viene sommato grazie a opaque (in una maniera semplicissima e intuitiva) all'argomento
#wheel_radius e wheel_separation e poi vengono presi in argomento dal nodo che implementa il controller del robot



#Questa funzione serve per Opaque(che serve per sommare il valore di 2 argomenti del launch file) context ti serve per avere
#accesso al valore di questi argomenti.
def noisy_controller(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context)) #OK qui faccio accesso all'argomento e con perform leggo il valore
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))
    
    #Avvio il controller col rumore che si aspetta degli argomenti
    noisy_controller_py = Node(
        package="bumperbot_controller",
        executable="noisy_controller.py",
        parameters=[
            {"wheel_radius" : wheel_radius + wheel_radius_error, # Questa cosa non la posso fare senza opaque
             "wheel_separation" : wheel_separation + wheel_separation_error,
             "use_sim_time": use_sim_time}
        ]
    )
    return [
        noisy_controller_py
    ]
     

def generate_launch_description():


    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    use_noisy_controller_arg = DeclareLaunchArgument(
        "use_noisy_controller",
        default_value="False",
    )

    #Imposto degli argomenti al launch file(parametri del robot) e valori di default nel caso non gli specifichi
    wheel_radius_arg = DeclareLaunchArgument(

        "wheel_radius",
        default_value="0.033"
    )

    wheel_separation_arg = DeclareLaunchArgument(

        "wheel_separation",
        default_value="0.17"
    )

    #Questo argomento lo inserisco per decidere quale dei controllori deve avviare il controller_manager. Hai due opzioni:
    #Avvii con true allora userai=> simple velocity controller#
    use_simple_controller_arg = DeclareLaunchArgument(

        "use_simple_controller",
        default_value="true"
    )

    #Inserisco degli erorir sul raggio e sulla separazione delle ruote giusto per vedere che succede
    wheel_radius_error_arg = DeclareLaunchArgument(

        "wheel_radius_error",
        default_value="0.005"
    )

    wheel_separation_error_arg = DeclareLaunchArgument(

        "wheel_separation_error",
        default_value="0.02"
    )

    #assegna a wheel radius il valore di wheel_radius(inserito dall'utente o quello di default).
    #Questi valori sono quelli che inserira l'utente
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_noisy_controller = LaunchConfiguration("use_noisy_controller")


    #Con questo Nodo puoi far spawnare il controller che vai a definire nel file yaml.
    #Questo è il modo in cui vai a chiamare il controllore e il joint_state_broadcaster. Non li chiami direttamente ma lanci
    #Questo nodo spawner con in argomento il jointState broadcaste e il controllore che ti interessa usare.
    #Pubblica in joint_states lo stato dei giunti del modello del robot
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner" ,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    #Non voglio che questa venga eseguita ogni volta ma solo se use_simple_controller == False
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner" ,
        arguments=[
            "bumperbot_controller", #Controllore dichiarato e configurato nel file yaml
            "--controller-manager",
            "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller)
    )


    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        #Tutti questi nodi vengono avviati solo se rispettano la condizione
        actions=[
        #Questa deve essere eseguita solo se use_simple_controller == True
            Node(
                package="controller_manager",
                executable="spawner" ,
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ],
            ),
            #Qui lancio il controller che ho creato io (quello per usare il controller di una xbox)
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                #Qui associo i parametri del launch file a quelli che ho creato nel nodo. "parametroNodo": parametroLaunchFile
                parameters=[{"wheel_radius": wheel_radius},
                            {"wheel_separation": wheel_separation},
                            {"use_sim_time": use_sim_time}]
            )
        ]
    )


    noisy_controller_launch = OpaqueFunction(
    function=noisy_controller,
    condition=IfCondition(use_noisy_controller)
    )

    #Lista di funzionalita che vuoi avviare in questo launch file mettici tutti gli argomenti e nodi da avviari o anche altri laucnh file
    return LaunchDescription([
        
        use_sim_time_arg,
        use_noisy_controller_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        wheel_controller_spawner,
        noisy_controller_launch,
    ])

#Come al solito di al compilatore dell'esistenza di questa cartella-> Modifica Cmake.
#Aggiungi anche le dipendenze dal controller_manager