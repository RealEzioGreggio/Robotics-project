from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    #Mi serve la trasformata(statica) tra il base_footprint e l'imu_link (quelli dell'urdf). QUindi creo qui una trasformata
    #lanciando static_transform_publisher con i giusti argomenti. Questa trasformata sara utilizzata da ekf_node (vedi file yaml)
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0.103", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_footprint_ekf", "--child-frame-id", "imu_link_ekf"],
    )

    #Questo nodo implementa un EKF. in input vuole dati imu, odometria(encoder) Sputa fuori invece: posizione, orientamento(quat)
    # e velocita. Se chiedi gentilmente puoi farti buttar fuori direttamente una trasformata.
    #Questo nodo lavora con trasformate. Perche i calcoli degli stati del robot li fa nel frame di riferimento del sensore e li 
    #riporta nel sistema di riferimetno del robot(glielo specifichi tu vedi file yaml)
    #Come fa a sapere qual' è il frame del sensore?? Lo legge dal messaggio se il mio sensore è imu al suo interno ha 
    #un campo con header.frame_id (Credo che tutti i msg dei sensori lo abbiano) e quel frame è lo stesso frame dell'urdf a
    #cui il sensore è collegato(IMU_link nel mio caso) PUBBLICA DA SOLO IN odometry/filtered
    robot_localization = Node(
        package="robot_localization", #Aggiungi sto pacchetto nel xml
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[os.path.join(get_package_share_directory("bumperbot_localization"), "config", "ekf.yaml")]
    )

    imu_republisher = Node(
        package="bumperbot_localization",
        executable="imu_republisher.py"
    )

    return LaunchDescription([

        static_transform_publisher,
        robot_localization,
        imu_republisher

    ])