import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    param = os.path.join(
		get_package_share_directory('rbf_clap_b7_driver'),
		'config',
		'clap_b7_driver.param.yaml'
	)

    node=Node(
        package = 'rbf_clap_b7_driver',
        name = 'rbf_clap_b7_driver',
        executable = 'rbf_clap_b7_driver_exe',
        parameters = [param]
    )

    with open(param,'r') as f:
        parameters = yaml.safe_load(f)

    ntrip_client_pack = parameters["/**"]["ros__parameters"]["ntrip_client_package_name"]

    if(parameters["/**"]["ros__parameters"]["activate_ntrip"]=="true"):
        try:
            ntrip_client_ros_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(ntrip_client_pack),
                       "launch/ntrip_client_ros.launch.py"
                    )
                )
            )
            ld.add_action(ntrip_client_ros_launch)
        except:
            print("\033[1;31mCouldnt find ntrip_client_ros package\033[0m")

    ld.add_action(node)
    return ld