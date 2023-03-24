import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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

    ld.add_action(node)
    return ld