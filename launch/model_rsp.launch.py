import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

import xacro

pkg_name = 'senai_models'
default_class = "robots"

default_model = "mir100"
default_model_name = "r2d2"

# Models supported organized by class
sensors = []
robots = ["mir100"]

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    model_arg = DeclareLaunchArgument(name='model', description='Set the model to configure (Ex: mir100)')
    model_name_arg = DeclareLaunchArgument(name='model_name', default_value=str(default_model_name), description='Set the model name (Ex: r2d2)')
    model_namespace_arg = DeclareLaunchArgument(name='model_namespace', default_value=str(""), description='Set the model namespace (Ex: /testNS)')
    launch_rviz2_arg = DeclareLaunchArgument(name='launch_rviz2', default_value='true', description='Execute rviz2 automatically (Ex: false)')

    # Run the node
    return LaunchDescription([
        model_arg,
        model_name_arg, #OPTIONAL_ARG
        model_namespace_arg, #OPTIONAL_ARG,
        launch_rviz2_arg, # OPTIONAL_ARG
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    model_namespace = LaunchConfiguration('model_namespace').perform(context)
    launch_rviz2 = LaunchConfiguration('launch_rviz2').perform(context)

    model_ns = model_namespace + "/" + model_name

    if (model in sensors):
        model_class = "sensors"
    elif(model in robots):
        model_class = "robots"
    else:
        model_class = default_class
    
    file_subpath = "models/"+model_class+"/"+model+"/urdf/"+model+".urdf.xacro"
    rviz_subpath = "models/"+model_class+"/"+model+"/rviz/"+model+"_description.rviz"

    # Create a dictionary of xacro arguments
    xacro_args = {'model_name': model_name}

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file, mappings=xacro_args).toxml()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=[model_ns],
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        namespace=[model_ns]
    )

    if (launch_rviz2):
        node_rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory(pkg_name), rviz_subpath)] 
        )    

    return [node_robot_state_publisher, node_joint_state_publisher_gui, node_rviz2]