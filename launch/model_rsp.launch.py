import os, re, yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

import xacro

pkg_name = 'senai_models'
default_class = "robots"

default_model = "mir100"
default_model_name = "r2d2"

#Choose between "ignition" or "classic"
default_gazebo = "classic"
default_world = "empty.sdf"

# Models supported organized by class
sensors = []
robots = ["mir100"]

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    model_arg = DeclareLaunchArgument(name='model', description='Set the model to configure (Ex: mir100)')
    model_name_arg = DeclareLaunchArgument(name='model_name', default_value=str(default_model_name), description='Set the model name (Ex: r2d2)')
    model_namespace_arg = DeclareLaunchArgument(name='model_namespace', default_value=str(""), description='Set the model namespace (Ex: /testNS)')
    launch_rviz2_arg = DeclareLaunchArgument(name='launch_rviz2', default_value='true', description='Execute rviz2 automatically (Ex: false)')
    launch_gazebo_arg = DeclareLaunchArgument(name='launch_gazebo', default_value='false', description='Execute gazebo automatically (Ex: false)')

    # Run the node
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        model_arg,
        model_name_arg, #OPTIONAL_ARG
        model_namespace_arg, #OPTIONAL_ARG,
        launch_rviz2_arg, # OPTIONAL_ARG
        launch_gazebo_arg, # OPTIONAL_ARG
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    model_name.replace(" ","_")

    model_namespace = LaunchConfiguration('model_namespace').perform(context)
    launch_rviz2 = LaunchConfiguration('launch_rviz2').perform(context)
    launch_gazebo = LaunchConfiguration('launch_gazebo').perform(context)

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
    xacro_args = {
        'model_name': model_name,
        'gazebo_version': default_gazebo
    }

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file, mappings=xacro_args).toxml()

    # Configure the node Robot_State_Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=[model_ns],
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': robot_description_raw
            }
        ] # add other parameters here if required
    )

    nodes = [node_robot_state_publisher]

    if (launch_rviz2 == "true"):
        # Loading Joint_State_Publisher_Gui Node
        node_joint_state_publisher_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            namespace=[model_ns]
        )

        nodes.append(node_joint_state_publisher_gui)

        # Loading RViZ2 Node
        node_rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory(pkg_name), rviz_subpath)] 
        )

        nodes.append(node_rviz2)

    if (launch_gazebo == "true"):

        # Launch Gazebo
        if(default_gazebo == "classic"):
            robot_description_node_name = "/"+model_name+"/robot_description"

            node_gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(
                        FindPackageShare('gazebo_ros').find('gazebo_ros'), 'launch', 'gazebo.launch.py'
                    )]
                )
            )
            nodes.append(node_gazebo)

            node_gazebo_spawn = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='urdf_spawner',
                output='screen',
                arguments=["-topic", robot_description_node_name, "-entity", model_name]
            )
            nodes.append(node_gazebo_spawn)

        elif(default_gazebo == "ignition"):

            # Launching Gazebo Ignition
            node_gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(
                        FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
                        'launch', 'gz_sim.launch.py'
                    )]
                ),
                launch_arguments={'gz_args': f'-r {default_world}'}.items()
            )

            nodes.append(node_gazebo)
                
            # Spawning model
            node_gazebo_spawn = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-string', robot_description_raw,
                    '-name', model_name,
                    '-allow_renaming', 'true'
                ],
                output='screen'
            )

            nodes.append(node_gazebo_spawn)

        # Loading controllers and broadcaster from ROS2 Control

        # Regular expression pattern to find content between <parameters> and </parameters>
        pattern = r"<parameters>(.*?)</parameters>"

        # Search for the pattern
        match = re.search(pattern, robot_description_raw, re.DOTALL)

        # Check if a match is found
        if match:
            # Extract the content between <parameters> and </parameters>
            controllers_yaml_file_path = match.group(1)

            # Read the YAML file from the path
            try:
                with open(controllers_yaml_file_path, 'r') as file:
                    yaml_content = yaml.safe_load(file)

                    for controller_name, _ in yaml_content["/**"]["controller_manager"]["ros__parameters"].items():
                        if(controller_name != "update_rate"):

                            # Spawning controller/broadcaster
                            node_ros2_control = Node(
                                package='controller_manager',
                                executable='spawner',
                                namespace=model_name,
                                arguments=[controller_name],
                                output='screen',
                            )

                            nodes.append(node_ros2_control)

            except FileNotFoundError:
                print(f"File not found: {controllers_yaml_file_path}")
            except yaml.YAMLError as exc:
                print("Error parsing YAML content:", exc)

    return nodes