from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    robot = 'robot_2'
        
    with sl.group(ns=robot):
        # Launch AMCL node with parameter file
        sl.node('nav2_amcl', 'amcl', name='amcl',
                parameters=[sl.find('mobro_sim', 'amcl_param.yaml')],
                remappings={'map':'/map', '/scan':'scan'},
                arguments='--ros-args --log-level warn')

        # Run lifecycle manager just for AMCL
        sl.node('nav2_lifecycle_manager', 'lifecycle_manager', name='lifecycle_manager',
        parameters={'autostart': True, 'node_names': ['amcl']})
                    
    return sl.launch_description()
