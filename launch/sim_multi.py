from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    # Map
    sl.include('map_simulator', 'simulation2d_launch.py',
               launch_arguments={'map': sl.find('mobro_sim', 'void.yaml'),
                                 'display': False})      
    sl.node('rviz2', arguments=['-d', sl.find('mobro_sim', 'config_multi.rviz')])
    #sl.node('rqt_reconfigure')
    
    # Robot
    sl.declare_arg('robot', default_value='bike', description='r2d2 / bike')
    with sl.group(ns='robot'):
        sl.robot_state_publisher('mobro_sim', sl.name_join(sl.arg('robot'), '.urdf'), 'urdf')
        sl.node('map_simulator', 'spawn', parameters = {'static_tf_odom': True, 'force_laser': False})
        sl.node('map_simulator', 'kinematics.py')
        
    # Obstacle 1
    sl.declare_arg('obs1', default_value='r2d2_obs1')
    with sl.group(ns='obs1'):
        sl.robot_state_publisher('mobro_sim', sl.name_join(sl.arg('obs1'), '.urdf'), 'urdf')
        sl.node('map_simulator', 'spawn', parameters = {'static_tf_odom': True, 'force_laser': False})
        sl.node('map_simulator', 'kinematics.py')

    # Obstacle 2
    sl.declare_arg('obs2', default_value='r2d2_obs2')
    with sl.group(ns='obs2'):
        sl.robot_state_publisher('mobro_sim', sl.name_join(sl.arg('obs2'), '.urdf'), 'urdf')
        sl.node('map_simulator', 'spawn', parameters = {'static_tf_odom': True, 'force_laser': False})
        sl.node('map_simulator', 'kinematics.py')

    return sl.launch_description()
