from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    # Map
    sl.include('map_simulator', 'simulation2d_launch.py',
               launch_arguments={'map': sl.find('mobro_sim', 'map.yaml'), 'map_server': True, 'display': False})
    
    # AMCL
    sl.include('mobro_sim', 'amcl_launch.py')
    
    # Rviz 
    sl.node('rviz2', arguments=['-d', sl.find('mobro_sim', 'config_rosbot_amcl.rviz')])
    #sl.node('rqt_reconfigure')

    # Obstacle 1 
    sl.declare_arg('obs1', default_value='r2d2_obs1')
    with sl.group(ns = 'obs1'):
        sl.robot_state_publisher('mobro_sim', sl.name_join(sl.arg('obs1'), '.urdf'), 'urdf')
        sl.node('map_simulator', 'spawn', parameters = {'static_tf_odom': True, 'force_scanner': False})
        sl.node('map_simulator', 'kinematics.py')

    return sl.launch_description()
