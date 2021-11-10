from pypot.dynamixel import DxlIO

motors_types = {
    1: 'MX_64', #https://emanual.robotis.com/docs/en/dxl/mx/mx-64/
    2: 'MX_64', # ^
    3: 'MX_28', #https://emanual.robotis.com/docs/en/dxl/mx/mx-28/
    4: 'AX_12A', #https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
    5: 'AX_18A', # ^
    6: 'MX_28' # ^^^
}

reachy_robot_config = {
    'controllers': {
        'my_dxl_controller': {
            'sync_read': False,
            'attached_motors': ['wrist'],
            'port': 'auto'
        }
    },
    'motorgroups': {
        'wrist': ['gripper', 'wrist_pitch', 'wrist_roll'],
    },
    'motors': {
        'gripper':{
            'orientation': 'direct',
            'id':17,
            'type':'AX_18A',
            'angle_limit':(-69, 20),
            'offset': 0.0,
            'moving_speed': 20,
                },
        'wrist_pitch':{
            'orientation': 'indirect',
            'id':15,
            'type':'AX_18A',
            'angle_limit':(-45, 45),
            'offset': 0.0,
            'moving_speed': 80,
        },
        'wrist_roll':{
            'orientation': 'indirect',
            'id':16,
            'type':'AX_18A',
            'angle_limit':(-45, 45),
            'offset': 0.0,
            'moving_speed': 80,
        },
    }
}

# assemble config to get motor min and maxes to be used when controlling
control_config = {}
for motor in reachy_robot_config['motors']:
    angle_limit = reachy_robot_config['motors'][motor]['angle_limit']
    control_config[motor] = {} # create dictionary for controlling motor max, min
    control_config[motor]['max'] = angle_limit[1]
    control_config[motor]['min'] = angle_limit[0]


def get_motor_config(motor_number, motor_name):
    """ use to get the motor config from the robot
    
    Args:
        motor_number: motor number that want to get data from
        motor_name: name of motor getting data from    
    """

    port = "/dev/ttyUSB"
    dxl_io = DxlIO(port, baudrate=1000000)

    angle_limits = dxl_io.get_angle_limit((motor_number,))[0]
    motor_type = motors_types[motor_number]

    print("'"+motor_name+"':{")
    print("'orientation': 'direct',")
    print("'id':" + str(motor_number) + ",")
    print("'type':'"+str(motor_type)+"',")
    print("'angle_limit':" + str(angle_limits) + ",")
    print("'offset': 0.0,")
    print("}")
