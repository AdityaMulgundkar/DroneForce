import time
from dronekit import connect
from pymavlink import mavutil

import logging

class DFAutopilot:
    """ A Dronekit autopilot connection manager. """
    def __init__(self, connection_string, *args, **kwargs):
        logging.debug('connecting to Drone (or SITL/HITL) on: %s', connection_string)
        self.alive = True
        self.master = connect(connection_string, wait_ready=True, rate=20)
        self.mavutil = mavutil.mavlink_connection('127.0.0.1:14552')
        # Add a heartbeat listener
        # Func for heartbeat
        def heartbeat_listener(_self, name, msg):
            self.last_heartbeat = msg

        self.heart = heartbeat_listener

        logging.info('Drone connection successful')

    def __enter__(self):
        ''' Send regular heartbeats while using in a context manager. '''
        self.alive = True
        logging.info('__enter__ -> reviving heart (if required)')
        # Listen to the heartbeat
        self.master.add_message_listener('HEARTBEAT', self.heart)
       
        return self

    def __exit__(self, *args):
        ''' Automatically disarm and stop heartbeats on error/context-close. '''
        self.alive = False
        logging.info('__exit__ -> disarming, stopping heart and closing connection')
        # TODO: add reset parameters procedure. Can be based on flag?
        # Disarm if not disarmed
        if self.master.armed:
            self.master.armed = False
        # Kill heartbeat
        self.master.remove_message_listener('HEARTBEAT', self.heart)
        # Close Drone connection
        logging.info('disconnect -> closing Drone connection') 
        self.master.close()

    def set_servo(self, motor_num, pwm_value):
        pwm_value_int = int(pwm_value)
        msg = self.master.message_factory.command_long_encode(
                    0, 0, 
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,
                    motor_num,
                    pwm_value_int,
                    0,0,0,0,0
                    )
        self.master.send_mavlink(msg)

    def set_motor_mode(self, motor_num, set_reset):
        # get servo function - what this motor does
        logging.debug("PRE SET PARAM %s: %s", f'SERVO{motor_num}_FUNCTION', self.master.parameters[f'SERVO{motor_num}_FUNCTION'])
        time.sleep(0.1)

        # set servo function - change to 1 for RCPassThru
        self.master.parameters[f'SERVO{motor_num}_FUNCTION'] = set_reset
        time.sleep(0.1)

        # get servo function - what this motor does
        logging.debug("POST SET PARAM %s: %s", f'SERVO{motor_num}_FUNCTION', self.master.parameters[f'SERVO{motor_num}_FUNCTION'])
        time.sleep(0.1)