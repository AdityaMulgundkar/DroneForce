from dronekit import connect
from pymavlink import mavutil

import logging

class P_Controller:
    """ P Controller manager. """
    def __init__(self, Kp=0.25, *args, **kwargs):
        logging.info('P Controller initiated')

    def __enter__(self):
        return self

    def __exit__(self, *args):
        Kp = 0
        # Kill variables here