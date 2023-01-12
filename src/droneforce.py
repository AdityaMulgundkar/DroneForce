"""
TODO: DroneForce class, can be imported to setup the drone and call its base functions.
"""
from src.utility.logger import *

class DroneForce:
    """ A DroneForce connection manager. """
    def __init__(self, frame, mass, inertia, *args, **kwargs):
        self.alive = True
        self.frame = frame
        self.mass = mass
        self.inertia = inertia
        logging.info('Drone connection successful')

    def __enter__(self):
        ''' Send regular heartbeats while using in a context manager. '''
        self.alive = True
        logging.info('__enter__ -> reviving heart (if required)')
       
        return self

    def __exit__(self, *args):
        ''' Automatically disarm and stop heartbeats on error/context-close. '''
        self.alive = False
        logging.info('__exit__ -> disarming, stopping heart and closing connection')
