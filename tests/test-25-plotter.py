import math
import os
import sys
import time

import pandas as pd

import bagpy
from bagpy import bagreader


cur_path=os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, cur_path+"/..")

from src.utility.map_range import map_range, torque_to_PWM

import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    start_time = time.time() 
    b = bagreader('dist/2023-01-24-18-34-05.bag')

    # get the list of topics
    # print(b.topic_table)
    x = list(b.topics)[0]
    print(f"Topic: {x}")

    csvfiles = []
    for t in b.topics:
        data = b.message_by_topic(t)
        csvfiles.append(data)

    vel = pd.read_csv(csvfiles[0])
    fig, ax = bagpy.create_fig(1)
    ax[0].scatter(x='position.y', y='position.x', data=vel)
    plt.show()