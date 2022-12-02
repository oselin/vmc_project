#! /usr/bin/env python3
from dataclasses import dataclass
from BurgerRobot import BurgerRobot
from racer import ACTIVE_REGION
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rospy.numpy_msg import numpy_msg
from vmc_project.msg import Floats
import time

data = np.arange(20)

def callback(msg):
    global data, hp
    data = msg.values
    hp = msg.heatmap




#plt.axis("equal")
#plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
#tab2 = ax[1].bar(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION, np.zeros([int(180/ACTIVE_REGION)]))



rospy.init_node('listener', anonymous=True)
rospy.Subscriber('/data', numpy_msg(Floats), callback)


def animate(frameno):
    occmap = hp.reshape(70,70)
    ax[0].imshow(occmap,cmap = "PiYG_r")

    ax[1].cla()
    ax[1].set_xlim(0,200)
    ax[1].set_ylim(0,200)   
    ax[1].bar(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION+2,data)


fig, ax = plt.subplots(3, clear=True, figsize=(4, 14))
plt.axis("equal")
plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)


ani = FuncAnimation(fig, animate, interval=100)

plt.show()
