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

def callback(msg):
    global data
    data = msg.values
    

    #ax[0].imshow(occmap,cmap = "PiYG_r")
    
    #ax[1].bar(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION+2,myhistogram2)
    #ax[2].plot(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION,myhistogram)
    #ax[2].plot(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION,myhistogram2)
    #plt.show()
    """for rec,h in zip(tab2, msg.values):
        rec.height(h)"""

    #rate.sleep()



#plt.axis("equal")
#plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
#tab2 = ax[1].bar(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION, np.zeros([int(180/ACTIVE_REGION)]))



rospy.init_node('listener', anonymous=True)
rospy.Subscriber('/data', numpy_msg(Floats), callback)


def animate(frameno):
    axes.cla()
    axes.set_xlim(0,200)
    axes.set_ylim(0,200)   
    axes.bar(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION+2,data)

    time.sleep(0.1)

fig = plt.figure(figsize=(7,5))
axes = fig.add_subplot(1,1,1)



ani = FuncAnimation(fig, animate, interval=100)

plt.show()