#! /usr/bin/env python3
from BurgerRobot import BurgerRobot
from racer import ACTIVE_REGION
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from rospy.numpy_msg import numpy_msg
from vmc_project.msg import Floats

def callback(msg):
    
    

    #ax[0].imshow(occmap,cmap = "PiYG_r")
    
    #ax[1].bar(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION+2,myhistogram2)
    #ax[2].plot(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION,myhistogram)
    #ax[2].plot(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION,myhistogram2)
    #plt.show()
    """for rec,h in zip(tab2, msg.values):
        rec.height(h)"""

    #rate.sleep()



plt.axis("equal")
plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
tab2 = ax[1].bar(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION, np.zeros([int(180/ACTIVE_REGION)]))



rospy.init_node('listener', anonymous=True)
rospy.Subscriber('/data', numpy_msg(Floats), callback)
# rospy.spin()

def animate(frameno):
    x = mu + sigma * np.random.randn(N)
    n, _ = np.histogram(x, bins, normed=True)
    for rect, h in zip(patches, n):
        rect.set_height(h)
    return patches

N, mu, sigma = 10000, 100, 15
fig, ax = plt.subplots(1,3, clear=True, figsize=(14, 4))
x = mu + sigma * np.random.randn(N)
n, bins, patches = plt.hist(x, 50, normed=1, facecolor='green', alpha=0.75)

frames = 100
ani = animation.FuncAnimation(fig, animate, blit=True, interval=0,
                              frames=frames,
                              repeat=True)
plt.show()