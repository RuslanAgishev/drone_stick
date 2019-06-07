#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 0.9

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=5.0)
    timeHelper.sleep(5.0)
    
    # for cf in allcfs.crazyflies:
    #     print 'go to'
    #     cf.goTo(np.array([0.8,0.8,Z]), 0, 3.0)
    # timeHelper.sleep(4.0)

    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    allcfs.land(targetHeight=0.02, duration=8.0)
    timeHelper.sleep(4.0)
