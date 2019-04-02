# -*- coding: utf-8 -*-
"""
Created on Sat Feb 23 17:38:33 2019

@author: Carol
"""

import Beacon as beacon
import Fetch_Optic as opflow
import Motor as motor
import Kalman as kal
import numpy as np
import pdb

opflow.optic_setup()
beacon.beacon_setup()
initial = beacon.fetch()
X = np.array([[initial[1]],[0.2],[initial[2]],[0]])
z = np.array([0.0, 0.0, 0.0, 0.0])

print(X)
motor.move(initial[1] + 30, initial[2], initial[1], initial[2], 10, z, X)
motor.move(0, 30, 0, 0, 10, z, X)
