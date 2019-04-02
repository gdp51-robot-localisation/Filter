# -*- coding: utf-8 -*-
"""
Created on Sun Feb 03 11:35:04 2019

@author: Carol
"""
import numpy as np
from numpy.linalg import inv
#import matplotlib.pyplot as plt
import Beacon as beacon
import Fetch_Optic as opflow
import pdb
import array

def prediction(x, xdot, y, ydot, t, a):
    A = np.array([[1, t, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, t],
                  [0, 0, 0, 1]])
    X = np.array([[x],
                  [xdot],
                  [y],
                  [ydot]])
    B = np.array([[0.5 * t ** 2],
                  [t],
                  [0.5 * t ** 2],
                  [t]])
    
    X_prime = A.dot(X) + B.dot(a)
    return X_prime


def covariance4d(sigma1, sigma2, sigma3, sigma4):
    cov1_2 = sigma1 * sigma2
    cov2_1 = sigma2 * sigma1
    cov1_3 = sigma1 * sigma3
    cov3_1 = sigma3 * sigma1
    cov1_4 = sigma1 * sigma4
    cov4_1 = sigma4 * sigma1
    cov2_3 = sigma2 * sigma3
    cov3_2 = sigma3 * sigma2
    cov2_4 = sigma2 * sigma4
    cov4_2 = sigma4 * sigma2
    cov3_4 = sigma3 * sigma4
    cov4_3 = sigma4 * sigma3
    
    cov_matrix = np.array([[sigma1 ** 2, cov1_2, cov1_3, cov1_4],
                           [cov2_1, sigma2 ** 2, cov2_3, cov2_4],
                           [cov3_1, cov3_2, sigma3 ** 2, cov3_4],
                           [cov4_1, cov4_2, cov4_3, sigma4 ** 2]])
    
    return np.diag(np.diag(cov_matrix))

def covariance2d(sigma1, sigma2):
    cov1_2 = sigma1 * sigma2
    cov2_1 = sigma2 * sigma1
    
    cov_matrix = np.array([[sigma1 ** 2, cov1_2],
                           [cov2_1, sigma2 ** 2],
                           ])
    
    return np.diag(np.diag(cov_matrix))


def observation(X, z):
    a = 0.5         #to change weighting of beacon (higher, more reliant on beacon)
    
    obsb = beacon.fetch()
    obso = opflow.fetch()
    x_obs_b = obsb[1]
    x_obs_o = obso[0] + X[0][0]
    y_obs_b = obsb[2]
    y_obs_o = obso[1] + X[2][0]

    x_obs = a*x_obs_b + (1-a)*x_obs_o
    y_obs = a*y_obs_b + (1-a)*y_obs_o
    #pdb.set_trace()
    obs = np.array([x_obs, 0, y_obs, 0])
    #pdb.set_trace()
    z = np.vstack((z, obs))
    return z

def kalman(X, z):
    # ICs
    a = 0
    t = 0.1


    # Process / Estimation Errors
    error_est_x = 5
    error_est_xdot = 1
    error_est_y = 5
    error_est_ydot = 1

    # Observation Errors
    error_obs_x_b = 1  # Uncertainty in the measurement
    error_obs_x_o = 1
    error_obs_x = error_obs_x_b * error_obs_x_o
    error_obs_y_b = 1
    error_obs_y_o = 1
    error_obs_y = error_obs_x_b * error_obs_x_o
    
    #Initial State
    #X = np.array([0, 1, 0, 1])
    #pdb.set_trace()
    
    #Initial Estimation Covariance Matrix
    P = covariance4d(error_est_x, error_est_xdot, error_est_y, error_est_ydot)

    A = np.array([
        
            [1, t, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, t],
            [0, 0, 0, 1]

            ])
    

    n = len(z[0])

    for data in z[1:]:

        X = prediction(X[0][0], X[1][0], X[2][0], X[3][0], t, a)
        P = np.diag(np.diag(A.dot(P).dot(A.T)))
        H = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0]
            ])
        
        R = covariance2d(error_obs_x, error_obs_y)
        S = H.dot(P).dot(H.T) + R
        K = P.dot(H.T).dot(inv(S))
        Y = H.dot(data).reshape(2, -1)  #makes 2 row col matrix, measuring 2 vals (x, y)
        X = X + K.dot(Y - H.dot(X))
        P = (np.identity(len(K)) - K.dot(H)).dot(P)

    print("Kalman Filter State Matrix:\n", X)
    return X

