#!/usr/bin/python
from marvelmind_cy_mm import MarvelmindHedge
#from time import sleep
import timeit 
import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
import pdb
import smbus
import time as t
from numpy.linalg import inv
import matplotlib.pyplot as plt

##################
# Assume this is the main code so thread is made only one time
##################

#   There are two competing factors that cause repeated values:
#       1)  Time taken to scan through data/ finding relevant info = substantial
#           which means that one code is sorting through data while other master code
#           is requesting an updated reading (which has not been calculated yet) i.e.
#           the same value is fetched rather than a new one. This process is repeated
#           until the reading is updated.
#       2)  Sampling update rate of the beacon system @8Hz

hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=4, debug=False) # create MarvelmindHedge thread
hedge.start() # start thread
t.sleep(3)

bus = smbus.SMBus(1)    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

MD25_ADDRESS12 = 0x58      #7 bit address (will be left shifted to add the read write bit)
MD25_ADDRESS34 = 0x59 
SOFTWARE_REG = 0x0D
CMD = 0x10
RESET_ENCODERS = 0x20
SPEED1 = 0X00
SPEED2 = 0X01
MODE_SELECTOR = 0X0F
ACCELERATION = 0X0E
VOLT_READ = 0X0A

# Set up
bus.write_byte_data(MD25_ADDRESS12, MODE_SELECTOR, 1)
bus.write_byte_data(MD25_ADDRESS34, MODE_SELECTOR, 1)



def fetch():
    location = hedge.position()
    return location

initial = fetch()
X = np.array([initial[1], initial[2]])



def get_software():
    software58 = bus.read_byte_data(MD25_ADDRESS12, SOFTWARE_REG)
    software59 = bus.read_byte_data(MD25_ADDRESS34, SOFTWARE_REG)
    return(software58, software59)

def reset_Encoders():
    bus.write_byte_data(MD25_ADDRESS12, CMD, RESET_ENCODERS)
    bus.write_byte_data(MD25_ADDRESS34, CMD, RESET_ENCODERS)
    return

def encoder2():
    result = bus.read_i2c_block_data(MD25_ADDRESS12, 0x02, 11)
    max_int_32 = 2 ** 31
    
    encoder = (result[0] << 24) + (result[1] << 16) + \
               (result[2] << 8) + result[3]
    
    if encoder > max_int_32:
        encoder = -(2 ** 32 - encoder)
    
    encoder = encoder * 0.06109
    return(encoder)

def encoder1():
    result = bus.read_i2c_block_data(MD25_ADDRESS12, 0x02, 11)
    max_int_32 = 2 ** 31
    
    encoder = (result[4] << 24) + (result[5] << 16) + \
               (result[6] << 8) + result[7]
    
    if encoder > max_int_32:
        encoder = -(2 ** 32 - encoder)
    
    encoder = encoder * 0.06109
    return(encoder)

def encoder3():
    result = bus.read_i2c_block_data(MD25_ADDRESS34, 0x02, 11)
    max_int_32 = 2 ** 31
    
    encoder = (result[4] << 24) + (result[5] << 16) + \
               (result[6] << 8) + result[7]
    
    if encoder > max_int_32:
        encoder = -(2 ** 32 - encoder)
    
    encoder = encoder * 0.06109
    return(encoder)

def encoder4():
    result = bus.read_i2c_block_data(MD25_ADDRESS34, 0x02, 11)
    max_int_32 = 2 ** 31
    
    encoder = (result[0] << 24) + (result[1] << 16) + \
               (result[2] << 8) + result[3]
    
    if encoder > max_int_32:
        encoder = -(2 ** 32 - encoder)
    
    encoder = encoder * 0.06109
    return(encoder)



def stop():
    bus.write_byte_data(MD25_ADDRESS12, SPEED1, 0)
    bus.write_byte_data(MD25_ADDRESS12, SPEED2, 0)
    bus.write_byte_data(MD25_ADDRESS34, SPEED1, 0)
    bus.write_byte_data(MD25_ADDRESS34, SPEED2, 0)
    return



def prediction2d(x, xdot, y, ydot, t, a):
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


def covariance2d(sigma1, sigma2, sigma3, sigma4):
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

def kalman(X):
    obs = fetch()
    x_obs = obs[1]
    xdot_obs = 0.2
    y_obs = obs[2]
    ydot_obs = 0

    z = np.array([x_obs, xdot_obs, y_obs, ydot_obs])
    print(z)
    # ICs
    a = 2
    t = 0.1


    # Process / Estimation Errors
    error_est_x = 5
    error_est_xdot = 1
    error_est_y = 5
    error_est_ydot = 1

    # Observation Errors
    error_obs_x = 1  # Uncertainty in the measurement
    error_obs_xdot = 4
    error_obs_y = 1
    error_obs_ydot = 4

    #Initial State
    #X = np.array([0, 1, 0, 1])


    #Initial Estimation Covariance Matrix
    P = covariance2d(error_est_x, error_est_xdot, error_est_y, error_est_ydot)

    A = np.array([[1, t, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, t],
              [0, 0, 0, 1]])
    
    

    n = len(z)

    X = prediction2d(X[0], X[1], X[2], X[3], t, a)
    P = np.diag(np.diag(A.dot(P).dot(A.T)))
    H = np.identity(n)
    R = covariance2d(error_obs_x, error_obs_xdot, error_obs_y, error_obs_ydot,)
    S = H.dot(P).dot(H.T) + R
    K = P.dot(H).dot(inv(S))
    Y = H.dot(z).reshape(n, -1)
    X = X + K.dot(Y - H.dot(X))
    P = (np.identity(len(K)) - K.dot(H)).dot(P)

    print("Kalman Filter State Matrix:\n", X)
    return X

def drive(sp1, sp2, sp3, sp4, enc_dist):
    while (abs(encoder1()) < abs(enc_dist) and abs(encoder2()) < abs(enc_dist) and abs(encoder3()) < abs(enc_dist) and abs(encoder4()) < abs(enc_dist)):
        bus.write_byte_data(MD25_ADDRESS12, SPEED1, int(sp1))
        bus.write_byte_data(MD25_ADDRESS12, SPEED2, int(sp2))
        bus.write_byte_data(MD25_ADDRESS34, SPEED1, -int(sp3))
        bus.write_byte_data(MD25_ADDRESS34, SPEED2, -int(sp4))
        X = kalman(X)
    stop()
    reset_Encoders()
    t.sleep(0.05)
    return
    
def move(x, y, xpr, ypr, sp):
    x_dist = x - xpr
    y_dist = y - ypr
    
    if x_dist > 0:
        if y_dist > 0:
            if abs(x_dist) <= abs(y_dist):
                print("1")
                diag_dist = abs(x_dist * (2)**0.5)
                perp_dist = abs(y_dist) - abs(x_dist)
                print(perp_dist)
                drive(sp * (2)**0.5, sp * (2)**0.5, sp * (2)**0.5, sp * (2)**0.5, diag_dist * (2)**0.5)
                drive(0, 0, sp, sp, perp_dist)
            else:
                print("2")
                diag_dist = abs(y_dist * (2)**0.5)
                perp_dist = abs(x_dist) - abs(y_dist)
                
                drive(sp * (2)**0.5, sp * (2)**0.5, sp * (2)**0.5, sp * (2)**0.5, diag_dist * (2)**0.5)
                drive(sp, sp, 0, 0, perp_dist)
        elif y_dist < 0:
            if abs(x_dist) <= abs(y_dist):
                print("3")
                diag_dist = abs(x_dist * (2)**0.5)
                perp_dist = abs(y_dist) - abs(x_dist)
                
                drive(sp * (2)**0.5, sp * (2)**0.5, -sp * (2)**0.5, -sp * (2)**0.5, diag_dist * (2)**0.5)
                drive(0, 0, -sp, -sp, perp_dist)
            else:
                print("4")
                diag_dist = abs(y_dist * (2)**0.5)
                perp_dist = abs(x_dist) - abs(y_dist)
                print(perp_dist)
                drive(sp * (2)**0.5, sp * (2)**0.5, -sp * (2)**0.5, -sp * (2)**0.5, diag_dist * (2)**0.5)
                drive(sp, sp, 0, 0, perp_dist)
        else:
            print("5")
            drive(sp, sp, 0, 0, abs(x_dist))
    elif x_dist < 0:
        if y_dist > 0:
            if abs(x_dist) <= abs(y_dist):
                print("6")
                diag_dist = abs(x_dist * (2)**0.5)
                perp_dist = abs(y_dist) - abs(x_dist)
                
                drive(-sp * (2)**0.5, -sp * (2)**0.5, sp * (2)**0.5, sp * (2)**0.5, diag_dist * (2)**0.5)
                drive(0, 0, sp, sp, perp_dist)
            else:
                print("7")
                diag_dist = abs(y_dist * (2)**0.5)
                perp_dist = abs(x_dist) - abs(y_dist)
                
                drive(-sp * (2)**0.5, -sp * (2)**0.5, sp * (2)**0.5, sp * (2)**0.5, diag_dist * (2)**0.5)
                drive(-sp, -sp, 0, 0, perp_dist)
        elif y_dist < 0:
            if abs(x_dist) <= abs(y_dist):
                print("8")
                diag_dist = abs(x_dist * (2)**0.5)
                perp_dist = abs(y_dist) - abs(x_dist)
                
                drive(-sp * (2)**0.5, -sp * (2)**0.5, -sp * (2)**0.5, -sp * (2)**0.5, diag_dist * (2)**0.5)
                drive(0, 0, -sp, -sp, perp_dist)
            else:
                print("9")
                diag_dist = abs(y_dist * (2)**0.5)
                perp_dist = abs(x_dist) - abs(y_dist)
                
                drive(-sp * (2)**0.5, -sp * (2)**0.5, -sp * (2)**0.5, -sp * (2)**0.5, diag_dist * (2)**0.5)
                drive(-sp, -sp, 0, 0, perp_dist)
        else:
            print("10")
            drive(-sp, -sp, 0, 0, abs(x_dist))
    else:
        if y_dist > 0:
            drive(0, 0, sp, sp, abs(y_dist))
        else:
            drive(0, 0, -sp, -sp, abs(y_dist))
    return



print(X)
move(initial[1] + 10, initial[2], initial[1], initial[2], 10)



# Normal Code Here...

'''
for i in range(1,10):
    sleep(0.8)
    start=timeit.timeit()
    loc=fetch()
    end=timeit.timeit()
    elapse=end-start
    print '\n\n\n\n\n\n\n\n\n', loc, elapse, '\n\n\n\n\n\n\n\n\n'
hedge.stop()
'''

'''
start=timeit.timeit()
fetch()
end=timeit.timeit()
elapse=end-start
print elapse
hedge.stop()
'''
'''
# BELOW IS THE CODE FOR REPLACING REPEATED VALUES WITH AN ESTIMATE
# THIS USES A FIRST ORDER POLYNOMIAL SO IS ONLY USEFUL FOR STRAIGHT LINES
# AND A STATIONARY POSITION.
def compensate():
    bx=[]; by=[]; bz=[]; btime=[]; r_time=[]
    if len(a)>2:
        if a[-3]!=a[-4]:
            # will want to change range to start of straight line function
            for i in range(2,(len(a)-1)): # -1 to exclude repeated value in a=[...]
                bx.append(a[i][1])
                by.append(a[i][2])
                bz.append(a[i][3])
                btime.append(a[i][5])
            starttime=btime[0]
            for i in range(0,len(btime)):
                r_time.append(btime[i]-starttime)     
            zx=np.polyfit(r_time,bx,1)
            zy=np.polyfit(r_time,by,1)
            zz=np.polyfit(r_time,bz,1)
            dif_time=btime[-1]-btime[-2] # perhaps use mastercode time here instead
            next_r_time=r_time[-1]+dif_time # need relative time for where (x=0,y=C)
            timenew=btime[-1]+dif_time
            xnew=zx[0]*(next_r_time)+zx[1]
            ynew=zy[0]*(next_r_time)+zy[1]
            znew=zz[0]*(next_r_time)+zz[1]    
        else:
            print 'too many repeats'
    #print "x=%.6fx+(%.6f)"%(zx[0],zx[1])
    return [xnew,ynew,znew,timenew]

def plot():
    b=[];time=[]
    for i in range(1,len(a)):
        b.append(a[i][1])
        time.append(a[i][5])
    z=np.polyfit(time,b,1)
    p=np.poly1d(z)
    print "y=%.6fx+(%.6f)"%(z[0],z[1])
    plt.plot(time,b,'x')
    plt.plot(time,p(time),'--')
    plt.show()


def ploty():
    b=[];time=[]
    for i in range(1,len(a)):
        b.append(a[i][2])
        time.append(a[i][5])
    z=np.polyfit(time,b,1)
    p=np.poly1d(z)
    print "y=%.6fx+(%.6f)"%(z[0],z[1])
    plt.plot(time,b,'x')
    plt.plot(time,p(time),'--')
    plt.show()

a=[]
while True:
    try:
        sleep(0.25)
        a.append(hedge.position())
        if len(a)<3:
            print 'storing initial samples'
        elif a[-1]==a[-2]:
            #print 'REPEATED VALUE!!!'
            #pdb.set_trace()
            XT=compensate()
            estimate=[a[-1][0],int(XT[0]),int(XT[1]),int(XT[2]),'unknown rotation',XT[3]]
            a.append(estimate)
            del a[-2]
            print 'Repeat Replaced with Estimate:', estimate
        else:
            print hedge.position()
    except:
        hedge.stop()  # stop and close serial port
        sys.exit()
        print 'smt wrong'
'''
