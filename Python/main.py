# -------------------------------------------------------------------------------------------------
# MPC Autonomous Racing 
# Berkeley Hybrid Robotics Lab
# Author : Johnathon Li (johnsjli@berkeley.edu)
# Code foundation from Ugo Rosolia (ugo.rosolia@berkeley.edu)
# -------------------------------------------------------------------------------------------------

import sys
sys.path.append('func')

from casadi import *
from Track import *

# Foundation for casadi code : racecar.py and rocket.py 

# Variable Initializations -------------------------------

N = 10 # Horizon

# Vehicle Parameters (Borrowed from Ugo's MPC Code)
g = 9.81 # gravity
m  = 1.98
lf = 0.125
lr = 0.125
Iz = 0.024
Df = 0.8 * m * g / 2.0
Cf = 1.25
Bf = 1.0
Dr = 0.8 * m * g / 2.0
Cr = 1.25
Br = 1.0    

# Controls
u = MX.sym("u",2,N)
delta = u[0,:] # Steering Inputs
accel = u[1,:] # Engine Inputs

# States
x = MX.sym("x",6,N) # hmmm....cannot do N+1 when u is N long 
s = x[0,:] # track progress
e_psi = x[1,:] # heading error wrt/road frame
e_lat = x[2,:] # lateral error from track centerline
vx = x[3,:] # velocity in vehicle x axis
vy = x[4,:] # velocity in vehicle y axis
wz = x[5,:] # vehile yaw wrt/road frame

# Tire Split Angles 
alpha_f = atan2(vx, vy + lf*wz) - delta
alpha_r = atan2(vx, vy - lr*wz)

F_nf = lf/(lf+lr)*m*g # Front Tire Normal Load
F_nr = lr/(lf+lr)*m*g # Rear Tire Normal Load

F_yf = F_nf*Df*sin( Cf * atan2(1, Bf*alpha_f)) # Front Tire Lateral Force
F_yr = F_nr*Dr*sin( Cr * atan2(1, Br*alpha_r)) # Rear Tire Lateral Force

# ODEs
ds = (vx*cos(e_psi) - vy*sin(e_psi))/(1-)
de_psi =
de_lat = 
dvx = 
dvy = 


# --------------------------------------------------------
