###### MEng Project Group 3
###### Title: Single Section Planar Continuum Robot Dynamic Model
###### Version: 2
###### Changes: New functions for non cylindrical backbone and disks, updated values to be representitive of materials
###### Based on the code by the same name from 23/24 project group 9

import numpy as np
import matplotlib.pyplot as plt
from math import pi, sin, cos

ModelParameters = {
    ###### Continuum Link Parameters ######
    'L': 0.5, # Continuum Link Length
    'alpha': 0.055, # Cable Radial Distance
    'n': 20, # Vertebrae Disk Count
    'h': 0.015789, # Vertebrae Disk Spacing
    
    ###### Backbone Rod Parameters ######
    'E_b': 79300000000, # Elasiticty Modulus AISI 1095
    'Rho_b': 7860, # Material Density
    'w_b' : 0.01, # Backbone width
    't_b' : 0.0005, # Backbone thickness

    ###### Actuation Cables Parameters ######
    'E_c': 1000000, # Elasiticty Modulus    
    'Rho_c': 950, # Material Density
    'r_c': 0.00025, # Radius
    
    ###### Vertebrae Disk Parameters ######
    # Disk shape is a cross with semicircle ends, with 4 axes of symmetry:
    #     _
    #  __| |__
    # (__   __)
    #    |_|
    #     
    'Rho_d': 1430, # Material Density
    'd_d': 0.010, # Distance from inner corner to start of semicircle
    'w_d': 0.005, # Width of one arm of the cross, diameter of semicircle
    's_d': 0.0005, # Thickness

    ###### General Model Parameters ######
    'g': 9.8 # Gravity Acceleration 
}

def CrossSectionAreaCyl(r):
    Ans = pi*(r**2)
    ### Calculates the cross-sectional area for a given radius 'r' ###
    return Ans
def CrossSectionAreaDisk (w, d):
    r = w/2
    Ans = w*w + 2*pi*(r**2) + 4*w*d
    ### Calculates the cross-sectional area of the cross support with given support width 'w' and length 'd'
    return Ans
def CrossSectionAreaRect (w, t):
    Ans = w*t
    ### Calculates the cross-sectional area of a rectangle given width 'w' and thickness 't'
    return Ans
def MomentOfInertiaCyl(r):
    Ans = (pi/4)*r**4
    ### Calculates the area moment of inertia for a given radius 'r' ###
    return Ans
def MomentOfInertiaDisk (w, d):
    r = w/2
    Ans = 1/12*(w**4 + 2*w**3*d + 2*w*d**3) + pi*r**4
    ### Calculates the area moment of inertia of the cross support with given support width 'w' and length 'd'
    return Ans
def MomentofInertiaRect (w, t):
    Ans = 1/3*w*t**3
    ### Calculates the area moment of inertia of a rectangle given width 'w' and thickness 't'
    return Ans
def DegreeToRadian(deg):
    rad = deg*(pi/180)
    ### Converts degrees to radian ###
    return rad

def InertiaComponent(Q, MP): ### Calculates the inertial value of the Continuum Robot, the equations used here are derived using the method detailed in the MEng Team 9 Final Report  
    L = MP.get('L')
    alpha = MP.get('alpha')
    ###### Rod ######
    w_b = MP.get('w_b')
    t_b = MP.get('t_b')
    Rho_b = MP.get('Rho_b')
    A_b = CrossSectionAreaRect(w_b, t_b)
    I_b = MomentofInertiaRect(w_b, t_b)
    Rod = (2*Rho_b*A_b*(L**3))/(3*(Q**2)) + (2*Rho_b*I_b*L)/3
    ###### Cable ######
    r_c = MP.get('r_c')
    Rho_c = MP.get('Rho_c')
    A_c = CrossSectionAreaCyl(r_c)
    I_c = MomentOfInertiaCyl(r_c)
    Cable = (2*Rho_c*A_c*(L**3))/(3*(Q**2)) + (2*Rho_c*I_c*L)/3 + 2*Rho_c*A_c*L*(alpha**2)
    ###### Disk ######
    n = MP.get('n')
    h = MP.get('h')
    s_d = MP.get('s_d')
    w_d = MP.get('w_d')
    d_d = MP.get('d_d')
    Rho_d = MP.get('Rho_d')
    A_d = CrossSectionAreaDisk(w_d, d_d)
    I_d = MomentOfInertiaDisk(w_d, d_d)
    Disk = 0
    for k in range(n):
        Ans = Rho_d*A_d*s_d*((k*h)/Q)**2 + I_d*((k*h)/L)**2
        Disk = Disk + Ans
    Inertia = Rod + Cable + Disk
    return Inertia
    
def CoriolisComponent(Q, Qdot, MP): ### Calculates the Coriolis Forces on the Continuum Robot, the equations used here are derived using the method detailed in the MEng Team 9 Final Report
    L = MP.get('L')
    alpha = MP.get('alpha')
    ###### Rod ######
    w_b = MP.get('w_b')
    t_b = MP.get('t_b')
    Rho_b = MP.get('Rho_b')
    A_b = CrossSectionAreaRect(w_b, t_b)
    I_b = MomentofInertiaRect(w_b, t_b)
    Rod = (-2*Rho_b*A_b*(L**3)*Qdot)/(3*(Q**3))
    ###### Cable ######
    r_c = MP.get('r_c')
    Rho_c = MP.get('Rho_c')
    A_c = CrossSectionAreaCyl(r_c)
    I_c = MomentOfInertiaCyl(r_c)
    Cable = (-2*Rho_c*A_c*(L**3)*Qdot)/(3*(Q**3))
    ###### Disk ######
    n = MP.get('n')
    h = MP.get('h')
    s_d = MP.get('s_d')
    w_d = MP.get('w_d')
    d_d = MP.get('d_d')
    Rho_d = MP.get('Rho_d')
    A_d = CrossSectionAreaDisk(w_d, d_d)
    I_d = MomentOfInertiaDisk(w_d, d_d)
    Disk = 0
    for k in range(n):
        Ans = Rho_d*A_d*s_d*((k*h*Qdot)**2/Q**3)
        Disk = Disk - Ans

    Coriolis = Rod + Cable + Disk
    return Coriolis

def GravityComponent(Q, MP): ### Calculates the gravity forces on the Continuum Robot, the equations used here are derived using the method detailed in the MEng Team 9 Final Report
    L = MP.get('L')
    alpha = MP.get('alpha')
    g = MP.get('g')
    ###### Rod ######
    w_b = MP.get('w_b')
    t_b = MP.get('t_b')
    Rho_b = MP.get('Rho_b')
    A_b = CrossSectionAreaRect(w_b, t_b)
    I_b = MomentofInertiaRect(w_b, t_b)
    Rod = Rho_b*A_b*g*(L**2)*(-1/(Q**2) + (2*sin(Q))/(Q**3) - cos(Q)/(Q**2))
    ###### Disk ######
    n = MP.get('n')
    h = MP.get('h')
    s_d = MP.get('s_d')
    w_d = MP.get('w_d')
    d_d = MP.get('d_d')
    Rho_d = MP.get('Rho_d')
    A_d = CrossSectionAreaDisk(w_d, d_d)
    I_d = MomentOfInertiaDisk(w_d, d_d)
    Disk = 0
    for k in range(n):
        Ans = Rho_d*A_d*s_d*g*L*(-1/(Q**2) + (((k*h)/L)*sin((k*h*Q)/L))/Q + (cos((k*h*Q)/L))/(Q**2))
        Disk = Disk + Ans            
    ###### Cable ######
    r_c = MP.get('r_c')
    E_b = MP.get('E_b')
    E_c = MP.get('E_c')
    I_c = MomentOfInertiaCyl(r_c)
    Elastic = (E_b*I_b*Q)/L + (2*E_c*I_c*Q)/L

    Gravity = Rod + Disk + Elastic
    return Gravity

def UpdateAngle(Qdot, Q, dt): # Updates the angle value using the forward euler method
    Q = Q + dt * Qdot
    return Q
def UpdateVelocity(Qddot, Qdot, dt): # Updates the velocity value using the forward euler method
    Qdot = Qdot + dt * Qddot
    return Qdot
def JointAcceleration(IC, CC, GC, Force, Qdot): # Updates the dynamics of the Continuum Robot
    Qddot = (Force - (CC*Qdot) - GC)/IC
    return Qddot

dt = 0.001
Runtime = 60
Samples = int(Runtime/dt)

### Stores the values used to plot the angle, velocity and force
theta_history = np.zeros(Samples - 1)
theta_dot_history = np.zeros(Samples - 1)
force_history = np.zeros(Samples - 1)
time_history = np.zeros(Samples - 1)

Q = np.zeros(Samples)
Qdot = np.zeros(Samples)
Qddot = np.zeros(Samples)
Force = np.zeros(Samples)

Q[0] = DegreeToRadian(1)

for i in range(Samples - 1):
    theta_history[i] = Q[i]
    theta_dot_history[i] = Qdot[i]
    force_history[i] = Force[i]
    time_history[i] = i

    IC = InertiaComponent(Q[i], ModelParameters)
    CC = CoriolisComponent(Q[i], Qdot[i], ModelParameters)
    GC = GravityComponent(Q[i], ModelParameters)

    Force[i+1] = IC*Qddot[i] + CC*Qdot[i] + GC   

    Qddot[i] = JointAcceleration(IC, CC, GC, Force[i], Qdot[i])
    Qdot[i+1] = UpdateVelocity(Qddot[i], Qdot[i], dt)
    Q[i+1] = UpdateAngle(Qdot[i+1], Q[i], dt)
    
### Plot the angle, velocity and forces on the Continuum Robot ###
plt.figure()
plt.subplot(311)
plt.plot(time_history, theta_history)
plt.title('Joint Angles (Theta)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.grid(True)

plt.subplot(312)
plt.plot(time_history, theta_dot_history)
plt.title('Joint Velocities (Theta Dot)')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (rad/s)')
plt.grid(True)

plt.subplot(313)
plt.plot(time_history, force_history)
plt.title('Joint Forces (F)')
plt.xlabel('Time (ms)')
plt.ylabel('Force (N)')
plt.grid(True)

plt.tight_layout()
plt.show()