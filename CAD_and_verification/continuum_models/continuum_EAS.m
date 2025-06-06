% Author: Maz Kane
% Version: 1
% Model based on Euler curves and Hooke's law to account for external
% forces at tip (mass of inching unit)

% Assumptions: no shear, twist, elongation or friction expereienced by
% backbone; spacer disks have no mass; inching unit weighs 3kg; single
% tendon providing moment

% Declare material and robot constants
E = 210*10^9; %Young's modulus of C75S steel (backbone) in Pa
n = 20; %Number of tendon guide disks
m = 0.01; %Mass of tendon guide disks in kg
l = 0.31; %Total length of backbone in m
k = 2; %Number of tendons
F_e = 3*9.81; %Euclidian value of external force acting on end effector
alpha = -3.14/2; %Angle between F_e and x_0
T = 50; %Force on tendon provided by 
d = 0.01; %Distance from backbone to a tendon
%Set up base frame - used as global frame
p_1 = [0; 0; 0]; %Origin of first disk
x_1 = [1; 0; 0]; %Initial x axis
y_1 = [0; 1; 0]; %Initial y axis
z_1 = [0; 0; 1]; %Initial z axis
%Set up desired end effector position and frame (in global frame)
p_n = [0.25; 0; 0.25];
x_n = [0; -1; 0];
y_n = [0; 1; 0];
z_n = [1; 0; 0];
%Calculate moment on backbone from tendon (constant)
m_t = d*T;
%Calculate initial and final moments
m_O_1 = cross((p_n - p_1), F_e*[cos(3*pi/2); 0; sin(3*pi/2)])+m_t*[0; 1; 0];
m_O_n = m_t*[0; 1; 0];
%Create simultaneous equations from m-E*I*kappa = 0
