% function garbage = EKF_loc(x_t, P_p, u_t, z_t, m)
%%% Worksheet 7
% EKF Localization
% Steve Guerrero and Paul Vuong
clc;
clear;
close all;

% This script will implement Extended Kalman Filter (EKF) Localization for the robot
x = -2.5;   % x position
y = -3;     % y position
theta = (pi/2); % theta of robot pose
x_t = [x; y; theta];

P_p = [0 0 0;
       0 0 0;
       0 0 0];
   
sigmer = 0.6;  % std dev error for movement
v_t = 0.5;  % linear velocity in cm/sec
w_t = 0.0001; % angular velocity in cm/sec
t = 3;          % time in seconds
alf_1 = 0.1;
alf_2 = 1;
alf_3 = 0.1;
alf_4 = 1;

F_x = [1 0 -((v_t/w_t)*cos(theta) + (v_t/w_t)*cos(theta + (w_t*t)));
    0 1 -((v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + (w_t*t)));
    0 0 1];

F_u = [((-sin(theta) + sin(theta + (w_t*t)))/w_t) (v_t*((sin(theta) - sin(theta + (w_t*t)))/(w_t^2)) + v_t*((cos(theta + (w_t*t))*t)/w_t));
    ((cos(theta) - cos(theta + (w_t*t)))/w_t) (-v_t*((cos(theta) - cos(theta + (w_t*t)))/(w_t^2)) + v_t*((sin(theta + (w_t*t))*t)/w_t));
    0 t];

Q_t = [(alf_1*(v_t^2) + alf_2*(v_t^2)) 0;
    0 (alf_3*(v_t^2) + alf_4*(v_t^2))];

X_hat = [(-(v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + (w_t*t)));
        ((v_t/w_t)*cos(theta) - (v_t/w_t)*cos(theta + (w_t*t)));
        (w_t*t)];
    
P_t = F_x*P_p*(F_x') + F_u *(F_u');

R = [(sigmer^2) 0 0;
    0  (sigmer^2) 0;
    0 0 (sigmer^2)];

garbage = P_t;
% end