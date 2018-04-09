% % ECPE 155 Worksheet 6
%% Localization worksheet
clc;
clear;
close all;
% initialize EKF with known values
t  = 3;
v_t = 0.5; %forward velocity
w_t = 0.0001; %angular velocity
x_t = [-2.5;-3;pi/2];
theta = x_t(3);
sigmer = 0.08;
P_p = [0 0 0;
       0 0 0;
       0 0 0];

% Initialize map
dataMap = csvread('worksheet6Map.csv',1,0);
% Determine lines in map data and midpoints in polar form
[malpha] = midPointMap(dataMap);

% Initialize obstacles
dataObs = csvread('worksheet6obs.csv',1,0);
% Determine lines from obstacle data and line midpoints in polar form.
z = midLine(dataObs);

% Gt: Jacobian of movement model w/respect to pose
F_x = [1 0 (-((v_t/w_t)*cos(theta) + (v_t/w_t)*cos(theta + (w_t*t))));
       0 1 (-((v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + (w_t*t))));
       0 0 1];
   
% Vt: Jacobian of movement w/respect to motion measurement
F_u = [((-sin(theta) + sin(theta + (w_t*t)))/w_t) (v_t*((sin(theta) - sin(theta + (w_t*t)))/(w_t^2)) + v_t*((cos(theta + (w_t*t))*t)/w_t));
       ((cos(theta) - cos(theta + (w_t*t)))/w_t) (-v_t*((cos(theta) - cos(theta + (w_t*t)))/(w_t^2)) + v_t*((sin(theta + (w_t*t))*t)/w_t));
       0 t];
   
% Mt: Assumptions for now until tried
Q_t = ([0.6 0;
       0 0.6]);

% Pose estimate
Xt_hat = x_t + [(-(v_t/w_t)*sin(theta) + (v_t/w_t)*sin(theta + (w_t*t)));
               ((v_t/w_t)*cos(theta) - (v_t/w_t)*cos(theta + (w_t*t)));
               (w_t*t)];
           
% Prediction update   
P_t = (F_x*P_p*(F_x')) + (F_u *Q_t*(F_u'));

% Modeling error
R = [(sigmer^2) 0 0;
     0  (sigmer^2) 0;
     0 0 (sigmer^2)];


% for all observed features, z
for  i=1:1:2;
    
    % for all landmarks k in map, m
    for k=1:1:16;
        
        q(k) = (malpha(k,1)*cos(malpha(k,2))-Xt_hat(1))^2 + (malpha(k,1)*sin(malpha(k,2))-Xt_hat(2)).^2;
        
        zt_hat = ([sqrt(q(k)); 
                   atan2((malpha(k,1)*sin(malpha(k,2))-Xt_hat(2)),(malpha(k,1)*cos(malpha(k,2))-Xt_hat(1)))-Xt_hat(3)]); 
              
        l(k).Ht = [-(malpha(k,1)*cos(malpha(k,2))-Xt_hat(1))/sqrt(q(k)), -(malpha(k,1)*sin(malpha(k,2))-Xt_hat(2))/sqrt(q(k)), 0;
                   (malpha(k,1)*sin(malpha(k,2))-Xt_hat(2))/q(k), -(malpha(k,1)*cos(malpha(k,2))-Xt_hat(1))/q(k), -1;
                   0, 0, 1];
                  
        l(k).Inn = l(k).Ht*P_t*l(k).Ht' + R;
    end
    
    for ii = 1:15
        if((det(l(ii).Ht)) > (det(l(ii+1).Ht)))
            ff = (det(l(ii).Ht));
        end
    end
    
    l(i).j = (2*pi*ff^(-0.5)*exp((-0.5)*(z(i)-zt_hat(i)).'*l(i).Inn^-1*(z(i)-zt_hat(i))));
    
    l(i).Kt =  Xt_hat'*(l(i).Ht)*(l(i).Inn)^-1;
    
    Xt_hat = Xt_hat'+l(i).Kt*(z(i)-zt_hat(i));
    
    P_t = (1-l(i).Kt*l(i).Ht)*P_t;    
    
end

x_t = Xt_hat;
P_p = P_t;


