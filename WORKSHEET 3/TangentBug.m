function finalRad = TangentBug(robotObj)
    % Set constants for this program
    maxDuration= 1200;  % Max time to allow the program to run (s)    
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    
    % Set initial robot location
    setMapStart(robotObj,[-2 -2 0]);
    % initial poses of robot
    posx = -2;
    posy = -2;
    
    % setting up both goals/targets
    T1 = [1,2];
    T2 = [1,-2];
    F = ReadIR(robotObj,2)
    [dmin,theta] = dminT_theta(posx,posy, T1(1), T1(2));
    
    % Start robot driving forward
    %SetFwdVelAngVel(robotObj,0.3,theta);
    TurnAngle(robotObj,0.2,theta);  
  
    % Enter main loop for target 1
    while ((toc(tStart) < maxDuration) && (autoCheck(robotObj)))
        while (dmin > 0.02) % while distance from goal is greater than 2 cm(close enough)
            F1 = ReadIR(robotObj,2);
            F2 = ReadIR(robotObj,2);
            F3 = ReadIR(robotObj,2);
            F = (F1 + F2 + F3)/3;
            if (F < 0.25)   % obstacle detected, hug boundary of obstacle
               [robotObj] =  Hugs(robotObj, T1);
            else
               [robotObj] = str8_shot(robotObj, T1);
            end
            [robotObj] = str8_shot(robotObj, T1);
            [x, y, ~] = OverheadLocalization(robotObj);
            [dmin,~] = dminT_theta(x,y, T1(1), T1(2));
        end
        pause(0.1)
    end
    
end

function w = v2w(v)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    % Robot constants
    maxWheelVel= 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius= 0.08;   % Radius of the robot (m)

    % Max velocity combinations obey rule v+wr <= v_max
    w= (maxWheelVel-v)/robotRadius;
end