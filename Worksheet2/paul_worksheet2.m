function finalRad = myFirstControlProgram(robotObj)
       
    % Set constants for this program
    maxDuration= 5;  % Max time to allow the program to run (s)
    l = 3;
    r = 2;
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    phi1 = 3; phi2 = 1;
    ang_vel = r*(phi1 - phi2)/(2*l);
    fwd_vel = r*(phi1 + phi2)/2;
    
    % Set initial robot location
    setMapStart(robotObj,[0 0 0]);
    
    % Start robot driving forward
    SetFwdVelAngVel(robotObj,fwd_vel/10,ang_vel);
    
    % Enter main loop
    while ((toc(tStart) < maxDuration) && (autoCheck(robotObj)))
        %TurnAngle(robotObj,0.05,90);
        
        % Briefly pause to avoid continuous loop iteration
        pause(1)
    end
    
    % Set constants for this program
    maxDuration= 10;  % Max time to allow the program to run (s)
    l = 3;
    r = 2;
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    phi1 = 2; phi2 = 2;
    ang_vel = r*(phi1 - phi2)/(2*l);
    fwd_vel = r*(phi1 + phi2)/2;
    
    % Set initial robot location
    %setMapStart(robotObj,[0 0 0]);
    
    % Start robot driving forward
    SetFwdVelAngVel(robotObj,fwd_vel/10,ang_vel);
    
    % Enter main loop
    while ((toc(tStart) < maxDuration) && (autoCheck(robotObj)))
        %TurnAngle(robotObj,0.05,90);
        
        % Briefly pause to avoid continuous loop iteration
        pause(1)
    end
    
    % Set constants for this program
    maxDuration= 1;  % Max time to allow the program to run (s)
    l = 3;
    r = 2;
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    phi1 = 4; phi2 = -4;
    ang_vel = r*(phi1 - phi2)/(2*l);
    fwd_vel = r*(phi1 + phi2)/2;
    
    % Set initial robot location
    %setMapStart(robotObj,[0 0 0]);
    
    % Start robot driving forward
    SetFwdVelAngVel(robotObj,fwd_vel/10,ang_vel);
    
    % Enter main loop
    while ((toc(tStart) < maxDuration) && (autoCheck(robotObj)))
        %TurnAngle(robotObj,0.05,90);
        
        % Briefly pause to avoid continuous loop iteration
        pause(1)
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