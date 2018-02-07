function finalRad = myFirstControlProgram(robotObj)
    % Set constants for this program
    maxDuration= 5;  % Max time to allow the program to run (s)    
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    phi_1 = 3;
    phi_2 = 1;
    [ang_vel, fwd_vel] = control_program(phi_1, phi_2);
    % Set initial robot location
    setMapStart(robotObj,[0 0 0]);
    
    % Start robot driving forward
    SetFwdVelAngVel(robotObj,fwd_vel,ang_vel);
    
    % Enter main loop
    while ((toc(tStart) < maxDuration) && (autoCheck(robotObj)))
        %turnAngle(serPort,0.05,90);

        % Briefly pause to avoid continuous loop iteration
        pause(1)
    end
    
    maxDuration= 10;  % Max time to allow the program to run (s)       
    tStart= tic;        % Time limit marker
    % Start robot driving forward
    
    phi_1 = 2;
    phi_2 = 2;
    [ang_vel_2, fwd_vel_2] = control_program(phi_1, phi_2);
    
    SetFwdVelAngVel(robotObj,fwd_vel_2,ang_vel_2);
    
    while ((toc(tStart) < maxDuration) && (autoCheck(robotObj)))
    %turnAngle(serPort,0.05,90);

    % Briefly pause to avoid continuous loop iteration
    pause(1)
    end
    
    maxDuration= 1;  % Max time to allow the program to run (s)     
    tStart= tic;        % Time limit marker
    % Start robot driving forward
    phi_1 = 4;
    phi_2 = -4;
    [ang_vel, fwd_vel] = control_program(phi_1, phi_2);
    
    SetFwdVelAngVel(robotObj,fwd_vel,ang_vel);
    
    while ((toc(tStart) < maxDuration) && (autoCheck(robotObj)))
    %turnAngle(serPort,0.05,90);

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


