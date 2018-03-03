function [slowbot] = Hugs(slowbot, T1)
% this function runs the state in which the robots hugs the border of the
% obstacle and returns when the robot has passed the edge or passed away,
% which ever comes first

Tx = T1(1);
Ty = T1(2);
[x, y, theta_c] = OverheadLocalization(slowbot);
[~,theta] = dminT_theta(x,y, T1(1), T1(2));

while ((abs(abs(theta_c)-abs(theta))) > 0.1)   % robot SHOULD hug the boundary until its facing the target

    % the depending on the appraoch to the obstacle the robot will choose a
    % path that provide the shortest diatance to the target
    xR = x + (.1*cos(theta_c - (pi/2)));
    yR = y + (.1*sin(theta_c - (pi/2)));
    
    xL = x + (.1*cos(theta_c + (pi/2)));
    yL = y + (.1*sin(theta_c + (pi/2)));

    [dminR,~] = dminT_theta(xR,yR, Tx, Ty);    % min distance form center of robot
    [dminL,~] = dminT_theta(xL,yL, Tx, Ty);    % min distance form center of robot

    if (dminR < dminL) % rotate about 5 degs( up to 180 ) CW until F sensor doesn't read obstacle
        for i = 0:35
        TurnAngle(slowbot,0.2,-0.0872665);
        [~, F1, L1] = ReadIRMultiple(slowbot);
        [~, F2, L2] = ReadIRMultiple(slowbot);
        [~, F3, L3] = ReadIRMultiple(slowbot);
        F = (F1 + F2 + F3)/3;
        L = (L1 + L2 + L3)/3;
            if( (F > 0.4) && ( L  < 0.3 ) )
                break;
            end
        end
    else    % rotate about 5 degs ccw until front sensor doesn't read obstacle
        for i = 0:35
        TurnAngle(slowbot,0.2,0.0872665);
        [R1, F1, ~] = ReadIRMultiple(slowbot);
        [R2, F2, ~] = ReadIRMultiple(slowbot);
        [R3, F3, ~] = ReadIRMultiple(slowbot);

        R = (R1 + R2 + R3)/3;
        F = (F1 + F2 + F3)/3;

            if((F > 0.4) && (R < 0.3))
                break;
            end
        end
    end

    % Hoping to take an average because of pesky noise
    [R1, F1, L1] = ReadIRMultiple(slowbot);
    [R2, F2, L2] = ReadIRMultiple(slowbot);
    [R3, F3, L3] = ReadIRMultiple(slowbot);

    R = (R1 + R2 + R3)/3;
    F = (F1 + F2 + F3)/3;
    L = (L1 + L2 + L3)/3;
    
    while ( (R < 0.31) && ( F > .4 ) )
        % Hoping to take an average because of pesky noise
        D = 0.5
        TravelDist(slowbot,0.2,D);
        % travel until the obstacle appears to be in front of robot
        [R1, F1, ~] = ReadIRMultiple(slowbot);
        [R2, F2, ~] = ReadIRMultiple(slowbot);
        [R3, F3, ~] = ReadIRMultiple(slowbot);

        R = (R1 + R2 + R3)/3;
        F = (F1 + F2 + F3)/3;
    end
    while ( (L < 0.31) && ( F > .4 ) )
        % Hoping to take an average because of pesky noise
        D = 0.5
        TravelDist(slowbot,0.2,D);
        % travel until the obstacle appears to be in front of robot
        [~, F1, L1] = ReadIRMultiple(slowbot);
        [~, F2, L2] = ReadIRMultiple(slowbot);
        [~, F3, L3] = ReadIRMultiple(slowbot);
        F = (F1 + F2 + F3)/3;
        L = (L1 + L2 + L3)/3;
    end

[x, y, theta_c] = OverheadLocalization(slowbot);
[~,theta] = dminT_theta(x,y, T1(1), T1(2));
end

% Robot shoudl align itself with the target before entering the other state (straight shot)
if((theta_c > pi) && (theta_c > theta))
TurnAngle(slowbot,0.2,-(theta_c + theta));
elseif((theta_c < theta) && (theta_c > 0))
TurnAngle(slowbot,0.2,theta);
elseif((theta_c < (2*pi)) && (theta_c > (1.33*pi)))
TurnAngle(slowbot,0.2,(2*pi) -theta_c + theta);
else
TurnAngle(slowbot,0.2,-(theta_c + theta));    
end
    
end