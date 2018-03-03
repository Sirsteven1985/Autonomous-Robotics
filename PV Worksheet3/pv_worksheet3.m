function finalRad = myFirstControlProgram(robotObj)
    
    T1 = [1,2];
    T2 = [1,-2];
    xCur=0; yCur=0;
    dR=0; dF=0; dL=0;
    dR_CCW=0; dF_CCW=0; dL_CCW=0;
    dR_CW=0; dF_CW=0; dL_CW=0;
    thetaCur=0; thetaNew=0; thetaBool=0;
    straddle=0;
    straddleR=0; straddleF=0; straddleL=0;
    
    % Set initial robot location
    setMapStart(robotObj,[-2 -2 0]);
    minApproachDist = 0.01; barrierDist=0;

    [xCur, yCur, thetaCur] = OverheadLocalization(robotObj);

    % Mode 1 Proceed to Goal 1
    while((xCur ~= T1(1)) && (yCur ~= T1(2)))

        % Determine current location
        [xCur, yCur, thetaCur] = OverheadLocalization(robotObj);
        % Determine destination angle
        thetaNew = atan((T1(2)-yCur)/(T1(1)-xCur));

        % If current angle to target is not true to destination angle
        if(thetaCur ~= thetaNew)
            thetaCur = thetaNew;
            % Rotate robot to destination angle
            TurnAngle(robotObj,0.2,thetaCur);
        end

        % Cut him loose
        SetFwdVelAngVel(robotObj,0.25,0);

        % Check for obstacle in forward path
        approachDist = ReadIR(robotObj,2);

        % Drive forward until obstacle is seen at minimum approach
        % distance
        if(approachDist > minApproachDist)
            SetFwdVelAngVel(robotObj,0.25,0);
            straddle = 1;
        else                
            % if approach distance is within minimum approach distance
            % stop robot and read RFL IR sensors
            SetFwdVelAngVel(robotObj,0,0)
            [dR, dF, dL] = ReadIRMultiple(robotObj);
            % rotate +45 degrees and read again
            TurnAngle(robotObj,0.2,thetaCur + pi/4);
            [dR_CCW, dF_CCW, dL_CCW] = ReadIRMultiple(robotObj);
            % rotate -90 degrees and read again
            TurnAngle(robotObj,0.2,thetaCur - pi/2);
            [dR_CW, dF_CW, dL_CW] = ReadIRMultiple(robotObj);
            % return to destination angle
            TurnAngle(robotObj,0.2,thetaCur + pi/4);
            % Also, a variable will be needed to allow robot to
            % jump into Mode 2 to straddle the barrier wall
            straddle = 1;
        end

        % Determine which angle represents the most optimum local 
        % minima to traverse to destination

        % if the local minima to the left of the robot is closer
        % re-angle in this direction.
        if(dF_CCW <= minApproachDist && dF_CCW < dF_CW)
            thetaBool = 1;
            thetaMinNew = thetaCur + pi/4;
            TurnAngle(robotObj,0.2,thetaMinNew);                
            % Read the new distance for the front IR sensor
            barrierDist = ReadIR(robotObj,2);
        else
            % the angle to the right of the robot must be most optimum
            thetaBool = 0;
            thetaMinNew = thetaCur - pi/4;
            TurnAngle(robotObj,0.2,thetaMinNew);
            % Read the new distance for the front IR sensor
            barrierDist = ReadIR(robotObj,2);
        end

        % Proceed to local minima and stop at barrier wall  
        if(barrierDist >= (barrierDist - (0.9*barrierDist)))
            SetFwdVelAngVel(robotObj,0.25,0);
        else
            SetFwdVelAngVel(robotObj,0,0);
        end

        % Once at the barrier wall reposition 
        % Parallel to the surface
        if(thetaBool == 1)
            thetaMinNew = thetaMinNew + pi/4;
            TurnAngle(robotObj,0.2,thetaMinNew);
        else
            thetaMinNew = thetaMinNew - pi/4;
            TurnAngle(robotObj,0.2,thetaMinNew);
        end

        % Read IR sensors and then jump into while loop for Mode 2
        % to straddle barrier wall while traversing
        [straddleR, straddleF, straddleL] = ReadIRMultiple(robotObj);

        % Mode 2
        % Straddle and Traverse until all IR sensors are equivalent
        while(straddle == 1)
            % check if Left and Right IR sensors are equivalent
            while(straddleR ~= straddleL)
            % cut it loose
            SetFwdVelAngVel(robotObj,0.25,0);
            % check IR sensors
            [straddleR, straddleF, straddleL] = ReadIRMultiple(robotObj);

                if(straddleR == straddleL)
                    % break out of loop
                    straddle = 0;
                    % stop
                    SetFwdVelAngVel(robotObj,0,0);
                else
                    % Cut him loose
                    SetFwdVelAngVel(robotObj,0.25,0); 
                    [straddleR, straddleF, straddleL] = ReadIRMultiple(robotObj);
                end
                
            end
        end

    end % end Goal 1
    
    % Proceed to Goal 2
    [xCur, yCur, thetaCur] = OverheadLocalization(robotObj);

    % Mode 1 Proceed to Goal 1
    while((xCur ~= T2(1)) && (yCur ~= T2(2)))

        % Determine current location
        [xCur, yCur, thetaCur] = OverheadLocalization(robotObj);
        % Determine destination angle
        thetaNew = atan((T1(2)-yCur)/(T1(1)-xCur));

        % If current angle to target is not true to destination angle
        if(thetaCur ~= thetaNew)
            thetaCur = thetaNew;
            % Rotate robot to destination angle
            TurnAngle(robotObj,0.2,thetaCur);
        end

        % Cut him loose
        SetFwdVelAngVel(robotObj,0.25,0);

        % Check for obstacle in forward path
        approachDist = ReadIR(robotObj,2);

        % Drive forward until obstacle is seen at minimum approach
        % distance
        if(approachDist > minApproachDist)
            SetFwdVelAngVel(robotObj,0.25,0);
            straddle = 1;
        else                
            % if approach distance is within minimum approach distance
            % stop robot and read RFL IR sensors
            SetFwdVelAngVel(robotObj,0,0)
            [dR, dF, dL] = ReadIRMultiple(robotObj);
            % rotate +45 degrees and read again
            TurnAngle(robotObj,0.2,thetaCur + pi/4);
            [dR_CCW, dF_CCW, dL_CCW] = ReadIRMultiple(robotObj);
            % rotate -90 degrees and read again
            TurnAngle(robotObj,0.2,thetaCur - pi/2);
            [dR_CW, dF_CW, dL_CW] = ReadIRMultiple(robotObj);
            % return to destination angle
            TurnAngle(robotObj,0.2,thetaCur + pi/4);
            % Also, a variable will be needed to allow robot to
            % jump into Mode 2 to straddle the barrier wall
            straddle = 1;
        end

        % Determine which angle represents the most optimum local 
        % minima to traverse to destination

        % if the local minima to the left of the robot is closer
        % re-angle in this direction.
        if(dF_CCW <= minApproachDist && dF_CCW < dF_CW)
            thetaBool = 1;
            thetaMinNew = thetaCur + pi/4;
            TurnAngle(robotObj,0.2,thetaMinNew);                
            % Read the new distance for the front IR sensor
            barrierDist = ReadIR(robotObj,2);
        else
            % the angle to the right of the robot must be most optimum
            thetaBool = 0;
            thetaMinNew = thetaCur - pi/4;
            TurnAngle(robotObj,0.2,thetaMinNew);
            % Read the new distance for the front IR sensor
            barrierDist = ReadIR(robotObj,2);
        end

        % Proceed to local minima and stop at barrier wall  
        if(barrierDist >= (barrierDist - (0.9*barrierDist)))
            SetFwdVelAngVel(robotObj,0.25,0);
        else
            SetFwdVelAngVel(robotObj,0,0);
        end

        % Once at the barrier wall reposition 
        % Parallel to the surface
        if(thetaBool == 1)
            thetaMinNew = thetaMinNew + pi/4;
            TurnAngle(robotObj,0.2,thetaMinNew);
        else
            thetaMinNew = thetaMinNew - pi/4;
            TurnAngle(robotObj,0.2,thetaMinNew);
        end

        % Read IR sensors and then jump into while loop for Mode 2
        % to straddle barrier wall while traversing
        [straddleR, straddleF, straddleL] = ReadIRMultiple(robotObj);

        % Mode 2
        % Straddle and Traverse until all IR sensors are equivalent
        while(straddle == 1)
            % check if Left and Right IR sensors are equivalent
            while(straddleR ~= straddleL)
            % cut it loose
            SetFwdVelAngVel(robotObj,0.25,0);
            % check IR sensors
            [straddleR, straddleF, straddleL] = ReadIRMultiple(robotObj);

                if(straddleR == straddleL)
                    % break out of loop
                    straddle = 0;
                    % stop
                    SetFwdVelAngVel(robotObj,0,0);
                else
                    % Cut him loose
                    SetFwdVelAngVel(robotObj,0.25,0); 
                    [straddleR, straddleF, straddleL] = ReadIRMultiple(robotObj);
                end
                
            end
        end

    end % end Goal 2    
        
end
