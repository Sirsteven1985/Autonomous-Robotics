function [rowboat] = str8_shot(rowboat, T1)
% this function runs the state in which the robot aligns itself with the
% target and drive straight toward it.
Tx = T1(1);
Ty = T1(2);
F = ReadIR(rowboat,2);

while (F >= 0.25)
    F = ReadIR(rowboat,2);
    TravelDist(rowboat,0.2,0.1)
    [x, y, ~] = OverheadLocalization(rowboat);   % finds the robots current pose

    [dmin,~] = dminT_theta(x,y, Tx, Ty);

    if(dmin < 0.1)  % goal is reached, sort of
        break;
    end
end


   
    
end