function [dmint,theta] = dminT_theta(posx,posy, Tx, Ty)
    % This function returns the minimum distance to target and theta needed
    % to reach target T from its current position
    
    x = (Tx - posx);
    y = (Ty - posy);
    dmint = sqrt( (x^2) + (y^2) );
    theta = atan(y/x);
    
end