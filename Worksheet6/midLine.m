function midLine = midLine(stuff)

% Read in values
%load featureTwo;
dataObs = stuff;
x_Obs = dataObs(:,1);
y_Obs = dataObs(:,2);
figure
plot(x_Obs,y_Obs,'*')
hold on

nSize = length(dataObs);

% Preliminary analysis of the data set indicates robot detects two clusters
% of points organized in a vertical direction. Robot maybe detecting
% parallel objects to the left and right of it. In which case the regression 
% formula would prove useless as it would not account for an infinite slope.
Xi = ones(nSize,2);
Xi(1:nSize,2) = dataObs(1:nSize,1);
Yi = dataObs(1:nSize,2);
Xi_transpose = Xi.';
wi = (Xi_transpose*Xi)^(-1)*Xi_transpose*Yi;
Si = Xi*wi-Yi;
Si_mag_L2_norm = (abs(Si)).^2;

% Repeating what was done in worksheet 5, manually split data into subsets
% 1st cluster seen at n=1 to n=13
Xi1 = ones(13,2);
Xi1(1:13,2) = dataObs(1:13,1);
Yi1 = dataObs(1:13,2);
Xi1_transpose = Xi1.';
wi1 = (Xi1_transpose*Xi1)^(-1)*Xi1_transpose*Yi1;
Si1 = Xi1*wi1-Yi1;
Si1_mag_L2_norm = (abs(Si1)).^2;
% Inspection of L2-norm indicates a best fit line location at n=7 of
% Si1_mag_L2_norm resultant matrix.Our vertical line should run through
% this point.
% For the first cluster of points our vertical line should run through
% point (x_Obs(7),y_Obs(7)).

% Remaining clusters seen at n=14 to end of data set
Xi2 = ones(10,2);
Xi2(1:10,2) = dataObs(14:23,1);
Yi2 = dataObs(14:23,2);
Xi2_transpose = Xi2.';
wi2 = (Xi2_transpose*Xi2)^(-1)*Xi2_transpose*Yi2;
Si2 = Xi2*wi2-Yi2;
Si2_mag_L2_norm = (abs(Si2)).^2;
% Inspection of L2-norm indicates a best fit line location at n=2 of
% Si2_mag_L2_norm resultant matrix.Our vertical line should run through
% this point.
% For the second cluster of points our vertical line should run through
% point (x_Obs(23),y_Obs(23)).

% Create matrix for line running through point (x_Obs(7),y_Obs(7)) and with
% vertical limits.
y1_max = y_Obs(8);
y1_min = y_Obs(2);
Line_x1 = ([x_Obs(7) x_Obs(7) x_Obs(7)]);
Line_y1 = ([y1_max y_Obs(7) y1_min]);

% Create matrix for line running through point (x_Obs(23),y_Obs(23)) and with
% vertical limits.
y2_max = y_Obs(22);
y2_min = y_Obs(17);
Line_x2 = ([x_Obs(23) x_Obs(23) x_Obs(23)]);
Line_y2 = ([y2_max y_Obs(23) y2_min]);

plot(Line_x1,Line_y1)
plot(Line_x2,Line_y2)
hold off 

% Polar coordinates of line segment midpoints
midLine1_mag = sqrt(x_Obs(7)^2 + ((y1_max-y1_min)/2)^2);
midLine1_angle = (atan((abs(y1_max-y1_min)/2)/abs(x_Obs(7)))) + pi;

midLine2_mag = sqrt(x_Obs(23)^2 + ((y2_max-y2_min)/2)^2);
midLine2_angle = (atan((abs(y2_max-y2_min)/2)/abs(x_Obs(23)))) + pi;

% matrix of values to be returned
midLine = ([midLine1_mag midLine1_angle; midLine2_mag midLine2_angle]);

end
