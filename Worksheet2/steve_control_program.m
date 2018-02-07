function [ang_vel, fwd_vel] = control_program(phi_1, phi_2)

L = 3;	% known distance from P to wheel
r = 2;	% known radius of wheel
ang_vel = (r*phi_1 - r* phi_2)/(2*L);	% eqn to find angular vel
fwd_vel = (r*phi_1 + r* phi_2)/2;		% eqn to find forward
fwd_vel = fwd_vel/10;					% scaled down to show on map

end
