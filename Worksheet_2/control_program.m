function [ang_vel, fwd_vel] = control_program(phi_1, phi_2)

L = 3;
r = 2;
ang_vel = (r*phi_1 - r* phi_2)/(2*L);
fwd_vel = (r*phi_1 + r* phi_2)/2;
fwd_vel = fwd_vel/10;

end
