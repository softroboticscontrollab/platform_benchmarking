function u = u_pid_control(x, c)
% Simple PID controller for a 2-DOF soft manipulator
%
% Inputs:
%   x = [q1; q2; dq1; dq2]
%   c.Kp, c.Ki, c.Kd  (2x1 or scalar gains)
%   c.q_des           (2x1 desired joint position)
%   c.dt              (time step)
%   t = current time (unused, but kept for interface consistency)

q  = x(1:2);
dq = x(3:4);

persistent e_int
if isempty(e_int)
    e_int = zeros(2,1);
end


e = c.p_des - q;

e_int = e_int + e * c.dt;

de = -dq;

u = c.Kp .* e + c.Ki .* e_int + c.Kd .* de;

end


