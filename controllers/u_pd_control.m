function u = u_pd_control(x, c, t)
% Simple PID controller for a 2-DOF soft manipulator
%
% Inputs:
%   x = [q1; q2; dq1; dq2]
%   c.Kp, c.Ki, c.Kd  (2x1 or scalar gains)
%   c.q_des           (2x1 desired joint position)
%   t = current time (unused, but kept for interface consistency)

q  = x(1:2);
dq = x(3:4);

e = c.p_des - q;

de = -dq;

% u = diag([c.k1, c.k2])*c.p_des + c.Kp .* e + c.Kd .* de;
u = c.Kp .* e + c.Kd .* de;

end

