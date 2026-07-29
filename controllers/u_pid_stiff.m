function w = u_pid_stiff(x, c, t)
% Simple PID controller for a 2-DOF soft manipulator
%
% Inputs:
%   x = [q1; q2; dq1; dq2]
%   c.Kp, c.Ki, c.Kd  (2x1 or scalar gains)
%   c.q_des           (2x1 desired joint position)
%   c.dt              (time step)
%   t = current time (unused, but kept for interface consistency)

% This controller should be used for one limb at a time, ie. only actuate
% one limb

base_offset = -100; % starting offset for internal pressure
% Negative values for base offsets -10,-20 ..
% Start from 0 to get initial q values
curr_lim  = 2; % which limb are we talking about

if curr_lim == 1

    u = [-60+base_offset;0]; % -60 for left side
    v = [base_offset;0];

else
    
    u = [0;-60+base_offset]; %-60 for left side
    v = [0;base_offset];

end

q  = x(1:2);

err = c.p_des(curr_lim) - q(curr_lim);

% v(1) = Kp .* err + Ki .* e_int

persistent previous_input
if isempty(previous_input)
    previous_input = v(1);
end

if err > 0.01
    v(curr_lim) = previous_input-0.1;
elseif err < -0.01
    v(curr_lim) = previous_input+0.1;
else
    v(curr_lim) = previous_input;
end 

previous_input = v(curr_lim);
%v = [0;0]; % Chris addition to inflate left side.
%w = [u;abs(v)]; % Chris removed to inflate left side.
w = [u;v];

end