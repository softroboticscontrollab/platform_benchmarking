function u = u_computed_torque_control(x, c, t)
% Computed torque controller for 2-DOF planar arm/soft-joint model
% State x = [q1;q2;dq1;dq2]
% Uses desired trajectory qd(t), dqd(t), ddqd(t)

    q  = x(1:2);
    dq = x(3:4);

    [qd, dqd, ddqd] = desired_traj_sine(t, c);

    e  = qd - q;
    de = dqd - dq;

    % Model terms
    [M, C] = M_C_Computation(x, c); 

    K = diag([c.k1, c.k2]);
    D = c.damping * eye(2);

    y = ddqd + c.Kp_ct.*e + c.Kd_ct.*de;          
    u = M*y + C*dq + K*q + D*dq;

end


function [qd, dqd, ddqd] = desired_traj_sine(t, c)
% Sinusoidal desired joint trajectory around c.p_des.

    q0 = x(1:2,1);

    % amplitude (rad) and frequency (Hz)
    A = deg2rad([60; 60]);      % 10 deg amplitude on each joint
    f = [0.2; 0.2];            % Hz
    w = 2*pi*f;

    qd   = q0 + A .* sin(w*t);
    dqd  = A .* (w) .* cos(w*t);
    ddqd = -A .* (w.^2) .* sin(w*t);
end
