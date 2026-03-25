function u = u_pd_control_trajectory(x, c, t)
    
    q  = x(1:2);
    dq = x(3:4);

    persistent u_prev
    if isempty(u_prev)
        u_prev = zeros(2,1);   
    end

    % Desired trajectory at time t
    [p_des, dp_des, ~] = desired_traj_sine(t);

    % Tracking errors
    e  = p_des - q;
    de = dp_des - dq;

    % feedforward term
    % u_ff = diag([c.k1, c.k2]) * p_des;

    % PD feedback
    u = c.Kp .* e + c.Kd .* de;

    % numerical stability check
    if isfield(c,'u_limit') && ~isempty(c.u_limit)
        if any(abs(u) > c.u_limit) || any(~isfinite(u))
            u = u_prev;
        else
            u_prev = u; 
        end
    end
end

function [qd, dqd, ddqd] = desired_traj_sine(t)
    persistent Fq1 Fq2 Fdq1 Fdq2 Fddq1 Fddq2 tmin tmax
    if isempty(Fq1)
        data = readtable('ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_CalibrationRunV1.csv', ...
        'HeaderLines', 2, 'VariableNamingRule', 'preserve');
        data(1,:) = [];
           
        time_data = data.("Test time");
        time_data = time_data(:);
        time_data = time_data - time_data(1);

        q1 = data.smoothed_q_0;
        q2 = data.smoothed_q_1;

        q1_dot = data.smoothed_dq_0;
        q2_dot = data.smoothed_dq_1;

        q1_ddot = data.smoothed_ddq_0;
        q2_ddot = data.smoothed_ddq_1;

        Fq1   = griddedInterpolant(time_data, q1, 'linear', 'nearest');
        Fq2   = griddedInterpolant(time_data, q2, 'linear', 'nearest');
        Fdq1  = griddedInterpolant(time_data, q1_dot,  'linear', 'nearest');
        Fdq2  = griddedInterpolant(time_data, q2_dot,  'linear', 'nearest');
        Fddq1 = griddedInterpolant(time_data, q1_ddot, 'linear', 'nearest');
        Fddq2 = griddedInterpolant(time_data, q2_ddot, 'linear', 'nearest');

        tmin = time_data(1);
        tmax = time_data(end);
    end

    tq = min(max(t, tmin), tmax);

    qd   = [Fq1(tq);   Fq2(tq)];
    dqd  = [Fdq1(tq);  Fdq2(tq)];
    ddqd = [Fddq1(tq); Fddq2(tq)];
end
