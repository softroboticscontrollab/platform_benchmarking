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
        data = readtable('sinwave_traj.csv', ...
        'HeaderLines', 2, 'VariableNamingRule', 'preserve');

        time_data = data.("Test time");
        time_data = time_data(:);
        time_data = time_data - time_data(1);

        q1 = deg2rad(2*data.theta_0(:));
        q2 = deg2rad(2*data.theta_1(:));

        window_length = 241;
        poly_order    = 3;
        [q1_dot, q1_ddot] = sgolay_derivatives(q1, time_data, window_length, poly_order);
        [q2_dot, q2_ddot] = sgolay_derivatives(q2, time_data, window_length, poly_order);

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

function [d1, d2] = sgolay_derivatives(y, t, window_length, poly_order)
        half_window = (window_length - 1) / 2;
        n = length(y);
        d1 = zeros(size(y));
        d2 = zeros(size(y));

        for i = 1:n
            idx = max(1, i - half_window):min(n, i + half_window);
            ti = t(idx);
            yi = y(idx);

            t_center = mean(ti);
            t_norm = ti - t_center;
            t_scale = max(abs(t_norm));
            if t_scale == 0
                t_scale = 1; 
            end
            t_norm = t_norm / t_scale;

            p = polyfit(t_norm, yi, poly_order);

            dp = polyder(p);
            ddp = polyder(dp);

            d1(i) = polyval(dp, 0) / t_scale;
            d2(i) = polyval(ddp, 0) / (t_scale^2);
        end
end
