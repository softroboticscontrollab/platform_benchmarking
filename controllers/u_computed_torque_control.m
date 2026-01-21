function u = u_computed_torque_control(x, c, t)
% Computed torque controller for 2-DOF planar arm/soft-joint model
% State x = [q1;q2;dq1;dq2]
% Uses desired trajectory qd(t), dqd(t), ddqd(t)

    q  = x(1:2);
    dq = x(3:4);

    [qd, dqd, ddqd] = desired_traj_sine(t);

    e  = qd - q;
    de = dqd - dq;

    % Model terms
    [M, C] = M_C_Computation(x, c); 

    K = diag([c.k1, c.k2]);
    D = c.damping * eye(2);

    y = ddqd + c.Kp_ct.*e + c.Kd_ct.*de;    
    % y = ddqd + c.Kp_ct.*qd + c.Kd_ct.*dqd - c.Kp_ct.*q - c.Kd_ct.*dq;  
    % u = M*y + C*dq + K*q + D*dq;
    u = M*y + C*dq;

end


% function [qd, dqd, ddqd] = desired_traj_sine(x, c, t)
% % Sinusoidal desired joint trajectory around c.p_des.
%     f = [0.2; 0.2];           
%     w = 2*pi*f;
% 
%     q_bias = deg2rad([5; 5]);         
%     A = deg2rad([50; 50]); 
% 
%     A = min(A, 0.8*abs(q_bias));
% 
%     qd = q_bias + A .* sin(w*t);
%     eps_q = 1e-6; 
%     if any(abs(qd) < eps_q)
%         disp("qd is near zero (possible singular region)");
%         qd = 1e-3;
%     end
%     dqd = A .* w .* cos(w*t);
%     ddqd = -A .* (w.^2) .* sin(w*t);
% 
% end

% function [qd, dqd, ddqd] = desired_traj_sine(t)
%     time_data = data.("Test time");
%     time_data = time_data(:);
% 
%     q1 = deg2rad(2*data.theta_0(:));
%     q2 = deg2rad(2*data.theta_1(:));
% 
%     window_length = 241;  
%     poly_order = 3;
% 
%     [q1_dot, q1_ddot] = sgolay_derivatives(q1, time_data, window_length, poly_order);
%     [q2_dot, q2_ddot] = sgolay_derivatives(q2, time_data, window_length, poly_order);
% 
%     % Clamp t into available time range
%     t = min(max(t, time_data(1)), time_data(end));
% 
%     % Interpolate to get values at time t (scalar)
%     qd   = [interp1(time_data, q1,      t, 'linear');
%             interp1(time_data, q2,      t, 'linear')];
% 
%     dqd  = [interp1(time_data, q1_dot,  t, 'linear');
%             interp1(time_data, q2_dot,  t, 'linear')];
% 
%     ddqd = [interp1(time_data, q1_ddot, t, 'linear');
%             interp1(time_data, q2_ddot, t, 'linear')];
% end

function [qd, dqd, ddqd] = desired_traj_sine(t)
    persistent Fq1 Fq2 Fdq1 Fdq2 Fddq1 Fddq2 tmin tmax

    if isempty(Fq1)
        data = readtable('ezloophw_closedloop_ros2_reserv_pneumatics_2026-1-20_153443.csv', ...
        'HeaderLines', 2, 'VariableNamingRule', 'preserve');

        time_data = data.("Test time");
        time_data = time_data(:);
        time_data = time_data - time_data(1);  % align to start at 0

        q1 = deg2rad(2*data.theta_0(:));
        q2 = deg2rad(2*data.theta_1(:));

        window_length = 241;
        poly_order    = 3;
        [q1_dot, q1_ddot] = sgolay_derivatives(q1, time_data, window_length, poly_order);
        [q2_dot, q2_ddot] = sgolay_derivatives(q2, time_data, window_length, poly_order);

        Fq1   = griddedInterpolant(time_data, q1,      'linear', 'nearest');
        Fq2   = griddedInterpolant(time_data, q2,      'linear', 'nearest');
        Fdq1  = griddedInterpolant(time_data, q1_dot,  'linear', 'nearest');
        Fdq2  = griddedInterpolant(time_data, q2_dot,  'linear', 'nearest');
        Fddq1 = griddedInterpolant(time_data, q1_ddot, 'linear', 'nearest');
        Fddq2 = griddedInterpolant(time_data, q2_ddot, 'linear', 'nearest');

        tmin = time_data(1);
        tmax = time_data(end);
    end

    tq = min(max(t, tmin), tmax);

    qd   = [Fq1(tq);   Fq2(tq)];
    if any(abs(qd) < 1e-6)
        disp("qd is near zero (possible singular region)");
    end
    dqd  = [Fdq1(tq);  Fdq2(tq)];
    ddqd = [Fddq1(tq); Fddq2(tq)];
end

function [d1, d2] = sgolay_derivatives(y, t, window_length, poly_order)
        % Computes smoothed first and second derivatives using Savitzky-Golay method
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