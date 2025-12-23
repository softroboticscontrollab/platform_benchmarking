function u = u_pid_control(q, dq, q_des, dt, c)
% u_pid_control  Joint-space PID (2-DOF) with angle wrapping & anti-windup.
%   q     : [2x1] current joint angles (rad)
%   dq    : [2x1] current joint velocities (rad/s)
%   q_des : [2x1] desired joint angles (rad)
%   dt    : controller sample time (s)
%   c     : struct with optional fields:
%           Kp, Ki, Kd   [2x2 diagonal or 2x2 matrices]
%           I_max        [2x1] integral clamp (rad*s)
%           u_min,u_max  [2x1] command limits (generic units)
%
% Notes:
% - Derivative is on measurement (uses dq).
% - If u_min/u_max are not provided, defaults to ±1.
% - If your "command" is unipolar (e.g., 0..Pmax), set c.u_min=[0;0].

    % ---------- defaults ----------
    if ~isfield(c,'Kp'),    c.Kp   = diag([50, 50]);   end
    if ~isfield(c,'Ki'),    c.Ki   = diag([0.5, 0.5]); end
    if ~isfield(c,'Kd'),    c.Kd   = diag([1.0, 1.0]); end
    if ~isfield(c,'I_max'), c.I_max = [10; 10];        end

    if ~isfield(c,'u_min') || ~isfield(c,'u_max')
        c.u_min = -1*ones(2,1);  % default symmetric limits
        c.u_max =  1*ones(2,1);
    end

    % ---------- shape ----------
    q     = q(:); 
    dq    = dq(:); 
    q_des = q_des(:);

    % ---------- position error with angle wrap (-pi,pi] ----------
    e = atan2( sin(q_des - q), cos(q_des - q) );

    % ---------- integral state ----------
    persistent e_int
    if isempty(e_int)
        e_int = zeros(2,1);
    end
    % integrate & clamp
    e_int = e_int + e * dt;
    e_int = max(min(e_int, c.I_max), -c.I_max);

    % ---------- unsaturated PID (D on measurement) ----------
    u_unsat = c.Kp*e + c.Ki*e_int - c.Kd*dq;

    % ---------- saturate ----------
    u = min(max(u_unsat, c.u_min), c.u_max);

    % ---------- anti-windup: stop integrating when pushing into saturation ----------
    sat = (u ~= u_unsat);
    if any(sat)
        % if integral term is pushing further into the limit, undo last step
        Ki_e = c.Ki*e;                % integral "push" direction
        same_dir = sign(u_unsat) == sign(Ki_e);
        stop = sat & same_dir;
        if any(stop)
            e_int(stop) = e_int(stop) - e(stop)*dt;
        end
        % recompute final command after adjusting the integrator
        u = min(max(c.Kp*e + c.Ki*e_int - c.Kd*dq, c.u_min), c.u_max);
    end
end



% function u_pid = u_pid_control(q, dq, p_des, dt, c)
%     % PID gains for Cartesian-space control (tune these)
%     Kp = [100; 100]; % Proportional gains for [x, y]
%     Ki = [2; 2];    % Integral gains
%     Kd = [50; 50];  % Derivative gains
% 
%     % Persistent variables for integral term
%     persistent int_e
%     if isempty(int_e)
%         int_e = [0; 0]; % Initialize integral error
%     end
% 
%     % Compute current end-effector position using forward kinematics
%     p = forward_kinematics(q, c); % p = [x; y]
% 
%     % Compute error in Cartesian space
%     e = p_des - p;           % Position error
%     int_e = int_e + e * dt;  % Integral error accumulation
%     de = -jacobian(q, c) * dq; % Compute Cartesian velocity error
% 
%     % Compute desired Cartesian forces (control input)
%     u_pid = Kp .* e + Ki .* int_e + Kd .* de;
% 
%     % % Convert Cartesian forces to joint torques using the Jacobian transpose
%     % J = jacobian(q, c); % Compute Jacobian matrix
%     % u_pid = J' * F_des; % Compute joint torques
% 
%     % Limit torques to prevent excessive actuation
%     u_max = [200; 100]; % Define reasonable max torque values
%     u_pid = max(min(u_pid, u_max), -u_max);
% 
% end
% 
% function p = forward_kinematics(q, c)
%     % Extract parameters
%     l1 = c.l1;
%     l2 = c.l2;
% 
%     % Compute end-effector position using forward kinematics
%     x = l1 * cos(q(1)) + l2 * cos(q(1) + q(2));
%     y = l1 * sin(q(1)) + l2 * sin(q(1) + q(2));
% 
%     p = [x; y]; % Return as a column vector
% end
% 
% function J = jacobian(q, c)
%     % Extract parameters
%     l1 = c.l1;
%     l2 = c.l2;
% 
%     % Compute Jacobian matrix
%     J = [-l1 * sin(q(1)) - l2 * sin(q(1) + q(2)), -l2 * sin(q(1) + q(2));
%           l1 * cos(q(1)) + l2 * cos(q(1) + q(2)),  l2 * cos(q(1) + q(2))];
% end
