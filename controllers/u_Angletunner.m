function u = u_Angletunner(x,c,t)
    % Computed torque controller for 2-DOF planar arm/soft-joint model
    % State x = [q1;q2;dq1;dq2]
    % Uses desired trajectory qd(t), dqd(t), ddqd(t)


    persistent u_prev
    if isempty(u_prev)
        u_prev = zeros(2,1);   
    end

    q  = x(1:2);
    dq = x(3:4);

    qd = [1;0];
    dqd = [0;0];
    ddqd = [0;0];

    e  = qd - q;
    de = dqd - dq;

    % Model terms
    [M, C] = M_C_Computation(x, c);

    K = diag([c.k1, c.k2]);
    D = c.damping * eye(2);

    uff = M*ddqd + C*dqd + K*qd + D*dqd;

    if abs(e) < 0.09

        ufb = c.Kp_ct.*e + c.Kd_ct.*de;
        u = uff + ufb;

    else 

        u = uff ;        

    end

    % y = ddqd - c.Kp_ct.*e - c.Kd_ct.*de;  

    % numerical stability check
    if isfield(c,'u_limit') && ~isempty(c.u_limit)
        if any(abs(u) > c.u_limit) || any(~isfinite(u))
            u = u_prev;
        else
            u_prev = u; 
        end
    end
end
