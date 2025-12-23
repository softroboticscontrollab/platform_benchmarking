function [M_sym, C_sym, G_sym] = precompute_symbolic_matrices()
    % Define symbolic variables
    syms q1 q2 l1 l2 m2 m6 q1_dot q2_dot g

    % Call the function to get the symbolic inertia matrix
    [M_sym, U] = SoftDynamics_Inertia_Derivation();

    % Compute Coriolis matrix C
    C_sym = sym(zeros(8));
    q_sym = [q1; q1; q1; q1; q2; q2; q2; q2];
    q_dot_sym = [q1_dot; q1_dot; q1_dot; q1_dot; q2_dot; q2_dot; q2_dot; q2_dot];
   
    for i = 1:8
        for j = 1:8
            for k = 1:2  % Only q1 and q2 are generalized coordinates
                C_sym(i, j) = C_sym(i, j) + (1/2) * (diff(M_sym(i,j), q_sym(k)) + ...
                                                     diff(M_sym(i,k), q_sym(j)) - ...
                                                     diff(M_sym(j,k), q_sym(i))) * q_dot_sym(k);
            end
        end
    end
    matlabFunction(C_sym, 'File', 'get_C')
   
    % Compute gradient of potential energy to get gravitational forces/torques
    G_sym = zeros(8, 1);
    for i = 1:8
        G_sym(i) = diff(U, q_sym(i));
    end

end

function [M_sym,U] = SoftDynamics_Inertia_Derivation()
    % Define symbolic variables for joint angles and link lengths
    syms q1 q2 real;
    syms l1 l2 m2 m6 g real;
    
    % q1 l d alpha
    DH_params = [ q1/2,  0,   0,  pi/2;
                  0, l1*(sin(q1/2))/q1, 0, 0;
                  0, l1*(sin(q1/2))/q1, 0, -pi/2;
                  q1/2, 0,    0,    0;
                  q2/2,  0,   0,  pi/2;
                  0, l2*(sin(q2/2))/q2, 0, 0;
                  0, l2*(sin(q2/2))/q2, 0, -pi/2;
                  q2/2, 0,    0,    0];
    
    % Number of joints
    n = 8;
    
    % Transformation from base to first link
    T = eye(4);
    
    % Initialize transformation matrices and Jacobians
    T_matrices = sym(zeros(4, 4, n));
    Jv = sym(zeros(3, n, n));
    Jw = sym(zeros(3, n, n));
    
    for i = 1:n
        % Extract DH parameters
        theta = DH_params(i, 1);
        d = DH_params(i, 2);
        a = DH_params(i, 3);
        alpha = DH_params(i, 4);

        % Compute transformation matrix
        Ti = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
              sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,           sin(alpha),             cos(alpha),            d;
              0,           0,                      0,                     1];
        
        % Multiply to get the transformation from base to the current link
        T = T * Ti;
        T_matrices(:, :, i) = T;
        
        % Extract rotation matrix and position vector
        R = T(1:3, 1:3);
        p = T(1:3, 4);
    
        % Compute Jacobians
        if i == 1
            Jv(:, 1, i) = cross([0; 0; 1], p);
            Jw(:, 1, i) = [0; 0; 1];
        else
            Jv(:, 1:i-1, i) = Jv(:, 1:i-1, i-1);
            Jw(:, 1:i-1, i) = Jw(:, 1:i-1, i-1);
            if DH_params(i, 2) ~= 0  % Prismatic joint
                Jv(:, i, i) = R(:, 3);
                Jw(:, i, i) = [0; 0; 0];
            else  % Revolute joint
                Jv(:, i, i) = cross(R(:, 3), p - T_matrices(1:3, 4, i-1));
                Jw(:, i, i) = R(:, 3);
            end
        end
    end
    
    % Define mass and inertia properties for each link
    I_xx1 = (1/3) * 0 * l1^2;
    I_zz1 = (1/3) * 0 * l1^2;
    
    I_xx2 = (1/3) * m2 * l1^2;
    I_zz2 = (1/3) * m2 * l1^2;

    I_xx3 = (1/3) * 0 * l1^2;
    I_zz3 = (1/3) * 0 * l1^2;

    I_xx4 = (1/3) * 0 * l1^2;
    I_zz4 = (1/3) * 0 * l1^2;

    I_xx5 = (1/3) * 0 * l2^2;
    I_zz5 = (1/3) * 0 * l2^2;

    I_xx6 = (1/3) * m6 * l2^2;
    I_zz6 = (1/3) * m6 * l2^2;

    I_xx7 = (1/3) * 0 * l2^2;
    I_zz7 = (1/3) * 0 * l2^2;

    I_xx8 = (1/3) * 0 * l2^2;
    I_zz8 = (1/3) * 0 * l2^2;

    m = [0, m2, 0, 0, 0, m6, 0, 0];
    I = cat(3, diag([I_xx1, 0, I_zz1]), diag([I_xx2, 0, I_zz2]), ...
               diag([I_xx3, 0, I_zz3]), diag([I_xx4, 0, I_zz4]), ...
               diag([I_xx5, 0, I_zz5]), diag([I_xx6, 0, I_zz6]), ...
               diag([I_xx7, 0, I_zz7]), diag([I_xx8, 0, I_zz8]));
    
    % Compute inertia matrix
    M = sym(zeros(n, n));
    for i = 1:n
        Jvi = Jv(:, :, i);
        Jwi = Jw(:, :, i);
        M = M + m(i) * (Jvi' * Jvi) + Jwi' * R * I(:, :, i) * R' * Jwi;
    end
    
    % Simplify the inertia matrix
    M_sym = simplify(M);
    matlabFunction(M_sym, 'File', 'get_M')

    g_0 = [0; 0; g];
    U = 0;
    for i = 1:n
        p_i = T_matrices(1:3, 4, i);
        U = U + m(i) * g_0.' * p_i;
    end

end
