syms q1 q2 q1_dot q2_dot l1 l2 m2 m6 real

q = [q1; q2];
q_dot = [q1_dot; q2_dot];

% --- Call your existing symbolic helpers ---
M_sym = get_M(q1, q2, l1, l2, m2, m6);         % symbolic mass matrix
C_sym = get_C(m6, q1, q2, q1_dot);            % symbolic Coriolis matrix

% --- Construct Jm symbolically ---
Jm_sym = sym(zeros(8, 2));
Jm_sym(1,1) = 1/2;
Jm_sym(2,1) = (l1 * (q1 * cos(q1/2) - sin(q1/2))) / (2*q1^2);
Jm_sym(3,1) = Jm_sym(2,1);
Jm_sym(4,1) = 1/2;
Jm_sym(5,2) = 1/2;
Jm_sym(6,2) = (l2 * (q2 * cos(q2/2) - sin(q2/2))) / (2*q2^2);
Jm_sym(7,2) = Jm_sym(6,2);
Jm_sym(8,2) = 1/2;

M_comp = simplify(Jm_sym' * M_sym * Jm_sym);
C_comp = simplify(Jm_sym' * C_sym * Jm_sym);

% --- Export numeric functions ---
matlabFunction(M_comp, 'File', 'get_M_comp', 'Vars', {q1, q2, l1, l2, m2, m6});
matlabFunction(C_comp, 'File', 'get_C_comp', 'Vars', {q1, q2, q1_dot, q2_dot, l1, l2, m6});
