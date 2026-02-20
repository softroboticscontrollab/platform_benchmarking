function u_star = soft_u_cbf_polygonal_safeset(u_nom, x, c)
%u_cbf_xxxx A controller for the 2-link rigid robot arm, implementing the
%quadratic program that imposes a barrier function constraint on a nominal
%control signal.
%
%   Inputs:
%       u_nom == nominal (presumably unsafe) control signal in R2.
%       x == states at time t. This is [q1; q2; dq1; dq2]
%       c == constants, see below. Everything needed for the constraint
%       functions to be evaluated, which means the dynamics constants, the
%       tuning parameter(s) for the CBF, and the tuning parameter(s) for
%       the constraint.

% The optimization problem is:
% u_star = argmin_u   u'*u - 2u_nom'*u
%       s.t. a(x) u \leq b(x)

% We have four constraints:  q1<q1max, q1>q1min, q2<q2max, q2>q2min 
% barrierB_sym = soft_barrierB_face1(x,c.l1,c.l2,c.aE,c.bE,c.k_env,c.F_max)
% disp(barrierB_sym);

%%%% DEBUGGING
% Test the magnitude of the safe set constraint, one row of Hr \leq h

%%%% DEBUGGING
% Test the barrier function to see if that's where we get imag numbers
barrierB_debug = soft_barrierB_face1(x, c.l1, c.l2, c.aE, c.bE, c.k_env, c.F_max);

% disp("Barrier B(x) = " + string(barrierB_debug) + ", [q1, q2, dq1, dq2] = " + string(x(1)) + " " + string(x(2)) + " " + string(x(3)) + " " + string(x(4)));

% safeset face1
a1 = soft_constraints_face1_a(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
     c.aE, c.bE, c.gam, c.g);
b1 = soft_constraints_face1_b(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
     c.aE, c.bE, c.gam, c.g);

% % safeset face2
% a2 = soft_constraints_face2_a(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
%      c.aE, c.bE, c.gam, c.g);
% b2 = soft_constraints_face2_b(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
%      c.aE, c.bE, c.gam, c.g);

% % safeset face3
a3 = soft_constraints_face3_a(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
     c.aE, c.bE, c.gam, c.g);
b3 = soft_constraints_face3_b(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
     c.aE, c.bE, c.gam, c.g);

% % safeset face4
% a4 = soft_constraints_face4_a(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
%      c.aE, c.bE, c.gam, c.g);
% b4 = soft_constraints_face4_b(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
%      c.aE, c.bE, c.gam, c.g);

%EXPANSION_SOFT_CONSTRAINTS_FACE1_A
% expanded safeset face1
a1_prime = expansion_soft_constraints_face1_a(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
     c.aE, c.bE, c.gam, c.g);
b1_prime = expansion_soft_constraints_face1_b(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
     c.aE, c.bE, c.gam, c.g);

% expanded safeset face2
% a2_prime = expansion_soft_constraints_face2_a(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
%      c.aE, c.bE, c.gam, c.g);
% b2_prime = expansion_soft_constraints_face2_b(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
%      c.aE, c.bE, c.gam, c.g);

% expanded safeset face3
a3_prime = expansion_soft_constraints_face3_a(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
     c.aE, c.bE, c.gam, c.g);
b3_prime = expansion_soft_constraints_face3_b(x, c.l1, c.l2, c.m2, c.m6, c.k1, c.k2, c.damping, ...
     c.aE, c.bE, c.gam, c.g);

%control input constraints
a_input = [ eye(2);
           -eye(2)];

b_input = [ 80;
            80;
            80;
            80 ];

% A = [a1; a2; a3];
% b = [b1; b2; b3]; 

% A = [a1_prime];
% b = [b1_prime];

% A = [a1; a3];
% b = [b1; b3];

% A = [a1_prime];
% b = [b1_prime];

A = [a1_prime];
b = [b1_prime];

% A = [a3;a_input];
% b = [b3;b_input];

% A = [a1];
% b = [b1];

% A = [a3_prime];
% b = [b3_prime];

% A = [a1_prime];
% b = [b1_prime];

% A = [a3;a_input];
% b = [b3;b_input];

% A = [a1_prime;a_input];
% b = [b1_prime;b_input];

% A = [a1_prime; a3_prime];
% b = [b1_prime; b3_prime];

% A = [a1;a3;a_input];
% b = [b1;b3;b_input];

% A = [];
% b = [];

% if you get the sign wrong for your safe set h(x), you can get complex
% numbers here.
if ~isreal(A)
    disp('WARNING: row of A in CBF constraint is a complex number, you may have started your simulation in an unsafe set? A= '+ string(A));
end
if ~isreal(b)
    disp('WARNING: row of b in CBF constraint is a complex number, you may have started your simulation in an unsafe set? b= '+ string(b));
end

% ...check and confirm, there should not be variables here! all values
% filled in for A and b.

% Quadprog solves 0.5x'Hx + f'x
H = 2*eye(2);
% f = 2*u_nom;
f = -2*u_nom';
% ...we can remove the factor of 2 and get the same argmin.

% We could really just do:
%u_star = quadprog(H, f, A, b);

% ...but that produces a ton of output text, want the output to be silent
options = optimset('Display', 'off');
% need to pass in empty arguments for equality constraints and box
% constraints.
Aeq = [];
beq = [];
lb = [];
ub = [];
x0 = [];
% disp('Calling quadprog for state')
u_star = quadprog(H, f, A, b, Aeq, beq, lb, ub, x0, options);
end