function [a, b] = expansion_soft_generate_constraints_sym()
%generate_constraints_sym Use MATLAB's symbolic solver to calculate the row
%vector a and scalar b for a constraint of the form a u \leq b, where a is
%a row in the A matrix for a CBF constraint, Lie_g1(barrier) ...
%Lie_gm(barrier), and b is the scalar gamma/barrier - Lie_f(barrier)
%   input = None, To Do allow user to pass in safety boundary etc
%   output = symbolic row a and scalar b
%   state change = save the calculation to a function .m file for easier
%   use later

% First, let's get the dynamics in the form of
% \dot x = f(x) + g(x)u, where x \in \mathbb{R}^2n, u \in \mathbb{R}^m, and
% x = [q \\ \dot q] with n configuration variables. So, f(x) should be a
% 2n x 1 vector, and g(x) should be a 2n x m matrix.

clear all;
close all;
clc;

% The constraint for this CBF will have this tuning constant, gamma.
syms gam 'real';
% The CBF itself has two tuning constants, need to declare them here for
% calling the matlabFunction later.
syms aE bE 'real';
% For our 2-joint rigid robot arm, these are the constants:
syms m2 m6 k1 k2 damping g 'real';
% ...and these are the states:
syms q1 q2 dq1 dq2 'real';
syms l1 l2 'real';
% syms k_env F_max 'real'

% Like with generate_barrierB_sym (which needs to be run first BEFORE this
% function!), we should name the CBF constraint and declare its parameters.
% Keep this naming consistent across files.

% We will be generating different barriers, so let's name them.
barriername = "face1";
% barriername = "face2";
% barriername = "face3";

disp("Calculating (symbolically) the constraint associated with the control barrier function " + barriername);

% Combine these variables into the arguments for one of our dynamics
% functions.
x = [q1; q2; dq1; dq2];
% the constants:
c.m2 = m2;
c.m6 = m6;
c.damping = damping;
c.k1 = k1;
c.k2 = k2;
c.l1 = l1;
c.l2 = l2;
c.g = g;
% To disable the numerical checking in the dynamics (not relevant for our
% symbolic calcs!), set the numerical tolerance to NaN.
c.tol = NaN;

% Choose which barrier function to consider - presumably, you've calculated
% this already, and the barrier function symbolic expression is
% barrierB_sym
barrierB_sym_filename = "expansion_soft_barrierB_" + barriername + "_sym.mat";
load(barrierB_sym_filename, 'barrierB_sym');

[Minv, bolddotx] = dynamics_soft(x, c, zeros(2,1));

% To get f(x), we can call the dynamics with the x as symbolic and =0.
f_dyn = bolddotx
% Similarly, to get g(x), we should be able to call the dynamics as x=0 and
% u=identity.
% g_dyn = f_doublepend_actuated(zeros(size(x)), c, eye(2))

% This doesn't really work because of dimensions. But we know, from the
% derivation, that g(x) is really just...
g_dyn = [eye(2); Minv]

% Now we calculate the Lie derivatives.
% To reuse our results, let's do the partials of B first. Per the Lie
% derivative, this should be a row vector rather than a column vector.
partialB = sym(zeros(size(x')));

disp('Partial derivatives of B with respect to each state are:')
for i = 1:size(x,1)
    % Calculate \partial B/partial x_i
    pBpx_i = diff(barrierB_sym, x(i));
    % force MATLAB to simplify as best as possible - often times, this is
    % more concise.
    % pBxp_i = simplify(pBpx_i)
    pBxp_i = pBpx_i
    partialB(i) = pBxp_i;
end

% partialB = simplify(partialB)

% The row vector a is a = [Lie_g1(barrier) ... Lie_gm(barrier)], which is
% 1 x m
% Here, m is the number of inputs, which we can infer via number of columns
% of g.
m_inputs = size(g_dyn,2);
a = sym(zeros(1, m_inputs));

for i = 1:m_inputs
    % just a dot product
    a(i) = partialB*g_dyn(:,i);
end

% On the right-hand side, b = \gamma/barrierB - Lie_f(barrier)
b = gam/barrierB_sym - partialB*f_dyn;

disp('This row of Ax \leq b is:')
a
b

% disp('Attempting to simplify a and b to avoid numerical issues when used later...')
% a = simplify(a)
% b = simplify(b)

% Finally, save these to functions to be called.
% the symbolic variables themselves, for debugging:
constraints_func_filename = "expansion_soft_constraints_" + barriername;
constraints_sym_filename = "expansion_soft_constraints_" + barriername + "_sym.mat";
save(constraints_sym_filename, 'a', 'b');

% swap out the q, dq for an indexed vector x. easier to plug in arguments
% when calling later.
x = sym("x", [4, 1]); % yes, this redefines the x above, TODO fix sloppy

% this is how you swap variable names in a matlab symbolic expression
a = subs(a, q1, x(1));
a = subs(a, q2, x(2));
a = subs(a, dq1, x(3));
a = subs(a, dq2, x(4));
b = subs(b, q1, x(1));  
b = subs(b, q2, x(2));
b = subs(b, dq1, x(3));
b = subs(b, dq2, x(4));

% We need to save two functions: the a vs the b need to be called
% separately.
constraints_func_filename_a = strcat(constraints_func_filename, '_a');
constraints_func_filename_b = strcat(constraints_func_filename, '_b');

% this is how to save the result for use in the future 
matlabFunction(a, "File", constraints_func_filename_a, 'vars', {x, l1, l2, m2, m6, k1, k2, damping, aE, bE, gam, g});
matlabFunction(b, "File", constraints_func_filename_b, 'vars', {x, l1, l2, m2, m6, k1, k2, damping, aE, bE, gam, g});

disp('Successfully saved these functions:');
constraints_func_filename_a
constraints_func_filename_b

end 