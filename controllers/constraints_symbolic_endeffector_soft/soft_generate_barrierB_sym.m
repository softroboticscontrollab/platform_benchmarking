function barrierB_sym = soft_generate_barrierB_sym()
%generate_barrierB_sym Use MATLAB's symbolic solver to calculate a control
%barrier function, given a safety constraint.
%   input = None, To Do allow user to pass in safety boundary etc
%   output = symbolic B
%   state change = save the calculation to a function .m file for easier
%   use later

clear all;
close all;
clc;

% The states:
syms q1 q2 dq1 dq2 l1 l2 'real'
% The barrier function will also have some tuning constants.
syms aE bE 'real'
syms k_env F_max 'real'

% V = [0, 0.8; 
%     1, -1;
%      1.8, 1.2];

% V = [-0.02, 0.16; 
%       0.15, -0.175;
%      0.45, 0.19];

% V = [-0.225, 0.2125;
%     -0.075, -0.2;
%     0.2, -0.2;
%     0.35, 0.2125];

% V = [0.35, -0.005;
%     0, 0.122389;
%     -0.2, -0.2;
%     0.2, -0.2];

% V = [ 0.35, -0.005;
%     0, 0.122389;
%     0,  -0.16;
%     0.35, -0.1];

% V = [ 0.35, -0.005;
%     0, 0.122389;
%     0,  -0.302072;
%     0.35, -0.1];

V = [ 0.35, 0.0322;
    0, 0.159627;
    0,  -0.1963;
    0.35, 0.00577];

x_r = l2 * ((cos(q1) * sin(q2) + sin(q1) * cos(q2) - sin(q1)) / q2) + l1 * (sin(q1) / q1);
y_r = l2 * ((sin(q1) * sin(q2) - cos(q1) * cos(q2) + cos(q1)) / q2) + l1 * ((1 - cos(q1)) / q1);
r = [x_r; y_r];

[H_prime, h_prime] = computeSafetyConstraints(V);

% We will be generating different barriers, so let's name them.
barriername = "face1";
% barriername = "face2";
% barriername = "face3";
% barriername = "face4";

h = h_prime(1,:) - H_prime(1,:) * r ;
% h = h_prime(2,:) - H_prime(2,:) * r ;
% h = h_prime(3,:) - H_prime(3,:) * r ;
% h = h_prime(4,:) - H_prime(4,:) * r ;

% h = h_prime(1,:) + H_prime(1,:) * r ;
% h = h_prime(2,:) + H_prime(2,:) * r ;
% h = h_prime(3,:) + H_prime(3,:) * r ;
% h = h_prime(4,:) + H_prime(4,:) * r ;

disp("Calculating symbolic control barrier function B(x) for constraint h(x) that is " + barriername);

% We need \dot h for the barrier function
% thanks to https://www.mathworks.com/matlabcentral/fileexchange/7174-fulldiff-m
% The second argument here is the independent variables that we're
% fulldiff-ing, to distinguish from the constants. Example, if h has both
% q2 and q2max syms, we only want to differentiate q2.
% hdot = fulldiff(h, {'q1', 'q2'});

J = jacobian([x_r; y_r], [q1, q2]);      
a_vec = H_prime(1, :);                   
grad_h = -a_vec * J;                      
hdot = grad_h * [dq1; dq2];  

% J = jacobian([x_r; y_r], [q1, q2]);      
% a_vec = H_prime(1, :);                   
% grad_h = a_vec * J;                      
% hdot = grad_h * [dq1; dq2];  

% our proposed barrier function is...
barrierB_sym = barrierB_hsu2015(h, hdot, aE, bE);

disp("The barrier function, symbolically, is B(x) = ")
barrierB_sym

% We will need to call this function directly, as well as calculate its
% derivatives, so save it as a function as well as a .mat with the sym vars
% in it.

barrierB_func_filename = "soft_barrierB_" + barriername;
barrierB_sym_filename = barrierB_func_filename + "_sym.mat";
save(barrierB_sym_filename, 'barrierB_sym');

% One way to force a certain number of arguments is to use a symbolic
% vector and then MATLAB will index into it. The definition of the barrier
% function takes the full states, x, which for our example is q1, q2, dq1,
% dq2.

% Let's use our full set of the non-constrained q^i's. We know there are
% the same number of these as our M matrix, so:

x = sym("x", [4, 1]);

%Introduce forward kinematics equation here to convert from the states q to
%corresponding position x and y

% this is how you swap variable names in a matlab symbolic expression
barrierB_sym = subs(barrierB_sym, q1, x(1));
barrierB_sym = subs(barrierB_sym, q2, x(2)); % not present.
barrierB_sym = subs(barrierB_sym, dq1, x(3));
barrierB_sym = subs(barrierB_sym, dq2, x(4));

disp('Swapped with syms for the state vector q, is B(q) = ')
barrierB_sym

% this is how to save the result for use in the future rather than needing
% to perform the inverse calculation every time you run your script.
matlabFunction(barrierB_sym, "File", barrierB_func_filename, 'vars', {x, l1, l2 aE, bE, k_env, F_max});
disp("Saved file to " + barrierB_func_filename);

end