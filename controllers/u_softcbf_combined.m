function u = u_softcbf_combined(x, constants, t)
%u_softcbf_combined The control calculation that combines the safe
%supervisory based on CBFs with a nominal controller for a two-segment soft
%manipulator
%
%   Inputs:
%       x == state at time t, assumed to be in R4 for now [q1; q2; dq1;
%       dq2]
%       constants == see below, various sine frequency and amplitude stuff
%       t == timestep. This is an open-loop controller, so we need to know
%       the time of the simulation.

nom_ctrlr = @u_computed_torque_control;
% nom_ctrlr = @u_babblesine;

supervisor = @soft_u_cbf_polygonal_safeset;

% two stages. First, the nominal controller.
% Assume the nominal controller takes arguments this way.
u_nom = nom_ctrlr(x, constants, t);
% u_nom = nom_ctrlr(t, constants);
% u_nom = 50 + nom_ctrlr(t, constants);
%  u_nom = abs(nom_ctrlr(t, constants));

% u_nom = [30.0,30.0]; % for bump testing
% u_nom = [24;7]; % plate to the right of the robot
% u_nom = [-20.0193;-58.7167]; % plate to the left of the robot

% Debugging:
% disp("Controller got x=" + string(x));
disp("Control before supervisor:");
disp(u_nom);

% Assume the supervisor takes arguments this way.
u = supervisor(u_nom, x, constants);
% u = u_nom; % ignores the CBF completely 
%
disp("Control after supervisor:");
disp(u);

end