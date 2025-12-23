function [ Tfinal, rot_angle ] = plotRPPRarm(DH, L1, q1, optns)
%plotRPPRarm  This function solves the forward kinematics of 4 DOF RPPR
% robot arm and draw each link (dotted line), joint (circle), and constant
% curvature CC segment (solid line) described by the pseudorigid approximation.
% MODIFIED by: Charlie DeLorey, from WPI robot dynamics homework code

% Inputs::
%    DH    : table of DH parameters for RPPR robot
%    L1    : length of segment (in m)
%    q1    : input value (which for CC should be 2 itmes end effector to base angle), angle in radians 
%    optns : plotting options (showing soft segment, rigid robot links)

% Returns::
%    3D plot of the arm
%    Tfinal    : 4x4 transfomation matrix with respect to the base frame, 
%                transformation from one frame to another is done through 
%                D-H convention. See dh2transMatrix.m for more details.
%    rot_angle : 1x3 Euler angle rotations

% Notes::
%    - This does 3D plotting, but math may not be correct to actually
%      try to bring the CC segments into 3D.
%    - **TODO: make plotting work with negative (rad) angles

show_links   = optns(1);
show_segment = optns(2);

syms q L real;
numLinks = 4;

% DH parameters of the robot arm, angle in radian, length in mm
theta = DH(:, 2);
d     = DH(:, 3);
alpha = DH(:, 4);
a     = DH(:, 5);

% Transformation between frames
for n=1:numLinks
    T(:, :, n) = dh2transMatrix(theta(n), d(n), alpha(n), a(n), 1);   
             % calculation done in radian for symbolic expression
end

% Composite transformation
TT(:, :, 1) = T(:, :, 1);
for n=2:numLinks
    TT(:, :, n) = TT(:, :, n-1) * T(:, :, n);
end


% Transformation matrix if q = 0 (??)
% TT = [1, 0, 0, L;
%       0, 1, 0, 0;
%       0, 0, 1, 0;
%       0, 0, 0, 1];

% Find composite transformation for home and desired configurations
% TT_home = subs(TT, [q, q, q, q], zeros(1,4));
TT = subs(TT, [L, q], [L1, q1]);

% Coordinates of the joints/frames
xJoint=[0, 0, 0, 0, 0];
yJoint=[0, 0, 0, 0, 0];
zJoint=[0, 0, 0, 0, 0];
for n=1:numLinks
    xJoint(n+1) = TT(1, 4, n);
    yJoint(n+1) = TT(2, 4, n);
    zJoint(n+1) = TT(3, 4, n);
end

% figWidth  = 600; % pixels
% figHeight = 450; % pixels
% rect = [0 50 figWidth figHeight];
% fig = figure('OuterPosition',rect);

% plot the pseudorigid RPPR robot
if show_links
    plot3(xJoint, yJoint, zJoint, 'k--', 'LineWidth', 3);
    % hold on
end

% plot segment ends as blueish circles
scatter3(xJoint, yJoint, zJoint, 'filled',  'MarkerFaceColor', ...
    [0.3010 0.7450 0.9330],  'SizeData', 100);

% plot X Y Z axes centered on the segment's end
quiver3(xJoint(end), yJoint(end), zJoint(end), TT(1, 1, 4), ...
    TT(2, 1, 4), TT(3, 1, 4), L1/5, 'r', 'LineWidth', 2)
quiver3(xJoint(end), yJoint(end), zJoint(end), TT(1, 2, 4), ...
    TT(2, 2, 4), TT(3, 2, 4), L1/5, 'g', 'LineWidth', 2)
quiver3(xJoint(end), yJoint(end), zJoint(end), TT(1, 3, 4), ...
    TT(2, 3, 4), TT(3, 3, 4), L1/5, 'b', 'LineWidth', 2)

% view(6.5651,34.0948)

% text(xJoint(end)+5, yJoint(end)-5, 'q='+string(q1)+' rad');

% axis([-2*L1 2*L1 -2*L1 2*L1 0 2*L1]);

% grid on
title('3D plot of pseudorigid RPPR approx robot with CC segments', 'FontSize', 14)
xlabel('x-axis (m)', 'FontSize', 20)
ylabel('y-axis (m)', 'FontSize', 20)
zlabel('z-axis (m)', 'FontSize', 20)
view(2)
% hold off
Tfinal = double(TT(:, :, numLinks));

%plot the CC approximate of the soft segment
if show_segment
    ccarc([0, 0]', [xJoint(end), yJoint(end)]', L1, q1);
end

% Convert rotation matrix to Euler angles
singularity_check = sqrt(Tfinal(1, 1)^2 + Tfinal(2, 1)^2);
if singularity_check < 10^-6
  PHI = atan2(Tfinal(2, 3), Tfinal(2, 2)); % corresponds to RX in Robot Studio
  THETA = atan2(-Tfinal(3, 1), sqrt(Tfinal(1, 1)^2+Tfinal(2, 1)^2)); % corresponds to RY in Robot Studio
  PSI = 0; % corresponds to RZ in Robot Studio
else
    THETA = atan2(-Tfinal(3, 1), sqrt(Tfinal(1, 1)^2+Tfinal(2, 1)^2)); % corresponds to RY in Robot Studio
    PHI   = atan2(Tfinal(3, 2),Tfinal(3, 3)); % corresponds to RX in Robot Studio
    PSI   = atan2(Tfinal(2, 1),Tfinal(1, 1)); % corresponds to RZ in Robot Studio
end

rot_angle = [PHI, THETA, PSI] * 180/pi;
end