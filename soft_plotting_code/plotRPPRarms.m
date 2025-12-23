function [ Tfinal, rot_angle ] = plotRPPRarms(DH, L, q, optns)
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
n_limb = length(L);
show_links   = optns(1);
show_segment = optns(2);
for i = 1:n_limb
    eval(['syms q' num2str(i) ' L' num2str(i) ' real']);
end
numLinks = 4*n_limb;
% DH parameters of the robot arm, angle in radian, length in mm
theta = DH(1:numLinks, 2);
d     = DH(1:numLinks, 3);
alpha = DH(1:numLinks, 4);
a     = DH(1:numLinks, 5);
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

%TT = subs(TT, [L, q], [L1, q1]);
%TT = subs(TT, [L1, q1], [L(1), q(1)]);
for i = 1:length(q)
   if abs(q(i)) < 1e-7
       q(i) = q(i) + 1e-7;
   end
end

for i = 1:n_limb
syms_list = [sym(['L', num2str(i)]), sym(['q', num2str(i)])];
values = [L(i), q(i)];
TT = subs(TT, syms_list, values);
end

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
    [0.8078, 0.7608, 0.9216],  'SizeData', 100);

% plot X Y Z axes centered on the segment's end
% quiver3(xJoint(end), yJoint(end), zJoint(end), TT(1, 1, end), ...
%     TT(2, 1, end), TT(3, 1, end), L(end)/4, 'b', 'LineWidth', 2)
% quiver3(xJoint(end), yJoint(end), zJoint(end), TT(1, 2, end), ...
%     TT(2, 2, end), TT(3, 2, end), L(end)/4, 'g', 'LineWidth', 2)
% quiver3(xJoint(end), yJoint(end), zJoint(end), TT(1, 3, end), ...
%     TT(2, 3, end), TT(3, 3, end), L(end)/4, 'r', 'LineWidth', 2)


% quiver3(xJoint(end), yJoint(end), zJoint(end), TT(1, 1, end), ...
%     TT(2, 1, end), TT(3, 1, end), L(end)/16, 'r', 'LineWidth', 2)
% quiver3(xJoint(end), yJoint(end), zJoint(end), TT(1, 2, end), ...
%     TT(2, 2, end), TT(3, 2, end), L(end)/16, 'g', 'LineWidth', 2)
% quiver3(xJoint(end), yJoint(end), zJoint(end), TT(1, 3, end), ...
%     TT(2, 3, end), TT(3, 3, end), L(end)/16, 'b', 'LineWidth', 2)

% quiver3(xJoint(1), yJoint(1), zJoint(1), 1,0,0, L(end)/4, 'b', 'LineWidth', 2)
% quiver3(xJoint(1), yJoint(1), zJoint(1), 0,1,0, L(end)/4, 'g', 'LineWidth', 2)
% quiver3(xJoint(1), yJoint(1), zJoint(1), 0,0,1, L(end)/4, 'r', 'LineWidth', 2)

scatter3(xJoint(end), yJoint(end), zJoint(end), 'filled',  'MarkerFaceColor', [0.8078, 0.7608, 0.9216])
scatter3(xJoint(end), yJoint(end), zJoint(end),  'filled',  'MarkerFaceColor', [0.8078, 0.7608, 0.9216])
scatter3(xJoint(end), yJoint(end), zJoint(end),  'filled',  'MarkerFaceColor', [0.8078, 0.7608, 0.9216])

% view(6.5651,34.0948)

% text(xJoint(end)+5, yJoint(end)-5, 'q='+string(q1)+' rad');

% axis([-2*L1 2*L1 -2*L1 2*L1 0 2*L1]);

% grid on
%title('3D plot of pseudorigid RPPR approx robot with CC segments', 'FontSize', 14)
xlabel('x-axis (m)', 'FontSize', 20)
ylabel('y-axis (m)', 'FontSize', 20)
zlabel('z-axis (m)', 'FontSize', 20)
view(2)
% hold off
Tfinal = double(TT(:, :, numLinks));

%plot the CC approximate of the soft segment
if show_segment
    for i = 1:n_limb
        %ccarc([xJoint((i-1)*4+1), yJoint((i-1)*4+1)]', [xJoint(i*4), yJoint(i*4)]', L(i), q(i));
        if i == 1
            tt_temp = eye(4);
        else
            tt_temp = TT(:,:,(i-1)*4); % get T base to last_joint_in_i-1_limb
        end
        rr = tt_temp(1:3,1:3); dd = tt_temp(1:3,4); 
        tt_rev = [rr',-rr'*dd;0,0,0,1];  % get T last_joint_in_i-1_limb to base 
        Joint_p_temp = tt_rev*[xJoint(i*4+1), yJoint(i*4+1),0,1]'; % get joint position wrt last_joint_in_i-1_limb
        [arc_pts,circ_ctr]=ccarc_wrt_last_limb([0, 0]', [Joint_p_temp(1), Joint_p_temp(2)], L(i), q(i));
        % treat every limb as if the first joint of the limb is located at
        % the orgin to find the arc and the circle center
        arc_pts = tt_temp*arc_pts; % transform to get the position wrt the base
        circ_ctr = tt_temp*circ_ctr; % transform to get the position wrt the base   
    plot(arc_pts(1,:),arc_pts(2,:),'Color', [0.4627, 0.0784, 0.6941], 'LineWidth', 3);
    %plot(arc_pts(1,:),arc_pts(2,:),'r-',circ_ctr(1),circ_ctr(2),'b*', 'LineWidth', 3);
    end
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