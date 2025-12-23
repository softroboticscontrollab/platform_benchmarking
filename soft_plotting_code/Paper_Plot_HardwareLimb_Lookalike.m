function [ Tfinal, rot_angle ] = Paper_Plot_HardwareLimb_Lookalike(DH, L, q, optns)
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
    eval(['TT = subs(TT, [L',num2str(i),', q',num2str(i),'], [L(',num2str(i),'), q(',num2str(i),')]);']);
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
    plot3(xJoint, yJoint, zJoint, 'k--', 'LineWidth', 8);
    % hold on
end



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
        % plot(arc_pts(1,:),arc_pts(2,:),'Color', [0.4627, 0.0784, 0.6941], 'LineWidth', 15);
        % plot(arc_pts(1,:),arc_pts(2,:),'Color', [0.6 0.5 0.8], 'LineWidth', 15);
    
        % === Convert to numeric early! ===
        arc_pts = double(arc_pts);  % important!
        
        % Extract arc coordinates
        x_arc = arc_pts(1,:);
        y_arc = arc_pts(2,:);
        z_arc = arc_pts(3,:);
        
        % Compute tangents and normals
        dx = gradient(x_arc);
        dy = gradient(y_arc);
        mag = sqrt(dx.^2 + dy.^2 + 1e-12);
        nx = -dy ./ mag;
        ny = dx ./ mag;
        
        % ribbon offset
        limb_thickness = 0.038;
        
        x_top = x_arc + limb_thickness/2 * nx;
        y_top = y_arc + limb_thickness/2 * ny;
        z_top = z_arc;
        
        x_bot = x_arc - limb_thickness/2 * nx;
        y_bot = y_arc - limb_thickness/2 * ny;
        z_bot = z_arc;
        
        x_fill = [x_top, fliplr(x_bot)];
        y_fill = [y_top, fliplr(y_bot)];
        z_fill = [z_top, fliplr(z_bot)];
        
        fill3(x_fill, y_fill, z_fill, [0.68, 0.66, 0.89], 'EdgeColor', 'none');
        
        % === ADD ROUNDED CAPS ===
        n_cap = 1000;
        theta_cap = linspace(0, pi, n_cap);
        r = limb_thickness / 2;
        
        % Start cap
        x_start = x_arc(1); y_start = y_arc(1); z_start = z_arc(1);
        angle = atan2(dy(1), dx(1));
        cap_x1 = x_start + r * cos(theta_cap) * cos(angle) - r * sin(theta_cap) * sin(angle);
        cap_y1 = y_start + r * cos(theta_cap) * sin(angle) + r * sin(theta_cap) * cos(angle);
        cap_z1 = z_start * ones(1, n_cap);
        cap_x2 = x_start - r * cos(theta_cap) * cos(angle) + r * sin(theta_cap) * sin(angle);
        cap_y2 = y_start - r * cos(theta_cap) * sin(angle) - r * sin(theta_cap) * cos(angle);
        cap_z2 = z_start * ones(1, n_cap);
        fill3(cap_x1, cap_y1, cap_z1, [0.68, 0.66, 0.89], 'EdgeColor', 'none');
        fill3(cap_x2, cap_y2, cap_z2, [0.68, 0.66, 0.89], 'EdgeColor', 'none');
        
        % End cap
        x_end = x_arc(end); y_end = y_arc(end); z_end = z_arc(end);
        angle = atan2(dy(end), dx(end));
        cap_x3 = x_end + r * cos(theta_cap) * cos(angle) - r * sin(theta_cap) * sin(angle);
        cap_y3 = y_end + r * cos(theta_cap) * sin(angle) + r * sin(theta_cap) * cos(angle);
        cap_z3 = z_end * ones(1, n_cap);
        cap_x4 = x_end - r * cos(theta_cap) * cos(angle) + r * sin(theta_cap) * sin(angle);
        cap_y4 = y_end - r * cos(theta_cap) * sin(angle) - r * sin(theta_cap) * cos(angle);
        cap_z4 = z_end * ones(1, n_cap);
        fill3(cap_x3, cap_y3, cap_z3, [0.68, 0.66, 0.89], 'EdgeColor', 'none');
        fill3(cap_x4, cap_y4, cap_z4, [0.68, 0.66, 0.89], 'EdgeColor', 'none');




    %plot(arc_pts(1,:),arc_pts(2,:),'r-',circ_ctr(1),circ_ctr(2),'b*', 'LineWidth', 3);
    end
end

% Show joint markers at top, middle, and bottom
marker_indices = round(linspace(1, length(xJoint), 3));

% AprilTag-style squares (white w/ black center)
tag_size = 0.02;        % outer square size
inner_scale = 0.6;      % black square relative size

% We'll plot in 2D view (Z=0), so drop to 2D rectangles
for i = 1:length(marker_indices)
    x = xJoint(marker_indices(i));
    y = yJoint(marker_indices(i));
    
    % Outer white square
    x0 = x - tag_size/2;
    y0 = y - tag_size/2;
    rectangle('Position', [x0, y0, tag_size, tag_size], ...
              'FaceColor', 'w', 'EdgeColor', 'k', 'LineWidth', 1.2);

    % Inner black square
    inner_size = tag_size * inner_scale;
    xi = x - inner_size/2;
    yi = y - inner_size/2;
    rectangle('Position', [xi, yi, inner_size, inner_size], ...
              'FaceColor', 'k', 'EdgeColor', 'none');
end

scatter3(xJoint(marker_indices), yJoint(marker_indices), zJoint(marker_indices), ...
    'filled', 'MarkerFaceColor', [0 1 0], 'SizeData', 50);  % bright green

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