function [arc_pts,circ_ctr]=ccarc_wrt_last_limb(orig, endpt, L, theta)
%ccarc  plots arc of constant curvature between two 2D points with a given
% segment length and angle theta.
% Author: Charlie DeLorey

% Inputs:: 
%    orig   : 2x1 vector of segment origin coordinates
%    endpt  : 2x1 vector of segment endpoint coordinates
%    L      : segment length (in mm, at least for `plotRPPRarm` 
%    theta  : segment angle from base/origin, also called `q` in the CC pseudorigid
%             formalization. 

% Notes::

% Resources used::
%  https://www.mathworks.com/matlabcentral/answers/367126-plot-an-arc-on-a-2d-grid-by-given-radius-and-end-points
    
    % To avoid singularity in theta = 0 position, add a small value
    if abs(theta) < 1e-7
       theta = theta + 1e-7;
    end
    R = L / wrapTo2Pi(abs(theta)); % R radius is found from segment's curvature : kappa=L/theta
    % %    axis equal
    p = [orig(1),endpt(1);orig(2),endpt(2)];
    a = sym('a',[2,1],'real');
    eqs = [1,1]*(p - repmat(a(:),1,2)).^2 - R^2;
    sol = vpasolve(eqs,a);
    ss = struct2cell(sol);
    xy = double([ss{:}]);
    % example: The arc of a circle with center at xy(1,:)
    angle_orig2endpt = atan2(endpt(2)-orig(2),endpt(1)-orig(1));
    angle_orig2center = atan2(xy(1,2)-orig(2),xy(1,1)-orig(1));
    if size(xy,1) >1 % more than one solution
        % -pi to -pi/2 and 0 to pi/2 
        if (angle_orig2endpt <= pi()/2 && angle_orig2endpt >= 0) || ...
            (angle_orig2endpt <= -pi()/2 && angle_orig2endpt > -pi())   
            if angle_orig2center > angle_orig2endpt
                v = xy(1,:);
                if (angle_orig2endpt <= -pi()/2 && angle_orig2endpt > -pi()) && angle_orig2center*angle_orig2endpt < 0
                    v = xy(2,:);
                end
            else
                v = xy(2,:);
            end
        else  % -pi/2 to 0 and pi/2 to pi
            if angle_orig2center < angle_orig2endpt
                v = xy(1,:);
                if (angle_orig2endpt > pi()/2 && angle_orig2endpt <= pi()) && angle_orig2center*angle_orig2endpt < 0
                    v = xy(2,:);
                end
            else
                v = xy(2,:);
            end
        end
    else %only one solution
        v = xy(1,:);
    end
    % v is the derived centre of the circle given R and the starting
    % and ending point of the arc
    p1 = p - v(:);
    vec_orig = p1(:,1);
    vec_endpt = p1(:,2);
    angle_orig = atan2(vec_orig(2),vec_orig(1));
    angle_endpt = atan2(vec_endpt(2),vec_endpt(1));
    if vec_orig(2)>0     % circle center is below the origin
        if angle_endpt >= pi()/2 &&  angle_endpt <= pi()
            angle_endpt = angle_endpt - 2*pi();
        end
    elseif vec_orig(2)<0 % circle center is above the origin
        if angle_endpt <= -pi()/2 &&  angle_endpt >= -pi()
            angle_endpt = angle_endpt + 2*pi();
        end
    end
    arc_pts_density = 50;
    phi = linspace(angle_orig,angle_endpt,arc_pts_density)';
    %plot(r*cos(phi) + v(1),r*sin(phi) + v(2),'r-',v(1),v(2),'b*', 'LineWidth', 3);
    arc_pts_x = R*cos(phi) + v(1);
    arc_pts_y = R*sin(phi) + v(2);
    circ_ctr = [v(1);v(2);0;1];
    arc_pts = [arc_pts_x,arc_pts_y,zeros(arc_pts_density,1),ones(arc_pts_density,1)]';
end