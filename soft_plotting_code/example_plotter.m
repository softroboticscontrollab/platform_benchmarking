%% Example Code for plotting of data 
clear all; close all; clc;

%% Setup
% values here are chosen based on physical hardware

% length of limb 1
l1 = 0.122; 
% length of limb 2
l2 = 0.122; 
epsilon = 1e-6; 
% calculated spring constant of deformable force plate   
k = 11.16; 
% allowable max force on force plate - used for force-critical tasks
Fmax = 11.16 * 1.6 / 100; 

%% Pulling reference data used for trajectory generation 

RTJ = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-4_121806.csv';
% RTJ = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-2_151809.csv';

RTJ = readtable(RTJ, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');

time_RTJ     = RTJ.("Test time");
time_RTJ     = time_RTJ(2:end);
theta_0_RTJ  = RTJ.theta_0;
theta_0_RTJ  = theta_0_RTJ(2:end);
theta_1_RTJ  = RTJ.theta_1;
theta_1_RTJ  = theta_1_RTJ(2:end);

startidx_RTJ = find(RTJ.("u_t(0)"), 1, 'first');

% Convert angles from degrees to radians
q0_RTJ  = deg2rad(theta_0_RTJ*2);
q1_RTJ  = deg2rad(theta_1_RTJ*2);

% Derived Control Input 
DCI0_RTJ = RTJ.("MPRpressure_0") - RTJ.("MPRpressure_1");
DCI0_RTJ = DCI0_RTJ(2:end);
DCI1_RTJ = RTJ.("MPRpressure_2") - RTJ.("MPRpressure_3"); 
DCI1_RTJ = DCI1_RTJ(2:end);

% Solve for end effector position using forward kinematics
Tip_x_RTJ = l2 * ((cos(q0_RTJ) .* sin(q1_RTJ) + sin(q0_RTJ) .* cos(q1_RTJ) - sin(q0_RTJ)) ./ q1_RTJ) ...
    + l1 * (sin(q0_RTJ) ./ q0_RTJ);
Tip_y_RTJ = l2 * ((sin(q0_RTJ) .* sin(q1_RTJ) - cos(q0_RTJ) .* cos(q1_RTJ) + cos(q0_RTJ)) ./ q1_RTJ) ...
    + l1 * ((1 - cos(q0_RTJ)) ./ q0_RTJ);

%% Pulling data from experiment XXX 

% XXX = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-2-12_154549.csv';
% XXX = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-2_154906.csv';
XXX = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-4_122516.csv';
YYY = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-2-13_144529.csv';

XXX = readtable(XXX, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');

time_XXX     = XXX.("Test time");
theta_0_XXX  = XXX.theta_0;
theta_1_XXX  = XXX.theta_1;

Fp0_XXX      = XXX.ForcePlate_0;

% time_XXX = time_XXX - time_XXX(startidx_XXX);

% Convert angles from degrees to radians
q0_XXX  = deg2rad(theta_0_XXX)*2;
q1_XXX  = deg2rad(theta_1_XXX)*2;

% Pressure data if needed 
L0_P0_XXX = XXX.("MPRpressure_0");
%L0_P0_FIL_XXX = XXX.("FILpressure_0");

% Valve aperture data 
L0_val0_XXX = XXX.("ValveOpening_0");

% Derived Control Input 
DCI0_XXX = XXX.("MPRpressure_0") - XXX.("MPRpressure_1");
DCI1_XXX = XXX.("MPRpressure_2") - XXX.("MPRpressure_3");

% Actual Control Input
ACI0_XXX = XXX.("u_t(0)");
ACI1_XXX = XXX.("u_t(1)");

% Solve for end effector position using forward kinematics
Tip_x_XXX = l2 * ((cos(q0_XXX) .* sin(q1_XXX) + sin(q0_XXX) .* cos(q1_XXX) - sin(q0_XXX)) ./ q1_XXX) ...
    + l1 * (sin(q0_XXX) ./ q0_XXX);
Tip_y_XXX = l2 * ((sin(q0_XXX) .* sin(q1_XXX) - cos(q0_XXX) .* cos(q1_XXX) + cos(q0_XXX)) ./ q1_XXX) ...
    + l1 * ((1 - cos(q0_XXX)) ./ q0_XXX);

%% Plotting Time 

startidx_XXX = find(XXX.("u_t(0)"), 1, 'first');

time_XXX     = time_XXX - time_XXX(startidx_XXX);

figure(1)
subplot(1,2,1); plot(time_RTJ,q0_RTJ,LineWidth=2);
hold on 
plot(time_XXX(startidx_XXX:end),q0_XXX(startidx_XXX:end),LineWidth=2)
title('Raw State Ref Data for q0')
subplot(1,2,2); plot(time_RTJ,q1_RTJ,LineWidth=2);
hold on 
plot(time_XXX(startidx_XXX:end),q1_XXX(startidx_XXX:end),LineWidth=2)
title('Raw State Ref Data for q1')
hold off

figure(2)
subplot(1,2,1); plot(time_RTJ,DCI0_RTJ,LineWidth=2);
hold on 
plot(time_XXX(startidx_XXX:end),DCI0_XXX(startidx_XXX:end),LineWidth=2)
plot(time_XXX(startidx_XXX:end),ACI0_XXX(startidx_XXX:end),LineWidth=2)
title('Raw Control Input vs Derived L0')
subplot(1,2,2); plot(time_RTJ,DCI1_RTJ,LineWidth=2);
hold on 
plot(time_XXX(startidx_XXX:end),DCI1_XXX(startidx_XXX:end),LineWidth=2)
plot(time_XXX(startidx_XXX:end),ACI1_XXX(startidx_XXX:end),LineWidth=2)
title('Raw Control Input vs Derived L1')
hold off

[dq0_RTJ, ddq0_RTJ] = sgolay_derivatives(q0_RTJ, time_RTJ, 241, 3);
[dq1_RTJ, ddq1_RTJ] = sgolay_derivatives(q1_RTJ, time_RTJ, 241, 3);

e0 = q0_RTJ - q0_XXX(startidx_XXX:startidx_XXX + length(q0_RTJ)-1);
e1 = q1_RTJ - q1_XXX(startidx_XXX:startidx_XXX + length(q1_RTJ)-1);

[de0, dde0] = sgolay_derivatives(e0, time_XXX(startidx_XXX:end), 241, 3);
[de1, dde1] = sgolay_derivatives(e1, time_XXX(startidx_XXX:end), 241, 3);

figure(3) 
subplot(2,2,1); plot(time_RTJ,e0);title('e0')
subplot(2,2,2); plot(time_RTJ,e1);title('e1')
subplot(2,2,3); plot(time_RTJ,de0);title('de0')
subplot(2,2,4); plot(time_RTJ,de1);title('de1')

% figure(2)
% subplot(1,2,1); plot(time_RTJ,dq0);
% title('sgolay dq0')
% subplot(1,2,2); plot(time_RTJ,dq1);
% title('sgolay dq1')
% 
% fdq0  = griddedInterpolant(time_RTJ, dq0,  'linear', 'nearest');
% fdq1  = griddedInterpolant(time_RTJ, dq1,  'linear', 'nearest');
% 
% figure(3)
% subplot(1,2,1); plot(time_RTJ,fdq0(time_RTJ));
% title('sgolay fdq0')
% subplot(1,2,2); plot(time_RTJ,fdq1(time_RTJ));
% title('sgolay fdq1')

%% Used for solving for velocities and accelerations from reference trajectory 

function [d1, d2] = sgolay_derivatives(y, t, window_length, poly_order)
        % Computes smoothed first and second derivatives using Savitzky-Golay method
        half_window = (window_length - 1) / 2;
        n = length(y);
        d1 = zeros(size(y));
        d2 = zeros(size(y));

        for i = 1:n
            idx = max(1, i - half_window):min(n, i + half_window);
            ti = t(idx);
            yi = y(idx);

            t_center = mean(ti);
            t_norm = ti - t_center;
            t_scale = max(abs(t_norm));
            if t_scale == 0
                t_scale = 1; 
            end
            t_norm = t_norm / t_scale;

            p = polyfit(t_norm, yi, poly_order);

            dp = polyder(p);
            ddp = polyder(dp);

            d1(i) = polyval(dp, 0) / t_scale;
            d2(i) = polyval(ddp, 0) / (t_scale^2);
        end
end