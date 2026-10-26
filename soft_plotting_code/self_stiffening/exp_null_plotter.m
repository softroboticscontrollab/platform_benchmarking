%% Plotting Code used for plotting of data in Paper
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

beta0 = -1.264908;
beta1 = 0.423700;
beta2 = -0.268104;

%% Example Code

RTJ = 'data/ezloophw_self_stiffening_2026-10-26_130010.csv';

RTJ = readtable(RTJ, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');

time     = RTJ.("Test time");
time     = time(2:end);
theta_0  = RTJ.theta_0;
theta_0  = theta_0(2:end);
theta_1  = RTJ.theta_1;
theta_1  = theta_1(2:end);

startidx = find(RTJ.("u_t(0)"), 1, 'first');

endidx   = find(RTJ.("u_t(0)"), 1, 'last');

u0 = RTJ.("u_t(0)")(startidx:endidx);
u1 = RTJ.("u_t(1)")(startidx:endidx);
u2 = RTJ.("u_t(2)")(startidx:endidx);
u3 = RTJ.("u_t(3)")(startidx:endidx);

last_time = endidx - startidx;

time = time(startidx:endidx);
time = time - time(startidx);

theta_0 = theta_0(startidx:endidx);
theta_1 = theta_1(startidx:endidx);

% Convert angles from degrees to radians
q0_rad  = deg2rad(theta_0*2);
q1_rad  = deg2rad(theta_1*2);

%% Plot

theta_0 = theta_0(startidx:endidx);

plot(time,theta_0*2)

%% Troubleshooting

p0 = RTJ.MPRpressure_0(startidx:endidx);
p1 = RTJ.MPRpressure_1(startidx:endidx);
p2 = RTJ.MPRpressure_2(startidx:endidx);
p3 = RTJ.MPRpressure_3(startidx:endidx);

p0 = p0 - p0(1);
p1 = p1 - p1(1);
p2 = p2 - p2(1);
p3 = p3 - p3(1);

figure(1)
plot(time,u0)
hold on
plot(time,p0)
hold off
title('u0 vs p0')

figure(2)
plot(time,u1)
hold on
plot(time,p1)
hold off
title('u1 vs p1')

Predict_angle = beta0 + beta1*p0 + beta2*p1;

figure(3)
plot(time,Predict_angle)
hold on
plot(time,theta_0*2)
hold off
title('Predicted Position vs Actual')

