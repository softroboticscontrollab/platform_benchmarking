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

%% Reference Trajectory 

RTJ = 'plotting_data/sinwave_traj.csv';

RTJ = readtable(RTJ, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');

time_RTJ     = RTJ.("Test time");
time_RTJ     = time_RTJ(2:end);
theta_0_RTJ  = RTJ.theta_0;
theta_0_RTJ  = theta_0_RTJ(2:end);
theta_1_RTJ  = RTJ.theta_1;
theta_1_RTJ  = theta_1_RTJ(2:end);

u_RTJ        = RTJ.("u_t(0)");

startidx_RTJ = find(RTJ.("u_t(0)"), 1, 'first');

endidx_RTJ   = find(RTJ.("u_t(0)"), 1, 'last');

last_time = endidx_RTJ - startidx_RTJ;

time_RTJ = time_RTJ - time_RTJ(startidx_RTJ);

% Convert angles from degrees to radians
q0_RTJ  = deg2rad(theta_0_RTJ*2);
q1_RTJ  = deg2rad(theta_1_RTJ*2);

%% Range values of q = [q0,q1] for PD+ w/o force plate

updwo1 = 'exp_data/upd_wo1_v2';
updwo2 = 'exp_data/upd_wo2_v2';
updwo3 = 'exp_data/upd_wo3_v2';
updwo4 = 'exp_data/upd_wo4_v2';
updwo5 = 'exp_data/upd_wo5_v2';

updwo1 = readtable(updwo1, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');
updwo2 = readtable(updwo2, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');
updwo3 = readtable(updwo3, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');
updwo4 = readtable(updwo4, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');
updwo5 = readtable(updwo5, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');

time_updwo1     = updwo1.("Test time");
time_updwo2     = updwo2.("Test time");
time_updwo3     = updwo3.("Test time");
time_updwo4     = updwo4.("Test time");
time_updwo5     = updwo5.("Test time");

q0_updwo1 = deg2rad(updwo1.theta_0*2);
q1_updwo1 = deg2rad(updwo1.theta_1*2);
q0_updwo2 = deg2rad(updwo2.theta_0*2);
q1_updwo2 = deg2rad(updwo2.theta_1*2);
q0_updwo3 = deg2rad(updwo3.theta_0*2);
q1_updwo3 = deg2rad(updwo3.theta_1*2);
q0_updwo4 = deg2rad(updwo4.theta_0*2);
q1_updwo4 = deg2rad(updwo4.theta_1*2);
q0_updwo5 = deg2rad(updwo5.theta_0*2);
q1_updwo5 = deg2rad(updwo5.theta_1*2);

start_idx1 = find(updwo1.("u_t(0)"), 1, 'first') + startidx_RTJ;
start_idx2 = find(updwo2.("u_t(0)"), 1, 'first') + startidx_RTJ; 
start_idx3 = find(updwo3.("u_t(0)"), 1, 'first') + startidx_RTJ;
start_idx4 = find(updwo4.("u_t(0)"), 1, 'first') + startidx_RTJ;
start_idx5 = find(updwo5.("u_t(0)"), 1, 'first') + startidx_RTJ;

end_idx1 = start_idx1 + last_time;
end_idx2 = start_idx2 + last_time;
end_idx3 = start_idx3 + last_time;
end_idx4 = start_idx4 + last_time;
end_idx5 = start_idx5 + last_time;

time_updwo1     = time_updwo1 - time_updwo1(start_idx1);
time_updwo2     = time_updwo2 - time_updwo2(start_idx2);
time_updwo3     = time_updwo3 - time_updwo3(start_idx3);
time_updwo4     = time_updwo4 - time_updwo4(start_idx4);
time_updwo5     = time_updwo5 - time_updwo5(start_idx5);

q0_updwo1 = q0_updwo1(start_idx1:end_idx1);
q1_updwo1 = q1_updwo1(start_idx1:end_idx1);
q0_updwo2 = q0_updwo2(start_idx2:end_idx2);
q1_updwo2 = q1_updwo2(start_idx2:end_idx2);
q0_updwo3 = q0_updwo3(start_idx3:end_idx3);
q1_updwo3 = q1_updwo3(start_idx3:end_idx3);
q0_updwo4 = q0_updwo4(start_idx4:end_idx4);
q1_updwo4 = q1_updwo4(start_idx4:end_idx4);
q0_updwo5 = q0_updwo5(start_idx5:end_idx5);
q1_updwo5 = q1_updwo5(start_idx5:end_idx5);

q0_updwo_all = [q0_updwo1,q0_updwo2,q0_updwo3,q0_updwo4,q0_updwo5];
mu = mean(q0_updwo_all,2);
sigma = std(q0_updwo_all,0,2);
q0_updwo_upper = mu + 2*sigma;
q0_updwo_lower = mu - 2*sigma;
q0_updwo_mean = mu;

q1_updwo_all = [q1_updwo1,q1_updwo2,q1_updwo3,q1_updwo4,q1_updwo5];
mu = mean(q1_updwo_all,2);
sigma = std(q1_updwo_all,0,2);
q1_updwo_upper = mu + 2*sigma;
q1_updwo_lower = mu - 2*sigma;
q1_updwo_mean = mu;

% q0

Plot1 = figure(1);
subplot(2,1,1);

x_fill = time_updwo1(start_idx1:end_idx1);
y_upper = q0_updwo_upper(:);
y_lower = q0_updwo_lower(:);

fill([x_fill; flipud(x_fill)], ...
     [y_upper; flipud(y_lower)], ...
     [0 0.604 0.192], 'EdgeColor', 'none','FaceAlpha', 0.2);
hold on 

p1 = plot(time_RTJ(startidx_RTJ:end),q0_RTJ(startidx_RTJ:end),LineWidth=2,Color=[0.25 0.25 0.25]);
p2 = plot(time_RTJ(startidx_RTJ:end),q0_updwo_mean(2:end),LineWidth=1,Color=[0 0.604 0.192]);

lgd = legend([p1 p2],{'Reference Trajectory','Achieved Trajectory'},Location='northeast');    
lgd.FontSize = 15;

ylabel('q_0 (radians)','FontSize',14) 
% xlabel('Time (s)','FontSize',14) 

title('Tracking with PD w/o Force Plate')

hold off 

% q1 

subplot(2,1,2);

x_fill = time_updwo1(start_idx1:end_idx1);
y_upper = q1_updwo_upper(:);
y_lower = q1_updwo_lower(:);

fill([x_fill; flipud(x_fill)], ...
     [y_upper; flipud(y_lower)], ...
     [0.188 0.361 0.92], 'EdgeColor', 'none','FaceAlpha', 0.2);
hold on 

p1 = plot(time_RTJ(startidx_RTJ:end),q1_RTJ(startidx_RTJ:end),LineWidth=2,Color=[0.25 0.25 0.25]);
p2 = plot(time_RTJ(startidx_RTJ:end),q1_updwo_mean(2:end),LineWidth=1,Color=[0.188 0.361 0.92]);

lgd = legend([p1 p2],{'Reference Trajectory','Achieved Trajectory'},Location='northeast');    
lgd.FontSize = 15;

ylabel('q_1 (radians)','FontSize',14) 
xlabel('Time (s)','FontSize',14) 

% title('Tracking with PD+')

hold off 


%% Range values of q = [q0,q1] for PD+ w force plate



%% Range values of q = [q0,q1] for ID w/o force plate

uidwo1 = 'exp_data/uid_wo1';
uidwo2 = 'exp_data/uid_wo2';
uidwo3 = 'exp_data/uid_wo3';
uidwo4 = 'exp_data/uid_wo4';
uidwo5 = 'exp_data/uid_wo5';

uidwo1 = readtable(uidwo1, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');
uidwo2 = readtable(uidwo2, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');
uidwo3 = readtable(uidwo3, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');
uidwo4 = readtable(uidwo4, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');
uidwo5 = readtable(uidwo5, ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');

time_uidwo1     = uidwo1.("Test time");
time_uidwo2     = uidwo2.("Test time");
time_uidwo3     = uidwo3.("Test time");
time_uidwo4     = uidwo4.("Test time");
time_uidwo5     = uidwo5.("Test time");

q0_uidwo1 = deg2rad(uidwo1.theta_0*2);
q1_uidwo1 = deg2rad(uidwo1.theta_1*2);
q0_uidwo2 = deg2rad(uidwo2.theta_0*2);
q1_uidwo2 = deg2rad(uidwo2.theta_1*2);
q0_uidwo3 = deg2rad(uidwo3.theta_0*2);
q1_uidwo3 = deg2rad(uidwo3.theta_1*2);
q0_uidwo4 = deg2rad(uidwo4.theta_0*2);
q1_uidwo4 = deg2rad(uidwo4.theta_1*2);
q0_uidwo5 = deg2rad(uidwo5.theta_0*2);
q1_uidwo5 = deg2rad(uidwo5.theta_1*2);

start_idx1 = find(uidwo1.("u_t(0)"), 1, 'first') + startidx_RTJ;
start_idx2 = find(uidwo2.("u_t(0)"), 1, 'first') + startidx_RTJ; 
start_idx3 = find(uidwo3.("u_t(0)"), 1, 'first') + startidx_RTJ;
start_idx4 = find(uidwo4.("u_t(0)"), 1, 'first') + startidx_RTJ;
start_idx5 = find(uidwo5.("u_t(0)"), 1, 'first') + startidx_RTJ;

end_idx1 = start_idx1 + last_time;
end_idx2 = start_idx2 + last_time;
end_idx3 = start_idx3 + last_time;
end_idx4 = start_idx4 + last_time;
end_idx5 = start_idx5 + last_time;

time_uidwo1     = time_uidwo1 - time_uidwo1(start_idx1);
time_uidwo2     = time_uidwo2 - time_uidwo2(start_idx2);
time_uidwo3     = time_uidwo3 - time_uidwo3(start_idx3);
time_uidwo4     = time_uidwo4 - time_uidwo4(start_idx4);
time_uidwo5     = time_uidwo5 - time_uidwo5(start_idx5);

q0_uidwo1 = q0_uidwo1(start_idx1:end_idx1);
q1_uidwo1 = q1_uidwo1(start_idx1:end_idx1);
q0_uidwo2 = q0_uidwo2(start_idx2:end_idx2);
q1_uidwo2 = q1_uidwo2(start_idx2:end_idx2);
q0_uidwo3 = q0_uidwo3(start_idx3:end_idx3);
q1_uidwo3 = q1_uidwo3(start_idx3:end_idx3);
q0_uidwo4 = q0_uidwo4(start_idx4:end_idx4);
q1_uidwo4 = q1_uidwo4(start_idx4:end_idx4);
q0_uidwo5 = q0_uidwo5(start_idx5:end_idx5);
q1_uidwo5 = q1_uidwo5(start_idx5:end_idx5);

q0_uidwo_all = [q0_uidwo1,q0_uidwo2,q0_uidwo3,q0_uidwo4,q0_uidwo5];
mu = mean(q0_uidwo_all,2);
sigma = std(q0_uidwo_all,0,2);
q0_uidwo_upper = mu + 2*sigma;
q0_uidwo_lower = mu - 2*sigma;
q0_uidwo_mean = mu;

q1_uidwo_all = [q1_uidwo1,q1_uidwo2,q1_uidwo3,q1_uidwo4,q1_uidwo5];
mu = mean(q1_uidwo_all,2);
sigma = std(q1_uidwo_all,0,2);
q1_uidwo_upper = mu + 2*sigma;
q1_uidwo_lower = mu - 2*sigma;
q1_uidwo_mean = mu;

% q0

Plot1 = figure(2);
subplot(2,1,1);

x_fill = time_uidwo1(start_idx1:end_idx1);
y_upper = q0_uidwo_upper(:);
y_lower = q0_uidwo_lower(:);

fill([x_fill; flipud(x_fill)], ...
     [y_upper; flipud(y_lower)], ...
     [0 0.604 0.192], 'EdgeColor', 'none','FaceAlpha', 0.2);
hold on 

p1 = plot(time_RTJ(startidx_RTJ:end),q0_RTJ(startidx_RTJ:end),LineWidth=2,Color=[0.25 0.25 0.25]);
p2 = plot(time_RTJ(startidx_RTJ:end),q0_uidwo_mean(2:end),LineWidth=1,Color=[0 0.604 0.192]);

lgd = legend([p1 p2],{'Reference Trajectory','Achieved Trajectory'},Location='northeast');    
lgd.FontSize = 15;

ylabel('q_0 (radians)','FontSize',14) 
% xlabel('Time (s)','FontSize',14) 

title('Tracking with ID w/o Force Plate')

hold off 

% q1 

subplot(2,1,2);

x_fill = time_uidwo1(start_idx1:end_idx1);
y_upper = q1_uidwo_upper(:);
y_lower = q1_uidwo_lower(:);

fill([x_fill; flipud(x_fill)], ...
     [y_upper; flipud(y_lower)], ...
     [0.188 0.361 0.92], 'EdgeColor', 'none','FaceAlpha', 0.2);
hold on 

p1 = plot(time_RTJ(startidx_RTJ:end),q1_RTJ(startidx_RTJ:end),LineWidth=2,Color=[0.25 0.25 0.25]);
p2 = plot(time_RTJ(startidx_RTJ:end),q1_uidwo_mean(2:end),LineWidth=1,Color=[0.188 0.361 0.92]);

lgd = legend([p1 p2],{'Reference Trajectory','Achieved Trajectory'},Location='northeast');    
lgd.FontSize = 15;

ylabel('q_1 (radians)','FontSize',14) 
xlabel('Time (s)','FontSize',14) 

% title('Tracking with PD+')

hold off 


%% Range values of q = [q0,q1] for ID w force plate



