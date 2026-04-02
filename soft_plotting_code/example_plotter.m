%% Example Code for plotting of data 
clear all; close all; clc;
l1 = 0.122; 
% length of limb 2
l2 = 0.122; 
epsilon = 1e-6; 
% calculated spring constant of deformable force plate   
k = 11.16; 
%% Using the new function 

% Format of data extractor and plotter. 
% data : .csv file location 
% doPlot : plots generated for general debugging 
% stepResponse : useful flag for visualizing step resposne behavior 
% sinResponse : useful when looking at sine data. Do not use for
% teach-and-repeat
% indexs : time values useful for plotting multiple data files on the same
% plot
% ezloopdata_compare(data,doplot,stepResponse,indexs,sinResponse)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% final tuning response for chamber 1
% dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC1.csv'; 
% ezloopdata_compare(dat, false, true);
% 
% % final tuning resposne for chamber 2
% dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC2.csv'; 
% ezloopdata_compare(dat, false, true);
% 
% % final tuning resposne for chamber 3
% dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC3.csv'; 
% ezloopdata_compare(dat, false, true);
% 
% % final tuning resposne for chamber 4
% dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC4.csv'; 
% ezloopdata_compare(dat, false, true);

% Open-loop motion for dynamics calibration with amplitude of 100 and period of 200
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_CalibrationRunV1.csv';
[time_cal, q_cal, dq_cal, ddq_cal, smooth_q_cal, smooth_dq_cal, smooth_ddq_cal, calib_start,~] = ezloopdata_compare(dat, false, false, true, 0);

% PD Tuning Results 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PD1.csv'; 
[time_PD1, q_PD1, dq_PD1, ddq_PD1, smooth_q_PD1, smooth_dq_PD1, smooth_ddq_PD1,~,~] = ezloopdata_compare(dat, false, false, true, 0);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PD2.csv'; 
[time_PD2, q_PD2, dq_PD2, ddq_PD2, smooth_q_PD2, smooth_dq_PD2, smooth_ddq_PD2,~,~] = ezloopdata_compare(dat, false, false, true, 0);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PD3.csv'; 
[time_PD3, q_PD3, dq_PD3, ddq_PD3, smooth_q_PD3, smooth_dq_PD3, smooth_ddq_PD3,~,~] = ezloopdata_compare(dat, false, false, true, 0);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PD4.csv'; 
[time_PD4, q_PD4, dq_PD4, ddq_PD4, smooth_q_PD4, smooth_dq_PD4, smooth_ddq_PD4,~,~] = ezloopdata_compare(dat, false, false, true, 0);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PD5.csv'; 
[time_PD5, q_PD5, dq_PD5, ddq_PD5, smooth_q_PD5, smooth_dq_PD5, smooth_ddq_PD5,~,~] = ezloopdata_compare(dat, false, false, true, 0);

smooth_q_cells = {smooth_q_PD1, smooth_q_PD2, smooth_q_PD3, ...
                  smooth_q_PD4, smooth_q_PD5};

time_cells = {time_PD1, time_PD2, time_PD3, time_PD4, time_PD5};

rho_cells = {};

time_step = time_PD1;

p_des = [deg2rad(20); -1*deg2rad(20)]; 

smooth_q_step = [p_des(1)*ones(length(time_PD1),1) p_des(2)*ones(length(time_PD1),1)];

TunePD = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_step, smooth_q_step, false, rho_cells,false);

% PD Attempt at Tracking Calibration 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal1.csv'; 
[time_PDCal1, q_PDCal1, dq_PDCal1, ddq_PDCal1, smooth_q_PDCal1, smooth_dq_PDCal1, smooth_ddq_PDCal1,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal2.csv'; 
[time_PDCal2, q_PDCal2, dq_PDCal2, ddq_PDCal2, smooth_q_PDCal2, smooth_dq_PDCal2, smooth_ddq_PDCal2,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal3.csv'; 
[time_PDCal3, q_PDCal3, dq_PDCal3, ddq_PDCal3, smooth_q_PDCal3, smooth_dq_PDCal3, smooth_ddq_PDCal3,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal4.csv'; 
[time_PDCal4, q_PDCal4, dq_PDCal4, ddq_PDCal4, smooth_q_PDCal4, smooth_dq_PDCal4, smooth_ddq_PDCal4,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_PDCal5.csv'; 
[time_PDCal5, q_PDCal5, dq_PDCal5, ddq_PDCal5, smooth_q_PDCal5, smooth_dq_PDCal5, smooth_ddq_PDCal5,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

smooth_q_cells = {smooth_q_PDCal1, smooth_q_PDCal2, smooth_q_PDCal3, ...
                  smooth_q_PDCal4, smooth_q_PDCal5};

time_cells = {time_PDCal1, time_PDCal2, time_PDCal3, time_PDCal4, time_PDCal5};

rho_cells = {};

CalibPD = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_cal, smooth_q_cal, false, rho_cells,false);

% CD Results tracking calib trajectory
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib1.csv'; 
[time_tra1, q_tra1, dq_tra1, ddq_tra1, smooth_q_tra1, smooth_dq_tra1, smooth_ddq_tra1,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib2.csv'; 
[time_tra2, q_tra2, dq_tra2, ddq_tra2, smooth_q_tra2, smooth_dq_tra2, smooth_ddq_tra2,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib3.csv'; 
[time_tra3, q_tra3, dq_tra3, ddq_tra3, smooth_q_tra3, smooth_dq_tra3, smooth_ddq_tra3,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib4.csv'; 
[time_tra4, q_tra4, dq_tra4, ddq_tra4, smooth_q_tra4, smooth_dq_tra4, smooth_ddq_tra4,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-25_TrackCalib5.csv'; 
[time_tra5, q_tra5, dq_tra5, ddq_tra5, smooth_q_tra5, smooth_dq_tra5, smooth_ddq_tra5,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

smooth_q_cells = {smooth_q_tra1, smooth_q_tra2, smooth_q_tra3, ...
                  smooth_q_tra4, smooth_q_tra5};

time_cells = {time_tra1, time_tra2, time_tra3, time_tra4, time_tra5};

rho_cells = {};

CalibCD = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_cal, smooth_q_cal, false, rho_cells, false);

%% To visualize how the filter works, below are two examples
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % AsymGaussian Filter Results 
% dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-19_122338.csv';
% [time_asym, q_asym, dq_asym, ddq_asym, smooth_q_asym, smooth_dq_asym, smooth_ddq_asym, ~, ~] = ezloopdata_compare(dat, false, false, true, 0);
% 
% % Butterworth Filter Results
% dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-19_123309.csv';
% [time_butter, q_butter, dq_butter, ddq_butter, smooth_q_butter, smooth_dq_butter, smooth_ddq_butter, ~, ~] = ezloopdata_compare(dat, false, false, true, 0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Dynamics Controller response to Teach-and-Repeat 4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Teach-and-Repeat 4 trajectory
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4.csv'; 
[time_TnR4, q_TnR4, dq_TnR4, ddq_TnR4, smooth_q_TnR4,smooth_dq_TnR4,smooth_ddq_TnR4, index_TnR4, ~] = ezloopdata_compare(dat, false, false, false, 0);

% Inverse Dynamics Controller on TnR4 w/o forceplate
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD1.csv'; 
[time_CD1, q_CD1, dq_CD1, ddq_CD1, smooth_q_CD1, smooth_dq_CD1, smooth_ddq_CD1, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD2.csv'; 
[time_CD2, q_CD2, dq_CD2, ddq_CD2, smooth_q_CD2, smooth_dq_CD2, smooth_ddq_CD2, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD3.csv'; 
[time_CD3, q_CD3, dq_CD3, ddq_CD3, smooth_q_CD3, smooth_dq_CD3, smooth_ddq_CD3, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD4.csv'; 
[time_CD4, q_CD4, dq_CD4, ddq_CD4, smooth_q_CD4, smooth_dq_CD4, smooth_ddq_CD4, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CD5.csv'; 
[time_CD5, q_CD5, dq_CD5, ddq_CD5, smooth_q_CD5, smooth_dq_CD5, smooth_ddq_CD5, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR4);

smooth_q_cells = {smooth_q_CD1, smooth_q_CD2, smooth_q_CD3, ...
                  smooth_q_CD4, smooth_q_CD5};

time_cells = {time_CD1, time_CD2, time_CD3, time_CD4, time_CD5};

rho_cells = {};

TnR = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR4, smooth_q_TnR4, false, rho_cells, false);

% Inverse Dynamics Controller on TnR4 w/ forceplate
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP1.csv'; 
[time_CDFP1, q_CDFP1, dq_CDFP1, ddq_CDFP1, smooth_q_CDFP1, smooth_dq_CDFP1, smooth_ddq_CDFP1, ~, rho0_CDFP1] = ezloopdata_compare(dat, false, false, false, index_TnR4);
 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP2.csv'; 
[time_CDFP2, q_CDFP2, dq_CDFP2, ddq_CDFP2, smooth_q_CDFP2, smooth_dq_CDFP2, smooth_ddq_CDFP2, ~, rho0_CDFP2] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP3.csv'; 
[time_CDFP3, q_CDFP3, dq_CDFP3, ddq_CDFP3, smooth_q_CDFP3, smooth_dq_CDFP3, smooth_ddq_CDFP3, ~, rho0_CDFP3] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP4.csv'; 
[time_CDFP4, q_CDFP4, dq_CDFP4, ddq_CDFP4, smooth_q_CDFP4, smooth_dq_CDFP4, smooth_ddq_CDFP4, ~, rho0_CDFP4] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_CD4FP5.csv'; 
[time_CDFP5, q_CDFP5, dq_CDFP5, ddq_CDFP5, smooth_q_CDFP5, smooth_dq_CDFP5, smooth_ddq_CDFP5, ~, rho0_CDFP5] = ezloopdata_compare(dat, false, false, false, index_TnR4);

smooth_q_cells = {smooth_q_CDFP1, smooth_q_CDFP2, smooth_q_CDFP3, ...
                  smooth_q_CDFP4, smooth_q_CDFP5};

rho_cells = {rho0_CDFP1, rho0_CDFP2, rho0_CDFP3, rho0_CDFP4, rho0_CDFP5};

time_cells = {time_CDFP1, time_CDFP2, time_CDFP3, time_CDFP4, time_CDFP5};

TnRCDFP = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR4, smooth_q_TnR4, true, rho_cells, true);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CBF Controller Response for High Conservative Case for Teach and Repeat 4

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH1.csv'; 
[time_H1, q_H1, dq_H1, ddq_H1, smooth_q_H1, smooth_dq_H1, smooth_ddq_H1, ~, rho0_H1] = ezloopdata_compare(dat, false, false, false, index_TnR4);
 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH2.csv'; 
[time_H2, q_H2, dq_H2, ddq_H2, smooth_q_H2, smooth_dq_H2, smooth_ddq_H2, ~, rho0_H2] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH3.csv'; 
[time_H3, q_H3, dq_H3, ddq_H3, smooth_q_H3, smooth_dq_H3, smooth_ddq_H3, ~, rho0_H3] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH4.csv'; 
[time_H4, q_H4, dq_H4, ddq_H4, smooth_q_H4, smooth_dq_H4, smooth_ddq_H4, ~, rho0_H4] = ezloopdata_compare(dat, false, false, false, index_TnR4);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-26_TnR4CBFH5.csv'; 
[time_H5, q_H5, dq_H5, ddq_H5, smooth_q_H5, smooth_dq_H5, smooth_ddq_H5, ~, rho0_H5] = ezloopdata_compare(dat, false, false, false, index_TnR4);

smooth_q_cells = {smooth_q_H1, smooth_q_H2, smooth_q_H3, ...
                  smooth_q_H4, smooth_q_H5};

rho_cells = {rho0_H1, rho0_H2, rho0_H3, rho0_H4, rho0_H5};

time_cells = {time_H1, time_H2, time_H3, time_H4, time_H5};

TnRHigh = ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR4, smooth_q_TnR4, true, rho_cells, true);

%% Example Plotting code used for Robosoft poster
%%%%%%%%%%%%%%%%%%%
sim_calib_data = load('callibration_data.mat');
sim_time = sim_calib_data.export_data(:,1);
sim_q1 = sim_calib_data.export_data(:,2);
sim_q2 = sim_calib_data.export_data(:,3);
sim_q1_inter = sim_calib_data.export_data(:,4);
sim_q2_inter = sim_calib_data.export_data(:,5);

% Side by Side of q1 q2 for Open-Loop trajectroy and CD tracking
% figure
% subplot(1,2,1)
% 
%     fill([CalibCD.time; flipud(CalibCD.time)], ...
%          [CalibCD.q0.upper; flipud(CalibCD.q0.lower)], ...
%          [0 0.604 0.192], ...
%          'EdgeColor','none','FaceAlpha',0.2);
%     hold on
% 
%     p1 = plot(time_cal, smooth_q_cal, ...
%               'LineWidth',2,'Color',[0.25 0.25 0.25]);
% 
%     p2 = plot(time_cal, CalibCD.q0.mu, ...
%               'LineWidth',1,'Color',[0 0.604 0.192]);

%% FIGURES for Paper 

single_column = true;
picturewidth_singlecolumn = 8.6*2 ; % cm

Fmax = 11.16 * 1.6 / 100;  

out1 = TnRCDFP;

    % Back-calculate raw force for every run
    out1.raw.force_all = Fmax * (1 - out1.raw.rho_all);

    % Best way: compute force statistics from the raw transformed data
    out1.force.mu    = mean(out1.raw.force_all, 2);
    out1.force.sigma = std(out1.raw.force_all, 0, 2);
    out1.force.upper = out1.force.mu + 2*out1.force.sigma;
    out1.force.lower = out1.force.mu - 2*out1.force.sigma;

    % Force fill data
    out1.force.fillX = [out1.time; flipud(out1.time)];
    out1.force.fillY = [out1.force.upper; flipud(out1.force.lower)];
    out1.force.fillColor = [0.850 0.325 0.098];
    out1.force.fillAlpha = 0.2;

out2 = TnRHigh;

    % Back-calculate raw force for every run
    out2.raw.force_all = Fmax * (1 - out2.raw.rho_all);

    % Best way: compute force statistics from the raw transformed data
    out2.force.mu    = mean(out2.raw.force_all, 2);
    out2.force.sigma = std(out2.raw.force_all, 0, 2);
    out2.force.upper = out2.force.mu + 2*out2.force.sigma;
    out2.force.lower = out2.force.mu - 2*out2.force.sigma;

    % Force fill data
    out2.force.fillX = [out2.time; flipud(out2.time)];
    out2.force.fillY = [out2.force.upper; flipud(out2.force.lower)];
    out2.force.fillColor = [0.850 0.325 0.098];
    out2.force.fillAlpha = 0.2;

F_max = Fmax*ones(length(out1.time),1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% rho plot
rho_plot = figure;
fill(out1.rho.fillX, out1.rho.fillY, [0 0.447 0.741], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);
hold on

fill(out2.rho.fillX, out2.rho.fillY, [0.850 0.325 0.098], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);

p1 = plot(out1.rho.time, out1.rho.mu, 'Color', [0 0.447 0.741], 'LineWidth', 1.5);
p2 = plot(out2.rho.time, out2.rho.mu, 'Color', [0.850 0.325 0.098], 'LineWidth', 1.5);
p3 = plot(out1.rho.time, out1.rho.max, '--r', 'LineWidth', 1.5);

ylabel('Rho','FontSize',15);xlabel('Time ($s$)','FontSize',15)
lgd = legend([p1,p2,p3],{'CD','CBF','Max Rho'},'Location','best','FontSize',12);

lgd.Units = 'normalized';       % Make position relative to axes
pos1 = lgd.Position;
pos1(1) = pos1(1) - 0.26;         % Move right
pos1(2) = pos1(2) + 0.3;         % Move up
lgd.Position = pos1;

hold off

hw_ratio = 1.1;

set(findall(rho_plot,'-property','Interpreter'), 'Interpreter', 'latex')
set(findall(rho_plot,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
set(findall(rho_plot,'-property','Box'), 'Box', 'off')
set(rho_plot, 'Units', 'centimeters', ...
        'Position', [2.3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);
grid on;

fname = 'fin_plots/Rho_Comparison';
exportgraphics(rho_plot, strcat(fname, '.png'), 'ContentType', 'vector');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% force plot
force_plot = figure;
fill(out1.force.fillX, out1.force.fillY, [0 0.447 0.741], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);
hold on

fill(out2.force.fillX, out2.force.fillY, [0.850 0.325 0.098], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.15);

p1 = plot(out1.time, out1.force.mu, 'Color', [0 0.447 0.741], 'LineWidth', 1.5);
p2 = plot(out2.time, out2.force.mu, 'Color', [0.850 0.325 0.098], 'LineWidth', 1.5);
p3 = plot(out1.time, F_max, '--r', 'LineWidth', 1.5);

ylabel('Force ($N$)','FontSize',15);xlabel('Time ($s$)','FontSize',15)
lgd = legend([p1,p2,p3],{'CD','CBF','Max Force'},'Location','best','FontSize',12);
% lgd.Units = 'normalized';       % Make position relative to axes
% pos1 = lgd.Position;
% pos1(1) = pos1(1) - 0;         % Move right
% pos1(2) = pos1(2) + 0;         % Move up
% lgd.Position = pos1;

hold off

hw_ratio = 1.1;

set(findall(force_plot,'-property','Interpreter'), 'Interpreter', 'latex')
set(findall(force_plot,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
set(findall(force_plot,'-property','Box'), 'Box', 'off')
set(force_plot, 'Units', 'centimeters', ...
        'Position', [2.3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);
grid on;

fname = 'fin_plots/Force_Comparison';
exportgraphics(force_plot, strcat(fname, '.png'), 'ContentType', 'vector');

%% comparison of q0 and q1 for CD vs CBF
traj_plot = figure;

subplot(2,1,1)
fill(out1.q0.fillX, out1.q0.fillY, [0 0.4470 0.7410], 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on
fill(out2.q0.fillX, out2.q0.fillY, [0.8500 0.3250 0.0980], 'FaceAlpha', 0.2, 'EdgeColor', 'none');

p_r1 = plot(out1.time_ref, out1.reference.q(:,1), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);

p1 = plot(out1.time, out1.q0.mu, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);
p2 = plot(out2.time, out2.q0.mu, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);

xlabel('Time (s)','FontSize',15)
ylabel('$q_0$ (radians)','FontSize',15)
lgd = legend([p1, p2, p_r1], {'CD','CBF','Reference'}, 'Location','best','FontSize',12);
grid on;
hold off

subplot(2,1,2)
fill(out1.q1.fillX, out1.q1.fillY, [0 0.4470 0.7410], 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on
fill(out2.q1.fillX, out2.q1.fillY, [0.8500 0.3250 0.0980], 'FaceAlpha', 0.2, 'EdgeColor', 'none');

p_r1 = plot(out1.time_ref, out1.reference.q(:,2), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);

p1 = plot(out1.time, out1.q1.mu, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);
p2 = plot(out2.time, out2.q1.mu, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);

xlabel('Time ($s$)','FontSize',15)
ylabel('$q_1$ (radians)','FontSize',15)
hold off

% lgd.Units = 'normalized';       % Make position relative to axes
% pos1 = lgd.Position;
% pos1(1) = pos1(1) - 0;         % Move right
% pos1(2) = pos1(2) + 0;         % Move up
% lgd.Position = pos1;

hw_ratio = 1.1;

set(findall(traj_plot,'-property','Interpreter'), 'Interpreter', 'latex')
set(findall(traj_plot,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
set(findall(traj_plot,'-property','Box'), 'Box', 'off')
set(traj_plot, 'Units', 'centimeters', ...
        'Position', [2.3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);
grid on;

fname = 'fin_plots/TrajectoryTracking_Comparison';
exportgraphics(traj_plot, strcat(fname, '.png'), 'ContentType', 'vector');

%% PD tuning results

pdtuning_plot = figure;

subplot(2,1,1)
fill(TunePD.q0.fillX, TunePD.q0.fillY, [0 0.4470 0.7410], 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on
p_r1 = plot(TunePD.time_ref, TunePD.reference.q(:,1), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
p1 = plot(TunePD.time, TunePD.q0.mu, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);

xlabel('Time (s)','FontSize',15)
ylabel('$q_0$ (radians)','FontSize',15)
lgd = legend([p1, p_r1], {'PD','Reference'}, 'Location','best','FontSize',12);
xlim([0 TunePD.time_ref(end)])
grid on;
hold off

subplot(2,1,2)
fill(TunePD.q1.fillX, TunePD.q1.fillY, [0 0.4470 0.7410], 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on
p_r1 = plot(TunePD.time_ref, TunePD.reference.q(:,2), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
p1 = plot(TunePD.time, TunePD.q1.mu, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);

xlabel('Time (s)','FontSize',15)
ylabel('$q_1$ (radians)','FontSize',15)
grid on;
hold off

hw_ratio = 1.1;

set(findall(pdtuning_plot,'-property','Interpreter'), 'Interpreter', 'latex')
set(findall(pdtuning_plot,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
set(findall(pdtuning_plot,'-property','Box'), 'Box', 'off')
set(pdtuning_plot, 'Units', 'centimeters', ...
        'Position', [2.3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);
xlim([0 TunePD.time_ref(end)])
grid on;

fname = 'fin_plots/PDTuning';
exportgraphics(pdtuning_plot, strcat(fname, '.png'), 'ContentType', 'vector');

%% PD tracking results

pdtracking_plot = figure;

subplot(2,1,1)
fill(CalibPD.q0.fillX, CalibPD.q0.fillY, [0 0.4470 0.7410], 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on
p_r1 = plot(CalibPD.time_ref, CalibPD.reference.q(:,1), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
p1 = plot(CalibPD.time, CalibPD.q0.mu, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);

xlabel('Time (s)','FontSize',15)
ylabel('$q_0$ (radians)','FontSize',15)
lgd = legend([p1, p_r1], {'PD','Reference'}, 'Location','best','FontSize',12);
xlim([0 CalibPD.time_ref(end)])
grid on;
hold off

subplot(2,1,2)
fill(CalibPD.q1.fillX, CalibPD.q1.fillY, [0 0.4470 0.7410], 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on
p_r1 = plot(CalibPD.time_ref, CalibPD.reference.q(:,2), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
p1 = plot(CalibPD.time, CalibPD.q1.mu, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);

xlabel('Time (s)','FontSize',15)
ylabel('$q_1$ (radians)','FontSize',15)
grid on;
hold off

hw_ratio = 1.1;

set(findall(pdtracking_plot,'-property','Interpreter'), 'Interpreter', 'latex')
set(findall(pdtracking_plot,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
set(findall(pdtracking_plot,'-property','Box'), 'Box', 'off')
set(pdtracking_plot, 'Units', 'centimeters', ...
        'Position', [2.3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);
xlim([0 CalibPD.time_ref(end)])
grid on;

fname = 'fin_plots/PDTracking';
exportgraphics(pdtracking_plot, strcat(fname, '.png'), 'ContentType', 'vector');

%% CD Tracking results 

cdtracking_plot = figure;

subplot(2,1,1)
fill(CalibCD.q0.fillX, CalibCD.q0.fillY, [0 0.4470 0.7410], 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on
p_r1 = plot(CalibCD.time_ref, CalibCD.reference.q(:,1), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
p1 = plot(CalibCD.time, CalibCD.q0.mu, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);

xlabel('Time (s)','FontSize',15)
ylabel('$q_0$ (radians)','FontSize',15)
lgd = legend([p1, p_r1], {'CD','Reference'}, 'Location','best','FontSize',12);
xlim([0 CalibCD.time_ref(end)])
grid on;
hold off

subplot(2,1,2)
fill(CalibCD.q1.fillX, CalibCD.q1.fillY, [0 0.4470 0.7410], 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on
p_r1 = plot(CalibCD.time_ref, CalibCD.reference.q(:,2), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
p1 = plot(CalibCD.time, CalibCD.q1.mu, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);

xlabel('Time (s)','FontSize',15)
ylabel('$q_1$ (radians)','FontSize',15)
grid on;
hold off

hw_ratio = 1.1;

set(findall(cdtracking_plot,'-property','Interpreter'), 'Interpreter', 'latex')
set(findall(cdtracking_plot,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
set(findall(cdtracking_plot,'-property','Box'), 'Box', 'off')
set(cdtracking_plot, 'Units', 'centimeters', ...
        'Position', [2.3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);
xlim([0 CalibCD.time_ref(end)])
grid on;

fname = 'fin_plots/CDTracking';
exportgraphics(cdtracking_plot, strcat(fname, '.png'), 'ContentType', 'vector');

%% CD tracking of TnR

cdtnr_plot = figure;

subplot(2,1,1)
fill(TnR.q0.fillX, TnR.q0.fillY, [0 0.4470 0.7410], 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on
p_r1 = plot(TnR.time_ref, TnR.reference.q(:,1), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
p1 = plot(TnR.time, TnR.q0.mu, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);

xlabel('Time (s)','FontSize',15)
ylabel('$q_0$ (radians)','FontSize',15)
lgd = legend([p1, p_r1], {'CD','Reference'}, 'Location','best','FontSize',12);
xlim([0 TnR.time_ref(end)])
grid on;
hold off

subplot(2,1,2)
fill(TnR.q1.fillX, TnR.q1.fillY, [0 0.4470 0.7410], 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on
p_r1 = plot(TnR.time_ref, TnR.reference.q(:,2), '-.', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
p1 = plot(TnR.time, TnR.q1.mu, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);

xlabel('Time (s)','FontSize',15)
ylabel('$q_1$ (radians)','FontSize',15)
grid on;
hold off

hw_ratio = 1.1;

set(findall(cdtnr_plot,'-property','Interpreter'), 'Interpreter', 'latex')
set(findall(cdtnr_plot,'-property','TickLabelInterpreter'), 'TickLabelInterpreter', 'latex')
set(findall(cdtnr_plot,'-property','Box'), 'Box', 'off')
set(cdtnr_plot, 'Units', 'centimeters', ...
        'Position', [2.3 1 picturewidth_singlecolumn hw_ratio * picturewidth_singlecolumn]);
xlim([0 TnR.time_ref(end)])
grid on;

fname = 'fin_plots/CDTnRTracking';
exportgraphics(cdtnr_plot, strcat(fname, '.png'), 'ContentType', 'vector');
