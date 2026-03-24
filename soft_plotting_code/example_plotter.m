%% Example Code for plotting of data 
clear all; close all; clc;

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

% final tuning response for chamber 1
dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC1.csv'; 
ezloopdata_compare(dat, false, true);

% final tuning resposne for chamber 2
dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC2.csv'; 
ezloopdata_compare(dat, false, true);

% final tuning resposne for chamber 3
dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC3.csv'; 
ezloopdata_compare(dat, false, true);

% final tuning resposne for chamber 4
dat = 'tuning/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-16_tuningC4.csv'; 
ezloopdata_compare(dat, false, true);

% Open-loop motion for dynamics calibration with amplitude of 100 and period of 200
dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-19_141950.csv'; 
[time_cal, q_cal, dq_cal, ddq_cal, smooth_q_cal, smooth_dq_cal, smooth_ddq_cal, calib_start,~] = ezloopdata_compare(dat, false, false, true, 0);

dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-24_TrackCalib1.csv'; 
[time_tra1, q_tra1, dq_tra1, ddq_tra1, smooth_q_tra1, smooth_dq_tra1, smooth_ddq_tra1,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-24_TrackCalib2.csv'; 
[time_tra2, q_tra2, dq_tra2, ddq_tra2, smooth_q_tra2, smooth_dq_tra2, smooth_ddq_tra2,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-24_TrackCalib3.csv'; 
[time_tra3, q_tra3, dq_tra3, ddq_tra3, smooth_q_tra3, smooth_dq_tra3, smooth_ddq_tra3,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-24_TrackCalib4.csv'; 
[time_tra4, q_tra4, dq_tra4, ddq_tra4, smooth_q_tra4, smooth_dq_tra4, smooth_ddq_tra4,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-24_TrackCalib5.csv'; 
[time_tra5, q_tra5, dq_tra5, ddq_tra5, smooth_q_tra5, smooth_dq_tra5, smooth_ddq_tra5,~,~] = ezloopdata_compare(dat, false, false, true, calib_start);

smooth_q_cells = {smooth_q_tra1, smooth_q_tra2, smooth_q_tra3, ...
                  smooth_q_tra4, smooth_q_tra5};

time_cells = {time_tra1, time_tra2, time_tra3, time_tra4, time_tra5};

ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_cal, smooth_q_cal, false)

% AsymGaussian Filter Results 
dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-19_122338.csv';
[time_asym, q_asym, dq_asym, ddq_asym, smooth_q_asym, smooth_dq_asym, smooth_ddq_asym, ~, ~] = ezloopdata_compare(dat, false, false, true, 0);

% Butterworth Filter Results
dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-19_123309.csv';
[time_butter, q_butter, dq_butter, ddq_butter, smooth_q_butter, smooth_dq_butter, smooth_ddq_butter, ~, ~] = ezloopdata_compare(dat, false, false, true, 0);

%% Dynamics Controller response to Teach-and-Repeat 2

% Teach-and-Repeat 2 trajectory
dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-19_TnR2.csv'; 
[time_TnR2, q_TnR2, dq_TnR2, ddq_TnR2, smooth_q_TnR2, smooth_dq_TnR2, smooth_ddq_TnR2, index_TnR2, ~] = ezloopdata_compare(dat, false, false, false, 0);

% Inverse Dynamics Controller on TnR2 w/o forceplate
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-20_TnR2CD1.csv'; 
[time_CD1, q_CD1, dq_CD1, ddq_CD1, smooth_q_CD1, smooth_dq_CD1, smooth_ddq_CD1, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR2);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-20_TnR2CD2.csv'; 
[time_CD2, q_CD2, dq_CD2, ddq_CD2, smooth_q_CD2, smooth_dq_CD2, smooth_ddq_CD2, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR2);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-20_TnR2CD3.csv'; 
[time_CD3, q_CD3, dq_CD3, ddq_CD3, smooth_q_CD3, smooth_dq_CD3, smooth_ddq_CD3, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR2);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-20_TnR2CD4.csv'; 
[time_CD4, q_CD4, dq_CD4, ddq_CD4, smooth_q_CD4, smooth_dq_CD4, smooth_ddq_CD4, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR2);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-20_TnR2CD5.csv'; 
[time_CD5, q_CD5, dq_CD5, ddq_CD5, smooth_q_CD5, smooth_dq_CD5, smooth_ddq_CD5, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR2);

smooth_q_cells = {smooth_q_CD1, smooth_q_CD2, smooth_q_CD3, ...
                  smooth_q_CD4, smooth_q_CD5};

time_cells = {time_CD1, time_CD2, time_CD3, time_CD4, time_CD5};

ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR2, smooth_q_TnR2, false)

% Inverse Dynamics Controller on TnR2 w/ forceplate
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_CDFP1.csv'; 
[time_CDFP1, q_CDFP1, dq_CDFP1, ddq_CDFP1, smooth_q_CDFP1, smooth_dq_CDFP1, smooth_ddq_CDFP1, ~, rho0_CDFP1] = ezloopdata_compare(dat, false, false, false, index_TnR2);
 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_CDFP2.csv'; 
[time_CDFP2, q_CDFP2, dq_CDFP2, ddq_CDFP2, smooth_q_CDFP2, smooth_dq_CDFP2, smooth_ddq_CDFP2, ~, rho0_CDFP2] = ezloopdata_compare(dat, false, false, false, index_TnR2);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_CDFP3.csv'; 
[time_CDFP3, q_CDFP3, dq_CDFP3, ddq_CDFP3, smooth_q_CDFP3, smooth_dq_CDFP3, smooth_ddq_CDFP3, ~, rho0_CDFP3] = ezloopdata_compare(dat, false, false, false, index_TnR2);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_CDFP4.csv'; 
[time_CDFP4, q_CDFP4, dq_CDFP4, ddq_CDFP4, smooth_q_CDFP4, smooth_dq_CDFP4, smooth_ddq_CDFP4, ~, rho0_CDFP4] = ezloopdata_compare(dat, false, false, false, index_TnR2);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_CDFP5.csv'; 
[time_CDFP5, q_CDFP5, dq_CDFP5, ddq_CDFP5, smooth_q_CDFP5, smooth_dq_CDFP5, smooth_ddq_CDFP5, ~, rho0_CDFP5] = ezloopdata_compare(dat, false, false, false, index_TnR2);

smooth_q_cells = {smooth_q_CDFP1, smooth_q_CDFP2, smooth_q_CDFP3, ...
                  smooth_q_CDFP4, smooth_q_CDFP5};

rho_cells = {rho0_CDFP1, rho0_CDFP2, rho0_CDFP3, rho0_CDFP4, rho0_CDFP5};

time_cells = {time_CDFP1, time_CDFP2, time_CDFP3, time_CDFP4, time_CDFP5};

ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR2, smooth_q_TnR2, true, rho_cells)

%% Dynamics Controller response to Teach-and-Repeat 3

% Teach-and-Repeat 3 trajectory
dat = 'plotting_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_TnR3.csv'; 
[time_TnR3, q_TnR3, dq_TnR3, ddq_TnR3, smooth_q_TnR3,smooth_dq_TnR3,smooth_ddq_TnR3, index_TnR3, ~] = ezloopdata_compare(dat, false, false, false, 0);

% Inverse Dynamics Controller on TnR2 w/o forceplate
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_TnR3CD1.csv'; 
[time_CD1, q_CD1, dq_CD1, ddq_CD1, smooth_q_CD1, smooth_dq_CD1, smooth_ddq_CD1, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR3);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_TnR3CD2.csv'; 
[time_CD2, q_CD2, dq_CD2, ddq_CD2, smooth_q_CD2, smooth_dq_CD2, smooth_ddq_CD2, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR3);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_TnR3CD3.csv'; 
[time_CD3, q_CD3, dq_CD3, ddq_CD3, smooth_q_CD3, smooth_dq_CD3, smooth_ddq_CD3, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR3);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_TnR3CD4.csv'; 
[time_CD4, q_CD4, dq_CD4, ddq_CD4, smooth_q_CD4, smooth_dq_CD4, smooth_ddq_CD4, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR3);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_TnR3CD5.csv'; 
[time_CD5, q_CD5, dq_CD5, ddq_CD5, smooth_q_CD5, smooth_dq_CD5, smooth_ddq_CD5, ~, ~] = ezloopdata_compare(dat, false, false, false, index_TnR3);

smooth_q_cells = {smooth_q_CD1, smooth_q_CD2, smooth_q_CD3, ...
                  smooth_q_CD4, smooth_q_CD5};

time_cells = {time_CD1, time_CD2, time_CD3, time_CD4, time_CD5};

ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR3, smooth_q_TnR3, false)

% Inverse Dynamics Controller on TnR3 w/ forceplate
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_CD2FP1.csv'; 
[time_CDFP1, q_CDFP1, dq_CDFP1, ddq_CDFP1, smooth_q_CDFP1, smooth_dq_CDFP1, smooth_ddq_CDFP1, ~, rho0_CDFP1] = ezloopdata_compare(dat, false, false, false, index_TnR3);
 
dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_CD2FP2.csv'; 
[time_CDFP2, q_CDFP2, dq_CDFP2, ddq_CDFP2, smooth_q_CDFP2, smooth_dq_CDFP2, smooth_ddq_CDFP2, ~, rho0_CDFP2] = ezloopdata_compare(dat, false, false, false, index_TnR3);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_CD2FP3.csv'; 
[time_CDFP3, q_CDFP3, dq_CDFP3, ddq_CDFP3, smooth_q_CDFP3, smooth_dq_CDFP3, smooth_ddq_CDFP3, ~, rho0_CDFP3] = ezloopdata_compare(dat, false, false, false, index_TnR3);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_CD2FP4.csv'; 
[time_CDFP4, q_CDFP4, dq_CDFP4, ddq_CDFP4, smooth_q_CDFP4, smooth_dq_CDFP4, smooth_ddq_CDFP4, ~, rho0_CDFP4] = ezloopdata_compare(dat, false, false, false, index_TnR3);

dat = 'manuscript_data/ezloophw_closedloop_ros2_reserv_pneumatics_2026-3-23_CD2FP5.csv'; 
[time_CDFP5, q_CDFP5, dq_CDFP5, ddq_CDFP5, smooth_q_CDFP5, smooth_dq_CDFP5, smooth_ddq_CDFP5, ~, rho0_CDFP5] = ezloopdata_compare(dat, false, false, false, index_TnR3);

smooth_q_cells = {smooth_q_CDFP1, smooth_q_CDFP2, smooth_q_CDFP3, ...
                  smooth_q_CDFP4, smooth_q_CDFP5};

rho_cells = {rho0_CDFP1, rho0_CDFP2, rho0_CDFP3, rho0_CDFP4, rho0_CDFP5};

time_cells = {time_CDFP1, time_CDFP2, time_CDFP3, time_CDFP4, time_CDFP5};

ezloopdata_safetyplotter(smooth_q_cells, time_cells, time_TnR3, smooth_q_TnR3, true, rho_cells)
