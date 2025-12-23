data1 = readtable('ezloophw_closedloop_ros2_pneumatics_2025-5-1_152443.csv', ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');

data2 = readtable('ezloophw_closedloop_ros2_pneumatics_2025-7-18_165857 2', ...
    'HeaderLines', 2, 'VariableNamingRule', 'preserve');

time_data1 = data1.("Test time");
theta_01  = data1.theta_0;
theta_11  = data1.theta_1;
mpr_01    = data1.MPRpressure_0;
mpr_11    = data1.MPRpressure_1;
mpr_21    = data1.MPRpressure_2;
mpr_31    = data1.MPRpressure_3;

time_data2 = data2.("Test time");
theta_02  = data2.theta_0;
theta_12  = data2.theta_1;
mpr_02    = data2.MPRpressure_0;
mpr_12    = data2.MPRpressure_1;
mpr_22    = data2.MPRpressure_2;
mpr_32    = data2.MPRpressure_3;

% Convert angles from degrees to radians
theta_01 = deg2rad(2*theta_01);
theta_11 = deg2rad(2*theta_11);

% Convert angles from degrees to radians
theta_02 = deg2rad(2*theta_02);
theta_12 = deg2rad(2*theta_12);

smoothed_theta_01 = theta_01;
smoothed_theta_11 = theta_11;
smoothed_mpr_01 = smoothdata(mpr_01, 'sgolay', 40);
smoothed_mpr_11 = smoothdata(mpr_11, 'sgolay', 40);
smoothed_mpr_21 = smoothdata(mpr_21, 'sgolay', 40);
smoothed_mpr_31 = smoothdata(mpr_31, 'sgolay', 40);

smoothed_theta_02 = theta_02;
smoothed_theta_12 = theta_12;
smoothed_mpr_02 = smoothdata(mpr_02, 'sgolay', 40);
smoothed_mpr_12 = smoothdata(mpr_12, 'sgolay', 40);
smoothed_mpr_22 = smoothdata(mpr_22, 'sgolay', 40);
smoothed_mpr_32 = smoothdata(mpr_32, 'sgolay', 40);

u_offset1 = [1149.0, 1135.0, 1139.0, 1152.0];
u_offset2 = [1155.0 1135.0 1140.0 1155.0];

p11 = smoothed_mpr_01 - u_offset1(1) - (smoothed_mpr_11 - u_offset1(2));
p21 = smoothed_mpr_21 - u_offset1(3) - (smoothed_mpr_31 - u_offset1(4));
p_raw1 = [p11, p21]';

p12 = smoothed_mpr_02 - u_offset2(1) - (smoothed_mpr_12 - u_offset2(2));
p22 = smoothed_mpr_22 - u_offset2(3) - (smoothed_mpr_32 - u_offset2(4));
p_raw2 = [p12, p22]';

figure(1);

subplot(4,2,1); plot(time_data1, p11); title('pressure in limb 1 April');
subplot(4,2,2); plot(time_data2, p12); title('pressure in limb 1 July');
subplot(4,2,3); plot(time_data1, p21); title('pressure in limb 2 April');
subplot(4,2,4); plot(time_data2, p22); title('pressure in limb 2 July');
