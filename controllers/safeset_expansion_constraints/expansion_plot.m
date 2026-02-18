clear all;
close all;
clc;

% V = [0, 0.8; 
%     1, -1;
%      1.8, 1.2];

% V = [-0.02, 0.16; 
%       0.15, -0.175;
%      0.45, 0.19];

V = [0.35, -0.005;
    0, 0.122389;
    -0.2, -0.2;
    0.2, -0.2];

F_max = 0.2232;
k_env = 11.16;

[H, h] = computeSafetyConstraints(V);
h(1,:) = 0.15;
[H_prime, h_prime] = generate_safeset_expansion(H, h, F_max, k_env);

figure; hold on;
grid on;

title('Robot arm double pendulum dynamics simulation')

P1 = Polyhedron(H, h);
P2 = Polyhedron(H, h_prime);

plot(P2, 'color', 'm');
plot(P1, 'color', 'c');
