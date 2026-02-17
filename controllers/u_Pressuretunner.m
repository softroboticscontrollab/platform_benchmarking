function u = u_Pressuretunner(~,~,t)
% Square-wave switching:
% u = 0 for first half-period
% u = 100 for second half-period

T = 20;

phase = mod(t, T);

if phase < T/2
    u = [0;0];
else
    u = [0;-100];
end
end
