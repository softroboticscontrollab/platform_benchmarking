function u = u_babblesine(t, constants)
%u_babblesine A controller for "motor babbling", calculates an open-loop
%signal of sinusoids. Assumes N=2 control inputs.
%
%   Inputs:
%       t == timestep. This is an open-loop controller, so we need to know
%       the time of the simulation.
%       constants == see below, various sine frequency and amplitude stuff

% pick out the constants
amp1 = constants.amp1;
amp2 = constants.amp2;
shift1 = constants.shift1;
shift2 = constants.shift2;
freq1 = constants.freq1;
freq2 = constants.freq2;

u = [amp1*sin(2*pi*freq1*t + shift1);
     amp2*sin(2*pi*freq2*t + shift2)];

end