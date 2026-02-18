function B = barrierB_hsu2015(h, hdot, aE, bE)
%barrierB_hsu2015 Implementation of the control barrier function B(h(x))
%per the Hsu2015 paper and the Rauscher2016 paper, for relative degree two
%systems.
%
%   Inputs:
%       h == sym, a scalar. We assume this is a function of time, i.e.,
%       it's an h(x) where dh/dt = (\partial h/\partial t) \dot x
%       hdot = \dot h(x). It's the caller's responsibility to calculate
%       this sym appropriately, best not to do it here.
%       aE = tuning constant in R+
%       bE = tuning constant in R+

B = -log(h/(1+h)) + aE*((bE*hdot^2)/(1 + bE*hdot^2));

end