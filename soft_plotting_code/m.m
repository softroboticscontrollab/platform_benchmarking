function [xi] = m(q, L)
%m mapping between the two configurations as explained in the Della Santina
% et al 2020 paper, where 
%         m : R^n --> R^{nh} Eq(3)

% Implemented by Charlie DeLorey
% The map m is such that the nonlinear constraint xi = m(q) assures that
% the end points and the point masses of each CC segment coincide in
% position and orientation with the corresponding points of the rigid
% robot. 
% The definition of xi below corresponds to Eqn (7) from the paper, and
% corresponds to the DH table they present in the paper as well (Table 1). 
% Input::
%    q : angle of bending segment
%    L : length of bending segment

% Returns::
%    xi : matrix mapping of rigid robot to CC configuration (in terms of q)

xi = [q/2;
     (L*(sin(q/2)/q));
     (L*(sin(q/2)/q));
      q/2];                % m_i(q_i) mapping, Eqn (4) in the paper
  
end

