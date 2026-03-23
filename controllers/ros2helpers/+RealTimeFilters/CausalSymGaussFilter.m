classdef CausalSymGaussFilter < RealTimeFilters.BaseRealTimeFilter
    % CausalSymGaussFilter: Standard symmetric Gaussian filter.
    % High fidelity linear phase but introduces fixed group delay.
    
    properties (Access = private)
        buffer, kernel_w, kernel_wp, halfWidth
        is_initialized = false
    end
    
    methods
        function obj = CausalSymGaussFilter(dt, sigma, halfWidthSigma)
            if nargin < 3 || isempty(halfWidthSigma), halfWidthSigma = 3; end
            obj.halfWidth = max(1, ceil(halfWidthSigma * sigma / dt));
            windowSize = 2 * obj.halfWidth + 1;
            t = (-obj.halfWidth:obj.halfWidth)' * dt; % Peak at the center
            
            w = exp(-(t.^2) / (2*sigma^2));
            obj.kernel_w = w / sum(w);
            wp = -(t / (sigma^2)) .* obj.kernel_w;
            obj.kernel_wp = wp - mean(wp);
            obj.buffer = zeros(windowSize, 1);
        end
        
        function [s, d] = step(obj, new_sample)
            obj.buffer = [obj.buffer(2:end); new_sample];
            if ~obj.is_initialized
                obj.buffer(:) = new_sample; obj.is_initialized = true;
            end
            s = obj.kernel_w' * obj.buffer;
            d = obj.kernel_wp' * obj.buffer;
        end
        
        function reset(obj)
            obj.buffer(:) = 0; obj.is_initialized = false;
        end
    end
end