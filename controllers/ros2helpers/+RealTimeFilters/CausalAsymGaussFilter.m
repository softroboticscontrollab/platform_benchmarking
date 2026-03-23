classdef CausalAsymGaussFilter < RealTimeFilters.BaseRealTimeFilter
    % CausalAsymGaussFilter: Asymmetric Gaussian filter with corrected derivative scaling.
    
    properties (Access = private)
        buffer, kernel_w, kernel_wp, windowSize
        is_initialized = false
    end
    
    methods
        function obj = CausalAsymGaussFilter(dt, sigma, halfWidthSigma)
            if nargin < 3 || isempty(halfWidthSigma), halfWidthSigma = 3; end
            
            % 1. Determine window size based on physics parameters
            obj.windowSize = max(2, ceil(halfWidthSigma * sigma / dt));
            t = (-(obj.windowSize-1):0)' * dt; % Time vector ending at 0
            
            % 2. Generate Smoothing Kernel (w)
            w = exp(-(t.^2) / (2*sigma^2));
            obj.kernel_w = w / sum(w); % Ensure unit gain
            
            % 3. Generate Corrected Derivative Kernel (wp)
            % Initial shape calculation
            wp_raw = -(t / (sigma^2)) .* obj.kernel_w;
            
            % Remove DC bias to ensure zero output for constant input
            wp_centered = wp_raw - mean(wp_raw);
            
            % --- The Correction Factor ---
            % We need to ensure that when we dot product with a ramp (q = t), 
            % the result is exactly 1. This handles the 1/dt scaling automatically.
            slope_gain = wp_centered' * t; 
            obj.kernel_wp = wp_centered / slope_gain; 
            
            % 4. Initialize Buffer
            obj.buffer = zeros(obj.windowSize, 1);
        end
        
        function [s, d] = step(obj, new_sample)
            % Keep the step function clean and fast
            obj.buffer = [obj.buffer(2:end); new_sample];
            
            if ~obj.is_initialized
                obj.buffer(:) = new_sample; 
                obj.is_initialized = true;
            end
            
            % Efficient matrix multiplication
            % Using (:) ensures it works even if input orientation changes
            s = obj.kernel_w(:)' * obj.buffer(:);
            d = obj.kernel_wp(:)' * obj.buffer(:);
        end
        
        function reset(obj)
            obj.buffer(:) = 0; 
            obj.is_initialized = false;
        end
    end
end