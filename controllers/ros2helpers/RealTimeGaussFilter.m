classdef RealTimeGaussFilter < handle
    % RealTimeGaussFilter: Causal Gaussian smoothing and derivative.
    % Note: This introduces a physical delay of (halfWidth * dt) seconds.
    
    properties (Access = private)
        buffer          % Stores the sliding window of samples
        kernel_w        % Smoothing kernel
        kernel_wp       % Derivative kernel
        halfWidth       % Number of samples in the window
        is_initialized = false
    end
    
    methods
        function obj = RealTimeGaussFilter(dt, sigma, halfWidthSigma)
            if nargin < 3 || isempty(halfWidthSigma), halfWidthSigma = 3; end
            
            % Calculate kernel size
            obj.halfWidth = max(1, ceil(halfWidthSigma * sigma / dt));
            % Total window size is 2*halfWidth + 1
            windowSize = 2 * obj.halfWidth + 1;
            
            % Create time vector for the window
            t = (-obj.halfWidth:obj.halfWidth)' * dt;
            
            % --- Gaussian smoothing kernel ---
            w = exp(-(t.^2) / (2*sigma^2));
            obj.kernel_w = w / sum(w);
            
            % --- Derivative kernel ---
            wp = -(t / (sigma^2)) .* obj.kernel_w;
            obj.kernel_wp = wp - mean(wp); % Enforce zero-sum
            
            % Initialize buffer with zeros
            obj.buffer = zeros(windowSize, 1);
        end
        
        function [x_smooth, dxdt] = step(obj, new_sample)
            % Update buffer: Slide existing samples and add the newest one
            obj.buffer = [obj.buffer(2:end); new_sample];
            
            % If this is the very first sample, fill buffer to avoid ramp-up
            if ~obj.is_initialized
                obj.buffer(:) = new_sample;
                obj.is_initialized = true;
            end
            
            % Perform Dot Product (equivalent to one step of convolution)
            % The result represents the smoothed value at the CENTER of the buffer.
            x_smooth = obj.kernel_w' * obj.buffer;
            dxdt     = obj.kernel_wp' * obj.buffer;
        end
        
        function reset(obj)
            obj.buffer(:) = 0;
            obj.is_initialized = false;
        end
    end
end