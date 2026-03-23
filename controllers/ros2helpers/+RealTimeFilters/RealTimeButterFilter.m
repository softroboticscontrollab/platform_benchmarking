classdef RealTimeButterFilter < RealTimeFilters.BaseRealTimeFilter
    % RealTimeButterFilter: IIR Butterworth low-pass filter (2nd order).
    % Extremely efficient for real-time control with low computational cost.
    
    properties (Access = private)
        b, a            % Filter coefficients
        x_prev, y_prev  % State history for IIR difference equation
        dt              % Sampling time
        is_initialized = false
    end
    
    methods
        function obj = RealTimeButterFilter(dt, fc)
            % fc: Cutoff frequency in Hz
            obj.dt = dt;
            fs = 1/dt;
            [obj.b, obj.a] = butter(2, fc/(fs/2)); % 2nd order low-pass
            obj.x_prev = zeros(2, 1);
            obj.y_prev = zeros(2, 1);
        end
        
        function [s, d] = step(obj, new_sample)
            if ~obj.is_initialized
                obj.x_prev(:) = new_sample;
                obj.y_prev(:) = new_sample;
                obj.is_initialized = true;
            end
            
            % Difference equation: a(1)y[n] = b(1)x[n] + b(2)x[n-1] + ...
            s = (obj.b(1)*new_sample + obj.b(2)*obj.x_prev(1) + obj.b(3)*obj.x_prev(2) ...
                 - obj.a(2)*obj.y_prev(1) - obj.a(3)*obj.y_prev(2)) / obj.a(1);
            
            % Numerical derivative (Backward Difference)
            d = (s - obj.y_prev(1)) / obj.dt;
            
            % Update states
            obj.x_prev = [new_sample; obj.x_prev(1)];
            obj.y_prev = [s; obj.y_prev(1)];
        end
        
        function reset(obj)
            obj.x_prev(:) = 0; obj.y_prev(:) = 0;
            obj.is_initialized = false;
        end
    end
end