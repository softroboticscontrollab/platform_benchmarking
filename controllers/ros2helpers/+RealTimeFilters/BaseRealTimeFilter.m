classdef (Abstract) BaseRealTimeFilter < handle
    % BaseRealTimeFilter: Abstract base class for all real-time filters.
    % Ensures a consistent interface for hardware rollout and control loops.
    
    methods (Abstract)
        % Processes a single sample and returns smoothed value and its derivative
        [smoothed, derivative] = step(obj, new_sample)
        
        % Resets the internal buffers and initialization state
        reset(obj)
    end
end