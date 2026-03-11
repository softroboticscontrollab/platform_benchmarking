classdef RobotStateFilter < handle
    properties (Access = private)
        FilterObj
        numStates
    end
    
    methods
        function obj = RobotStateFilter(dt, sigma, halfWidthSigma, numStates)
            % Store number of states (default to 2 if not provided)
            if nargin < 4, numStates = 2; end
            obj.numStates = numStates;
            
            % Initialize the core Gaussian filter with your custom settings
            %
            obj.FilterObj = RealTimeGaussFilter(dt, sigma, halfWidthSigma);
        end
        
        function [q_now, dq_now] = process(obj, dataWindow)
            % dataWindow: 10x2 matrix (Rows = time, Cols = states)
            % Returns: 1x2 current q and 1x2 current dq
            
            q_now = zeros(1, obj.numStates);
            dq_now = zeros(1, obj.numStates);
            
            for i = 1:obj.numStates
                % Reset buffer to ensure 10-step window is processed fresh
                obj.FilterObj.reset();
                
                % Extract column for the specific angle
                column = dataWindow(:, i);
                
                % Process the window; the final 'step' returns the most recent result
                for j = 1:length(column)
                    [s, ds] = obj.FilterObj.step(column(j)); %
                end
                
                q_now(i) = s;
                dq_now(i) = ds;
            end
        end
    end
end