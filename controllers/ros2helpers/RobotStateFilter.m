classdef RobotStateFilter < handle
    properties (Access = private)
        FilterObj
        VelFilterObj 
        numStates
    end
    
    methods
        function obj = RobotStateFilter(dt, sigma, halfWidthSigma, numStates)
            % Store number of states (default to 2 if not provided)
            if nargin < 4
                numStates = 2; 
            end

            obj.numStates = numStates;
            
            % Initialize the core Gaussian filter with your custom settings
            obj.FilterObj = RealTimeGaussFilter(dt, sigma, halfWidthSigma);
            % velocity filters (one per state)
            for i = 1:numStates
                obj.VelFilterObj{i} = RealTimeGaussFilter(dt, sigma, halfWidthSigma-2);
            end           
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
                [dq_filtered, ~] = obj.VelFilterObj{i}.step(ds);
                dq_now(i) = dq_filtered;
            end
        end
    end
end