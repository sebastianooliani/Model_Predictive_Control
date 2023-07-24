%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef LQR
    properties
        K
    end
    
    methods
        %constructor
        function obj = LQR(Q, R, params)
            % YOUR CODE HERE
            % obj.K = ... (save feedback matrix for use in eval function)
            A = params.model.A;
            B = params.model.B;
            [obj.K, ~, ~] = dlqr(A, B, Q, R); 
        end
        
        function [u, ctrl_info] = eval(obj, x)
            % YOUR CODE HERE
            % u = ...
            u = - obj.K * x;
            ctrl_info = struct('ctrl_feas',true);
        end
    end
end