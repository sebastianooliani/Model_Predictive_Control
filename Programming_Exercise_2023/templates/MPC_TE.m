%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TE
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TE(Q,R,N,params)
            % YOUR CODE HERE
            nu = params.model.nu;
            nx = params.model.nx;

            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X0 = sdpvar(nx,1,'full');

            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;
            A = params.model.A;
            B = params.model.B;
            constraints = [];
            objective = 0;

            x_i = X0;
            for i = 1:1:N
                u_i = U{:, i};
                objective = objective + x_i' * Q * x_i + u_i' * R * u_i;
                constraints = [constraints, H_x * x_i <= h_x, H_u * u_i <= h_u];
                x_i = A * x_i + B * u_i;
            end
            [P, ~, ~] = idare(A, B,Q,R, [], eye(nx));
            objective = objective + x_i' * P * x_i;
            constraints = [constraints, x_i == zeros(nx, 1)];
            
            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{U{1} objective});
        end

        function [u, ctrl_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            ctrl_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end