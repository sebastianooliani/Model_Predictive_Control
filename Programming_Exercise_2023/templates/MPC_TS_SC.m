%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS_SC
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS_SC(Q,R,N,H,h,S,v,params)            
            % YOUR CODE HERE
            nu = params.model.nu;
            nx = params.model.nx;

            A = params.model.A;
            B = params.model.B;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;

            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            E = sdpvar(repmat(length(h_x),1,N), ones(1,N), 'full');
            X = sdpvar(repmat(nx,1,N+1), ones(1,N+1), 'full');
            X0 = X{1};

            % Compute P (J_inf)
            [P,~,~] = idare(A,B,Q,R,[],eye(nx));

            % Generate objective and constraints for optimization problem.
            objective = 0;
            constraints = [];
            for k = 1:N
                x_k = X{k};
                x_kp1 = X{k+1};
                u_k=U{k};
                e_k = E{k}; 
                objective = objective + x_k'*Q*x_k + u_k'*R*u_k...
                    + e_k'*S*e_k + v*norm(e_k, Inf);
                constraints = [constraints, e_k >= 0,x_kp1 == A*x_k + B*u_k,...
                    H_u*u_k<=h_u, H_x*x_k <= h_x + e_k];       
            end
            % Add terminal cost
            e_k = E{end}; x_k = X{end};
            objective = objective + x_k'*P*x_k...
                + e_k'*S*e_k + v*norm(e_k, Inf);
            constraints = [constraints, e_k >= 0, H_x*x_k <= h_x + e_k, H*x_k <= h];

            % Can use yalmip optimize to debug
            %diagnostics = optimize(constraints, objective);
            %disp(diagnostics);

            opts = sdpsettings('verbose',1,'solver','quadprog','quadprog.TolFun',1e-8);
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