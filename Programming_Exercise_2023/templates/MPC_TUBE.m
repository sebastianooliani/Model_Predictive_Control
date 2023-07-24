%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TUBE
    properties
        yalmip_optimizer
        K_tube
    end

    methods
        function obj = MPC_TUBE(Q,R,N,H_N,h_N,H_tube,h_tube,K_tube,params)
            obj.K_tube = K_tube;

            % YOUR CODE HERE
            % Note that constraints in params is already tightened!!!
            nu = params.model.nu;
            nx = params.model.nx;

            A = params.model.A;
            B = params.model.B;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;

            % define optimization variables
            X = sdpvar(repmat(nx,1,N+1), ones(1,N+1), 'full');
            X0 = X{1};
            V = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            Z = sdpvar(repmat(nx,1,N+1), ones(1,N+1), 'full');

            % Compute P (J_inf) for terminal cost
            [P,~,~] = idare(A,B,Q,R,[],eye(nx));

            % Generate objective and constraints for optimization problem.
            % Use Z and V, except for initial condition on X0
            z_k = Z{1};
            objective = 0;
            constraints = [H_tube*X0 <= H_tube*z_k + h_tube];
            for k = 1:N
                v_k = V{k}; z_k = Z{k}; z_kp1 = Z{k+1};
                objective = objective + z_k'*Q*z_k + v_k'*R*v_k;
                constraints = [constraints, H_x*z_k <= h_x, H_u*v_k<=h_u,...
                    z_kp1 == A*z_k + B*v_k];%, u_k == K_tube(x_k - z_k)+v_k];
            end
            % Add terminal cost
            z_k = z_kp1;
            objective = objective + z_k'*P*z_k;

            % Polytopic terminal set constraint
            constraints = [constraints, H_N*z_k <= h_N];

            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{V{1} Z{1} objective});
        end

        function [u, ctrl_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            % YOUR CODE HERE
            % Define u as K_tube(xk-zk) + vk
            objective = optimizer_out{3};
            u = obj.K_tube*(x - optimizer_out{2}) + optimizer_out{1};
            
            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            ctrl_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end