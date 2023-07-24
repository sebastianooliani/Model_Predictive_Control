%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [tuning_struct, i_opt] = lqr_tuning(x0,Q,params)
    % YOUR CODE HERE
    tuning_struct = struct([]);
    J_opt = Inf;
    i_opt = nan;
    for i = 1:1:length(Q(1, :))
        q_i = Q(:, i);
        Q_i = diag(q_i);
        ctrl = LQR(Q_i, eye(params.model.nu), params);
        [Xt,Ut,~] = simulate(x0, ctrl, params);
        [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(Xt,Ut,params);

        % define the structure
        i_struct.InitialCondition = x0;
        i_struct.Qdiag = q_i;
        i_struct.MaxAbsPositionXZ = s_max;
        i_struct.MaxAbsPositionY = y_max;
        i_struct.MaxAbsThrust = u_max;
        i_struct.InputCost = J_u;
        i_struct.MaxFinalPosDiff = df_max;
        i_struct.MaxFinalVelDiff = vf_max;
        i_struct.TrajFeasible = traj_feas;
        tuning_struct = [tuning_struct; i_struct];

        % compute, if exists, i_opt
        if traj_feas
            if J_u < J_opt
                i_opt = i;
                J_opt = J_u;
            end
        end
    end
end