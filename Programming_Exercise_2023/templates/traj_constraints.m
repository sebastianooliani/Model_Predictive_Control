%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(x,u,params)
    % YOUR CODE HERE
    X_max = max(abs(x(1, :)));
    Z_max = max(abs(x(3, :)));
    s_max = max(X_max, Z_max);

    y_max = max(abs(x(2, :)));

    ux = max(abs(u(1, :)));
    uy = max(abs(u(2, :)));
    uz = max(abs(u(3, :)));
    u_max = max(max(ux, uy), uz);

    J_u = u(1,:) * u(1,:)' + u(2,:) * u(2,:)' + u(3,:) * u(3,:)';

    N = length(x(1,:));
    df_max = (x(1, N) ^ 2 + x(2, N) ^ 2 + x(3, N) ^ 2) ^ 0.5;
    vf_max = (x(4, N) ^ 2 + x(5, N) ^ 2 + x(6, N) ^ 2) ^ 0.5;
    
    if s_max <= params.constraints.MaxAbsPositionXZ && y_max <= params.constraints.MaxAbsPositionY && ...
            u_max <= params.constraints.MaxAbsThrust && df_max <= params.constraints.MaxFinalPosDiff && ...
            vf_max <= params.constraints.MaxFinalVelDiff
        traj_feas = true;
    else 
        traj_feas = false;
    end
end

