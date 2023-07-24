%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_u, h_u, H_x, h_x] = generate_constraints(params)
    % YOUR CODE HERE
    H_u = [eye(params.model.nu);
        - eye(params.model.nu)];
    h_u = ones(params.model.nx, 1) * params.constraints.MaxAbsThrust;
    H_x = [eye(params.model.nu) zeros(params.model.nu);
        - eye(params.model.nu) zeros(params.model.nu)];
    h_x = [params.constraints.MaxAbsPositionXZ; params.constraints.MaxAbsPositionY; ...
        params.constraints.MaxAbsPositionXZ; params.constraints.MaxAbsPositionXZ; ...
        params.constraints.MaxAbsPositionY; params.constraints.MaxAbsPositionXZ];
end