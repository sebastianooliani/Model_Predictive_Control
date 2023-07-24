%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, B] = generate_system(Ac, Bc, params)
    % YOUR CODE HERE
    Cc = [eye(params.model.nu), zeros(params.model.nu)];
    Dc = zeros(params.model.nu, params.model.nu);
    sys = ss(Ac, Bc, Cc, Dc);
    sysd = c2d(sys, params.model.TimeStep);
    A = sysd.A;
    B = sysd.B;
end