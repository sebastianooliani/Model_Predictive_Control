%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ac, Bc] = generate_system_cont(params)
    % YOUR CODE HERE
    wn = (params.model.GravitationalParameter / (params.model.TargetRadius) ^ 3) ^ 0.5;
    m = params.model.Mass;
    Ac = [0 0 0 1 0 0;
          0 0 0 0 1 0;
          0 0 0 0 0 1;
          3*wn^2 0 0 0 2*wn 0;
          0 0 0 -2*wn 0 0;
          0 0 -wn^2 0 0 0];
    Bc = [zeros(params.model.nu);
         1/m 0 0;
         0 1/m 0;
         0 0 1/m];
end