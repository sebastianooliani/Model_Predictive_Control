%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Wt = generate_disturbances(params)
    % YOUR CODE HERE
    H_w = params.constraints.DisturbanceMatrix;
    h_w = params.constraints.DisturbanceRHS;
    nx = params.model.nx;
    nu = params.model.nu;
    Nt = params.model.HorizonLength;
    Wt = zeros(nx, Nt);
    
    P = Polyhedron('A', H_w, 'b', h_w);
    for i = 1:1:Nt
        Wt(:, i) = P.randomPoint();
    end

end