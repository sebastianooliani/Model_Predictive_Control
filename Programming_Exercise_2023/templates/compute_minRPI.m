%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_tube,h_tube,n_iter] = compute_minRPI(K_tube,params)
    % YOUR CODE HERE
    A = params.model.A;
    B = params.model.B;
    H_w = params.constraints.DisturbanceMatrix;
    h_w = params.constraints.DisturbanceRHS;
    N = params.model.HorizonLength;
    
    W = Polyhedron(H_w, h_w);
    A_c = A + B * K_tube;
    % init first element
    E = W;
    E_cell{1} = E;
    % approximate mRPI set
    for i=1:1:N
        E = E + A_c^i * W;
        E_cell{i+1} = E; % for plotting each iteration
        E_cell{i+1} = E_cell{i+1}.minHRep();
        E_cell{i} = E_cell{i}.minHRep();
        if E_cell{i+1} == E_cell{i}
            n_iter = i;
            break
        end
    end
    
    E = E.minHRep();

    H_tube = E.A;
    h_tube = E.b;
end