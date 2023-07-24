%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params = compute_tightening(K_tube,H_tube,h_tube,params)  
	% YOUR CODE HERE
    H_x = params.constraints.StateMatrix;
    h_x = params.constraints.StateRHS;
    State = Polyhedron(H_x, h_x);
    H_u = params.constraints.InputMatrix;
    h_u = params.constraints.InputRHS;
    Input = Polyhedron(H_u, h_u);
    
    Tube = Polyhedron(H_tube, h_tube);

    % difference
    State_const = State - Tube;
    Input_const = Input - K_tube * Tube;

    params.constraints.StateMatrix = State_const.A;
    params.constraints.StateRHS = State_const.b;
    params.constraints.InputMatrix = Input_const.A;
    params.constraints.InputRHS = Input_const.b;
end