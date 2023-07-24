%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H, h] = lqr_maxPI(Q,R,params)
	% YOUR CODE HERE
    obj = LQR(Q, R, params);
    K = obj.K;
    A = params.model.A;
    B = params.model.B;
    H_x = params.constraints.StateMatrix;
    h_x = params.constraints.StateRHS;
    H_u = params.constraints.InputMatrix;
    h_u = params.constraints.InputRHS;
    A_cl = A - B * K;

    system = LTISystem('A', A_cl, 'B', zeros(params.model.nx, params.model.nu));
    % system.initialize('xinit', zeros(nx, 1))
    set_State = Polyhedron('A', H_x, 'b', h_x);
    set_Input = Polyhedron('A', H_u * K, 'b', h_u);

    system.x.with('setConstraint');
    system.x.setConstraint = intersect(set_State, set_Input);
    InvSet = system.invariantSet();

    H = InvSet.A;
    h = InvSet.b;
end

