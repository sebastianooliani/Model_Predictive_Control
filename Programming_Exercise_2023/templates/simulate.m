%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2023, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xt,Ut,ctrl_info] = simulate(x0, ctrl, params)

% YOUR CODE HERE
% Hint: you can access the control command with ctrl.eval(x(:,i))
Nt = params.model.HorizonLength;
A = params.model.A;
B = params.model.B;

Ut = zeros(params.model.nu, Nt);
Xt = zeros(params.model.nx, Nt + 1);
ctrl_info = [];

Xt(:, 1) = x0;

for i=1:1:Nt
    [Ut(:, i), ctrl_info_temp] = ctrl.eval(Xt(:, i));
    Xt(:, i + 1) = A * Xt(:, i) + B * Ut(:, i);
    ctrl_info = [ctrl_info, ctrl_info_temp];
end

end