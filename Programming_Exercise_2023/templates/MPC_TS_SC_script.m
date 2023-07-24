%% Task 23 - Final version

% Threshold
e = 1e-6;

[params] = generate_params();
nu = params.model.nu;
N = 30;

Q = diag([94.0;0.1579;300;0.01;0.1;0.1]);
R = eye(nu);

% Initial conditions
X0 = [...
    params.model.InitialConditionA,...
    params.model.InitialConditionB,...
    params.model.InitialConditionC];
[nx, n0] = size(X0);

% Compute Max Positive Invariant Set: 
[H, h] = lqr_maxPI(Q,R,params);

% S and v selections
Sgrid = 0:250:1000; % scalar
vgrid = 0:250:1000;

[Scoeff, vall] = ndgrid(Sgrid,vgrid);

% Control Object without slack constraints
ctrl = MPC_TS(Q,R,N,H,h,params);

[I,J] = size(Scoeff);
max_u_deviation = zeros([I,J,n0]);

for i = 1:I
    for j = 1:J
% Control Object with slack constraints
S = Scoeff(i,j)*eye(length(params.constraints.StateRHS));
v = vall(i,j);
ctrl_slack = MPC_TS_SC(Q,R,N,H,h,S,v,params);

% U_slack = zeros([nu, n0]);
% U = zeros([nu, n0]);
% U_match = zeros([1, n0]);
for k = 1:n0-1
    [u, ctrl_info] = ctrl.eval(X0(:,k));
    %U(:, k) = u;

    [u_slack, ctrl_info] = ctrl_slack.eval(X0(:,k));
    %U_slack(:, k) = u_slack;

    max_u_deviation(i,j,k) = max(abs(u_slack - u));
    
%     if all(abs(u_slack - u) <= e)
%         U_match(k) = 1;
%     end
end

    end
end

% Return lowest S and v with the min max_u_deviation
u_dev_bool = (max_u_deviation <= e);
u_dev_bool_combined = ones(size(u_dev_bool(:,:,1)));
for i=1:n0
    u_dev_bool_combined=u_dev_bool_combined.*u_dev_bool(:,:,i);
end
candidate_i = min(find(u_dev_bool_combined));

S = Scoeff(candidate_i)*eye(length(params.constraints.StateRHS));
v = vall(candidate_i);

save('MPC_TS_SC_params.mat', 'S', 'v');
