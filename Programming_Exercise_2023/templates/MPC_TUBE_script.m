%% Task 31
clear all;
clc;

[params] = generate_params();
[params_z] = generate_params_z(params);
nx = params.model.nx;
N = 50;

Q = diag([300;0.1]);
R = 1;
p = [0.05, 0.1];

K_tube = compute_tube_controller(p, params_z);
[H_tube, h_tube, n_iter] = compute_minRPI(K_tube, params_z);
params_z_tube = compute_tightening(K_tube, H_tube, h_tube, params_z);
[H_N, h_N] = lqr_maxPI(Q, R, params_z_tube);

save('MPC_TUBE_params.mat', 'p', 'K_tube', 'H_tube', 'h_tube', 'H_N', 'h_N', 'params_z_tube');