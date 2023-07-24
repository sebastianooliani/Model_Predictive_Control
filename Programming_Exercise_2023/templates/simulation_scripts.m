function simulation_scripts(Task)

[params] = generate_params();

if Task == 8
    simulate_task_8(params);
end

end

function simulate_task_8(params)
x0 = params.model.InitialConditionA;

% Set q and r values
qx = 1;
qy = 1;
qz = 1;
qvx = 0.1;
qvy = 0.1;
qvz = 0.1;

rx = 0;
ry = 0;
rz = 0;

q = [qx; qy; qz; qvx; qvy; qvz];
Q = diag(q);

r = [rx; ry; rz];
R = diag(r);

% Initialize controller
ctrl = LQR(Q, R, params);

[Xt,Ut,ctrl_info] = simulate(x0, ctrl, params);

[fig_time,axes_time,fig_pos,axes_pos] = plot_trajectory(Xt,Ut,ctrl_info,params);

end