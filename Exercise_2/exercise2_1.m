close all;
clear all;

%% A. Finite Horizon Controller
N = 5; % minimum value to have same final cost
x0 = [10 10]';

A = [4/3 -2/3; ...
     1 0];
B = [1 0]';
C = [-2/3 1];
D = 0;

R = 0.001;
Q = C' * C + 0.001 * eye(2);
P = Q;

% discrete time Riccati equation
F = zeros(2, 1);
for i = N:-1:1
    F = -inv(R + B' * P * B) * B' * P * A;
    P = A' * P * A + Q - A' * P * B * inv(B' * P * B + R) * B' * P * A;
    Fpred{i} = F;
end

% state trajectory
u = zeros(1, N);
x = zeros(2, N);

x(:,1) = x0;
for i = 1:length(Fpred)-1
    x(:, i+1) = A * x(:, i) + B * Fpred{i} * x(:, i);
end

figure(1)
plot(x(1, 1:length(Fpred)), x(2, 1:length(Fpred)), 'ro-', LineWidth=2)
hold on;
title('State trajectory N = 5')
xlabel('x1')
ylabel('x2')
hold off;

%% Receding horizon fashion
figure(2)

x(:,1) = x0;
i = 1;

while 1
    if norm(x(:,i)) > 3*norm(x0), fprintf('===> System unstable\n'); break; end
    if norm(x(:,i)) < 5e-2, fprintf('===> System stable\n'); break; end
    
    u(:, i) = F * x(:, i);
    x(:, i+1) = A * x(:, i) + B * u(:, i);
    % Plot prediction
    x_pred = compute_prediction(x(:, i), Fpred, A, B);
    
    clf;hold on;grid on;
    plot(x_pred(1, :), x_pred(2, :), '-o');
    plot(x(1, 1:i) , x(2, 1:i) , 'LineWidth', 1.5);
    legend('Prediction','Closed Loop System');
    title('State Space');
    pause(0.1)
    i = i + 1;
end

fprintf("\nMinimum time horizon: %d\n", i+1)

%% B. Infinite horizon LQR controller
% Compare cost
[Klqr,~,~] = dlqr(A,B,Q,R);
costlqr = compute_cost(x0, A - B * Klqr, Klqr , Q, R, 1000);
costPn = compute_cost(x0, A + B * F, F , Q, R, 1000);

fprintf('Cost of the optimal LQR controller : %.2f\n', costlqr);
fprintf('Cost of the N−step controller : %.2f\n', costPn);

%%
function x = compute_prediction(x0,K,A,B)
x(:,1) = x0;
for i = 1:length(K)
    x(:,i+1) = (A + B * K{i}) * x(:,i);
end

end

function cost = compute_cost(x0,Ak,K,Q,R,steps)
cost = 0;
x = x0;

for i=1:steps
    cost = cost + x'*(Q+K'*R*K)*x;
    x = Ak*x;
end

end