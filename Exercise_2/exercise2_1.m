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
Pnow = Q;

P = zeros(2 * N, 2);

P(2 * N - 1, :) = Pnow(1, :);
P(2 * N, :) = Pnow(2, :);

% discrete time Riccati equation

for i = N-1:-1:1
    Pnext = A' * Pnow * A + Q - A' * Pnow * B * inv(B' * Pnow * B + R) * B' * Pnow * A;
    P(i * 2 - 1 , :) = Pnext(1, :);
    P(i * 2, :) = Pnext(2, :);
    Pnow = Pnext;
end

F = zeros(2 * N, 1);
for i = N:-1:1
    Pnow = [P(i * 2 - 1, :); P(i * 2, :)];
    Fnext = - inv(B' * Pnow * B + R) * B' * Pnow * A;
    F(i * 2 - 1 , :) = Fnext(1);
    F(i * 2, :) = Fnext(2);
end

% state trajectory
u = zeros(2, N);
x = zeros(2, N);

x(:, 1) = x0;

for i=1:N-1
    Fnow = [F(i * 2 - 1); F(i * 2)];
    u(:, i) = Fnow' * x(:, i);
    x(:, i + 1) = A * x(:, i) + B' * u(:, i);
end

figure(1)
plot(x(1, 1:N), x(2, 1:N), 'ro-', LineWidth=2)
hold on;
title('State trajectory')
xlabel('x1')
ylabel('x2')

Finite_cost = 0;

for i=1:N-1
    Finite_cost = Finite_cost + x(:, i)' * Q * x(:, i) + u(:, i)' * R * u(:, i);
end

%% Receding horizon fashion
x0 = [10 10]';
x = x0;
n = 1;
% while x(1) ~= 0 && x(2) ~= 0
%     [K, S, CLP] = dlqr(A, B, Q, R);
%     x = A * x + B * K * x;
%     n = n + 1;
% end

%% B. Infinite horizon LQR controller
N = 1000;
x0 = [10 10]';

A = [4/3 -2/3; ...
     1 0];
B = [1 0]';
C = [-2/3 1];
D = 0;

R = 0.001;
Q = C' * C + 0.001 * eye(2);
Pnow = Q;

P = zeros(2 * N, 2);

P(2 * N - 1, :) = Pnow(1, :);
P(2 * N, :) = Pnow(2, :);

% discrete time Riccati equation

for i = N-1:-1:1
    Pnext = A' * Pnow * A + Q - A' * Pnow * B * inv(B' * Pnow * B + R) * B' * Pnow * A;
    P(i * 2 - 1 , :) = Pnext(1, :);
    P(i * 2, :) = Pnext(2, :);
    Pnow = Pnext;
end

F = zeros(2 * N, 1);
for i = N:-1:1
    Pnow = [P(i * 2 - 1, :); P(i * 2, :)];
    Fnext = - inv(B' * Pnow * B + R) * B' * Pnow * A;
    F(i * 2 - 1 , :) = Fnext(1);
    F(i * 2, :) = Fnext(2);
end

% state trajectory
u = zeros(2, N);
x = zeros(2, N);

x(:, 1) = x0;

for i=1:N-1
    Fnow = [F(i * 2 - 1); F(i * 2)];
    u(:, i) = Fnow' * x(:, i);
    x(:, i + 1) = A * x(:, i) + B' * u(:, i);
end

plot(x(1, 1:N), x(2, 1:N), 'bo-', LineWidth=2)
hold off;
legend('Finite horizon', 'Infinite horizon')

Inf_cost = 0;

for i=1:N
    Inf_cost = Inf_cost + x(:, i)' * Q * x(:, i) + u(:, i)' * R * u(:, i);
end

% %% verify costs are the same
% if round(Finite_cost, 4) == round(Inf_cost, 4) 
%     fprintf('Costs are the same!\n')
%     fprintf('Final cost = %f\n', Finite_cost)
% else
%     fprintf('ERROR!! Costs are different!\n')
% end