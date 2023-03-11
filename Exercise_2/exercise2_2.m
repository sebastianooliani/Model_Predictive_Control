close all;
clear all;

%% 1. Finite Horizon Controller: Batch
% Optimal control input and optimal cost
N = 50;
x0 = [1 -1]';

A = [0.77 -0.35; ...
     0.49 0.91];
B = [0.04 0.15]';

R = 1;
Q = diag([500, 100]);
P = diag([1500, 100]);

Sx = ones(2 * N + 2, 2);
Su = zeros(2 * N + 2, 2 * N);

Sx(2, 1) = 0;
Sx(1, 2) = 0;
for i = 1:N
    Anew = A ^ i;
    Sx(i * 2 + 1 , :) = Anew(1, :);
    Sx(i * 2 + 2, :) = Anew(2, :);
end

vector = zeros(2 * N + 2, 2);
for i = 1:N
    Cnew = A ^ (i-1) * B;
    vector(i * 2 + 1 , :) = Cnew(1, :);
    vector(i * 2 + 2, :) = Cnew(2, :);
end

Su(:, 1:2) = vector;
for i=1:N-1
    Su(i*2+1:end, i*2+1:i*2+2) = vector(1:2*N-(i-1)*2, :);
end

Q_bar = zeros(2 * N + 2, 2 * N + 2);
vector = [Q; zeros(2 * N, 2)];
Q_bar(1,1) = Q(1,1);
Q_bar(2,2) = Q(2,2);
Q_bar(2*N+1, 2*N+1) = P(1,1);
Q_bar(2*N+2, 2*N+2) = P(2,2);
for i=1:N-1
    Q_bar(i*2+1:end, i*2+1:i*2+2) = vector(1:2*N-(i-1)*2, :);
end

R_bar = diag([ones(1, 2 * N)]);

U0 = - inv(Su' * Q_bar * Su + R_bar) * Su' * Q_bar * Sx * x0;
J0 = x0' * (Sx' * Q_bar * Sx - Sx' * Q_bar * Su * inv(Su' * Q_bar * Su + R_bar) * Su' * Q_bar * Sx) * x0;

%% 2. verify the result by solving an opt. problem
H = 2 * (Su' * Q_bar * Su + R_bar);
F = Sx' * Q_bar * Su;
f = 2 * x0' * F;

% X = quadprog(H,f,A,b);

%% 3. Recursive approach
Pnow = P;

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
for i = N-1:-1:1
    Pnow = [P(i * 2 + 1 , :); P(i * 2 + 2, :)];
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

Finite_cost = 0;

for i=1:N-1
    Finite_cost = Finite_cost + x(:, i)' * Q * x(:, i) + u(:, i)' * R * u(:, i);
end

%% 4. comparison
D = [0.1 0.1]';
w = randn(N, 1) * sqrt(10);