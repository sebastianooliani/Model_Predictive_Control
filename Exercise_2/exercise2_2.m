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
Su = zeros(2 * N + 2, N);

Sx = eye(size(A));
Su = zeros(size(A,1)*N,size(B,2)*N);

for i=1:N
    Sx = [Sx; A^i];
    Su = Su + kron( diag( ones(N-i+1,1) , -i+1 ) , A^(i-1)*B );
end

Su = [ zeros(size(A,1),size(B,2)*N) ; Su ];

Q_bar = zeros(2 * N + 2, 2 * N + 2);
vector = [Q; zeros(2 * N, 2)];
Q_bar(1,1) = Q(1,1);
Q_bar(2,2) = Q(2,2);
Q_bar(2*N+1, 2*N+1) = P(1,1);
Q_bar(2*N+2, 2*N+2) = P(2,2);
for i=1:N-1
    Q_bar(i*2+1:end, i*2+1:i*2+2) = vector(1:2*N-(i-1)*2, :);
end

R_bar = diag([ones(1, N)]);

U0 = - inv(Su' * Q_bar * Su + R_bar) * Su' * Q_bar * Sx * x0;
J0 = x0' * (Sx' * Q_bar * Sx - Sx' * Q_bar * Su * inv(Su' * Q_bar * Su + R_bar) * Su' * Q_bar * Sx) * x0;

fprintf("Cost batch approach: %d\n", J0)

%% 2. verify the result by solving an opt. problem
H = 2 * (Su' * Q_bar * Su + R_bar);
F = Sx' * Q_bar * Su;
f = 2 * x0' * F;

[Ustar, costQ] = quadprog(H, f');
% Cost completion
costQ = costQ + x0'*Sx'*Q_bar*Sx*x0;
fprintf("Cost quadratic program: %d\n", costQ)

%% 3. Recursive approach
PRec{N+1} = P;
for i=N:-1:1
    PRec{i} = A'*PRec{i+1}*A + Q - A'*PRec{i+1}*B...
        *(B'*PRec{i+1}*B + R)^(-1)*B'*PRec{i+1}*A;
end
% Optimum of the recursive approach
costRec = x0'*PRec{1}*x0;

fprintf("Cost recursive approach: %d\n", costRec)
