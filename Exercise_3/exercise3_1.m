clear all
close all

%% 1. stability
A = [0.995 0.099;
    -0.099 0.985];

x0 = [1; 0];

Q = [2 0;
    0 1];

lambda = eig(A);

if norm(lambda(1)) < 1 && norm(lambda(2)) < 1
    fprintf("Discrete time system is asymptotically stable!\n")
end

%% 2. cost function
N = 1000;

x = x0;
cost = zeros(N, 1);
cost(1) = x' * Q * x;
cost_now = cost(1);

for i=2:N
    x = A * x;
    cost_now = cost_now + x' * Q * x;
    cost(i) = cost_now;
end

figure(1)
plot(1:N, cost)
hold on;

%% 3. infinite horizon cost
P = dlyap(A,Q);
cost_lyap = x0' * P * x0;

plot(1:N, cost_lyap - cost)
hold off;
legend("Cost function", "Lyapunov function - cost function")
xlabel("Timestep")