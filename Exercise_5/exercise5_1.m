clear all;
close all;

%% 1
x0 = 2;
n = 5; % number of decision variables
% x = [x0, x1, x2, u0, u1]
H = eye(n);
Aeq = [1 0 0 0 0;
    -0.5 1 0 -1 0;
    0 -0.5 1 0 -1;
    0 0 0 0 0;
    0 0 0 0 0];
beq = [x0; 0; 0; 0; 0];
LB = [x0; 2.5; -1; -2; -2];
UB = [x0; 5; 1; 2; 2];
A = [1 0 0 0 0;
    1 0 0 0 0;
    0 1 0 0 0;
    0 -1 0 0 0;
    0 0 1 0 0;
    0 0 -1 0 0;
    0 0 0 1 0;
    0 0 0 -1 0;
    0 0 0 0 1;
    0 0 0 0 -1];
b = [x0; x0; 5; -2.5; 1; 1; 2; 2; 2; 2];
[x] = quadprog(H, zeros(n,1), A, b, Aeq, beq);
% quadprog(H, b, A, b, Aeq, beq, LB, UB);

%% 2
[x, FVAL, ~, ~, LAMBDA] = quadprog(H, zeros(n,1), A, b, Aeq, beq);

for i=1:1:length(LAMBDA.ineqlin)
    if LAMBDA.ineqlin(i) < 0
        fprintf('\nNegative value of lambda! KKT conditions not satisfied!\n')
    end
end

% active constraints: x1 >= 2.5
%% 3
figure(1)
for i = 1:14
    x0 = i;
    beq = [x0; 0; 0; 0; 0];
    b = [x0; x0; 5; -2.5; 1; 1; 2; 2; 2; 2];
    [x, FVAL] = quadprog(H, zeros(n,1), A, b, Aeq, beq);
    plot(x0, FVAL,'ro-');
    hold on;
    plot(x0, x(4), 'bo-')
end
title('x0 vs f*(x0) & x0 vs u0*')
xlabel('x0')
ylabel('f*(x0) & u0*')
hold off

%% 4
% if we remove the lower bound on x1, its value will decrease -> lower cost.
% If we remove the upper bound on x1, the cost will not change.

%% 5
x0 = 2;
% cost function
f = [1 1 1 1 0 0]';
% inequality constraint
Geps = [-1 0 0 0;...
        -1 0 0 0;...
        0 -1 0 0;...
        0 -1 0 0;...
        0 0 -1 0;...
        0 0 -1 0;...
        0 0 0 -1;
        0 0 0 -1];

Gu = [1 0;...
    -1 0;...
    0.5 1;...
    -0.5 -1;...
    1 0;...
    -1 0;...
    0 1;...
    0 -1];

G = [1 0;...
    -1 0;...
    0.5 1;...
    -0.5 -1;...
    1 0;...
    -1 0;...
    0 1;...
    0 -1];

Eeps = [-0.5; 0.5; -0.25; 0.25; 0; 0; 0; 0];
Eu = [-0.5; +0.5; -0.25; 0.25; zeros(4,1)];
E = [Eeps; Eu];
w = [zeros(8,1); 5; -2.5; 1; 1; 2; 2; 2; 2];

% matrix that we will be feeded to linprog
A = [Geps Gu; zeros(8,4) G];
b = E*x0 + w;
[x_lin, fval] = linprog(f, A, b);
cost = fval + abs(x0);

%% 6
figure(2)
for i = 1:14
    x0 = i;
    b = E*x0 + w;
    [x_lin, fval] = linprog(f, A, b);
    cost = fval + abs(x0);
    plot(x0, cost,'ro-');
    hold on;
    plot(x0, x_lin(5), 'bo-')
end
title('x0 vs f*(x0) & x0 vs u0*')
xlabel('x0')
ylabel('f*(x0) & u0*')
hold off

%% 7
% if we remove the lower bound on x1, its value will decrease -> lower cost.
% If we remove the upper bound on x1, the cost will not change.
