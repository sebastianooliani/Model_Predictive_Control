clear all
close all

%%
Q = [1 4; 4 1];
xu = [5; 6];
xl = [-5; -5];

x0 = [0; 0];

fun = @(x) x' * Q * x;
[x, FVAL, ~, ~, LAMBDA] = fmincon(fun, x0, zeros(2,2), zeros(2,1), zeros(2,2), zeros(2,1), xl, xu);

fprintf("\nx0: %d %d\n", x0(1), x0(2))
fprintf("\nx: %d %d\n", x(1), x(2))
fprintf("\nf(x): %d\n", FVAL)
fprintf("\nlambda_l: %d %d\n", LAMBDA.lower(1), LAMBDA.lower(2))
fprintf("\nlambda_u: %d %d\n", LAMBDA.upper(1), LAMBDA.upper(2))

%%
x0 = [1; -1];
[x, FVAL, ~, ~, LAMBDA] = fmincon(fun, x0, zeros(2,2), zeros(2,1), zeros(2,2), zeros(2,1), xl, xu);

fprintf("\nx0: %d %d\n", x0(1), x0(2))
fprintf("\nx: %d %d\n", x(1), x(2))
fprintf("\nf(x): %d\n", FVAL)
fprintf("\nlambda_l: %d %d\n", LAMBDA.lower(1), LAMBDA.lower(2))
fprintf("\nlambda_u: %d %d\n", LAMBDA.upper(1), LAMBDA.upper(2))

%%
x0 = [-1; 1];
[x, FVAL, ~, ~, LAMBDA] = fmincon(fun, x0, zeros(2,2), zeros(2,1), zeros(2,2), zeros(2,1), xl, xu);

fprintf("\nx0: %d %d\n", x0(1), x0(2))
fprintf("\nx: %d %d\n", x(1), x(2))
fprintf("\nf(x): %d\n", FVAL)
fprintf("\nlambda_l: %d %d\n", LAMBDA.lower(1), LAMBDA.lower(2))
fprintf("\nlambda_u: %d %d\n", LAMBDA.upper(1), LAMBDA.upper(2))