%%
clc
A = [0, 1, 0; 
    2*9.8, 0, -6.26;
    0, 0, -3];

B = [0; 0; 1];

C = [1, 0, 0];

D = 0;

eig(A)

%%
[numerator, denominator] = ss2tf(A, B, C, D);

G = tf(numerator, denominator)

zpk(G)

%%
clc
[sizes, x0, states] = magball

%%
xstar = [1;
         0;
         (9.8)^(0.5)];
ustar = 3*(9.8^(0.5));

[A, B, C, D] = linmod('magball', xstar, ustar)

%%
clc
system = ss(A, B, C, D);
impulse(system, 2)

%%
figure
step(system, 2)