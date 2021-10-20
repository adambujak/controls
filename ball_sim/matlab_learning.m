h = 0.01;
A = [1 h; 0 1];
B = [0; h];
C = [1 0];
D = [0];
x0 = [10; -1];

m = 1;
g = 10;
fa = 9.9;

u(1:1000) = fa/m - g;
t = transpose([0:0.01:9.99]);

sys = ss(A, B, C, D, h);
lsim(sys, u, t, x0)