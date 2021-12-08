fc = 10000;

figure; hold on
for N0 = 0.01:0.001:0.1
    pe = qfunc(sqrt(5/(4*N0*fc)));
    plot(N0, pe, '.');
end