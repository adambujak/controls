make ARGS="1000 1500 30 0.1 0 .05" run && gnuplot plot.plt

kp = 1000
ki = 1500
kd = 30

I tuned the pid with the above parameters and got good results but need to investigate the max force applied to see if feasivle
