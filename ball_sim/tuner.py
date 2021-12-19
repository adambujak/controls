import os
import matplotlib.pyplot as plt

def get_kp():
    print('enter kp: ')
    #return 0
    return input()

def get_ki():
    print('enter ki: ')
    return 0
    #return input()

def get_kd():
    print('enter kd: ')
    #return 0
    return input()

while 1:
    kp = get_kp()
    ki = get_ki()
    kd = get_kd()

    os.system('make ARGS="{} {} {}" run > /dev/null'.format(kp, ki, kd))
    os.system('gnuplot plot.plt')
