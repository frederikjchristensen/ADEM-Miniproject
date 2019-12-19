from sympy import *
from numpy import pi
t = Symbol("t")
y = -2*t**2

def fourier_series_n_plotter(i,y):
    a0 = 1/pi * integrate (y, (t, -pi,pi))
    fs = 0
    for n in range (1, i+1):
        an = 1/pi * integrate ((y)*cos(n*t),(t,-pi,pi))  
        bn = 1/pi * integrate ((y)*sin(n*t),(t,-pi,pi))
        fs += an*cos(n*t) + bn*sin(n*t)
    func = a0/2 + fs
    print("N:",i)
    print(func)
    plot(func,(t,-pi,pi))
    
fourier_series_n_plotter(1,y)
fourier_series_n_plotter(3,y)
fourier_series_n_plotter(5,y)
fourier_series_n_plotter(10,y)
fourier_series_n_plotter(50,y)

print("Original function -2t^2")
plot(y,(t,-pi,pi))

