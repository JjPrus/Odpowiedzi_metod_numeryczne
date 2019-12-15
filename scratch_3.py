import numpy as np
import matplotlib.pyplot as plt

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy import linspace , cos , exp, random, meshgrid, zeros
from scipy.optimize import fmin
from matplotlib.pyplot import plot, show, legend, figure, cm, contour, clabel
from mpl_toolkits.mplot3d import Axes3D
from scipy import optimize as opt


xx=linspace(-5, 5, 50)
yy=xx
X, Y =meshgrid (xx, yy)
Z= (1-X)**2+100*(Y-X**2)**2
ax= Axes3D(plt.figure())
ax.plot_surface(X, Y, Z)

def f(x):
    return (1-x[0])**2+100*(x[1]-x[0]**2)**2
x0=random.randn(2)

x_min=opt.minimize(f, x0, method='Nelder-Mead', tol=1e-6)
ax.plot([x_min.x[0]], [x_min.x[1]], [f(x_min.x)], color='k', marker='o', markersize=5, label='final')
show()

def y(t):
    return exp(-2*t)
iksy=linspace(0, 100, 100)