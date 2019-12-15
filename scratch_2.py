from scipy import optimize as opt
from scipy import linspace , cos , exp, random, meshgrid, zeros
from scipy.optimize import fmin
from matplotlib.pyplot import plot, show, legend, figure, cm, contour, clabel


def funkcja_celu(x):
    return x[0]**2+x[1]**2

#minimize(funkcja_celu, punkt_poczatkowy, method='Nulcler Meed')
#differential_evolution

x0 = random.randn()
minimum = opt.minimize(funkcja_celu(), x0, 'Nelder-Mead')
print(minimum)