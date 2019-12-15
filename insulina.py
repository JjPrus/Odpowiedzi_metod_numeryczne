import numpy as np
import matplotlib.pyplot as plt

import numpy as np
import scipy.io
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy import linspace , cos , exp, random, meshgrid, zeros
from scipy.optimize import fmin
from matplotlib.pyplot import plot, show, legend, figure, cm, contour, clabel
from mpl_toolkits.mplot3d import Axes3D
from scipy import optimize as opt

plik = open('cukry.txt')
try:
	tekst = plik.read()
finally:
	plik.close()

print (tekst[8])
#get data
zbior1_mat = scipy.io.loadmat("zadanie_3_zbior_1.mat")
zbior2_mat = scipy.io.loadmat("zadanie_3_zbior_2.mat")
u1 = np.array(zbior1_mat['u'])
u1 = u1[0]
y1 = np.array(zbior1_mat['y'])
y1 = y1[0]

u2 = np.array(zbior2_mat['u'])
u2 = u2[0]
y2 = np.array(zbior2_mat['y'])
y2 = y2[0]

print(u1)
print(y1)
print(u2)
print(y2)
plt.plot(y1,'k', label='x')
#plt.plot(u2,y2,'g', label='y')
plt.xlabel('t', fontsize=14)
plt.ylabel('state', fontsize=14)
plt.legend(loc='upper right', fontsize=14)
plt.show()