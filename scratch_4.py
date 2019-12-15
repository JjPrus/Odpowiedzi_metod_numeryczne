# Optimizing exponential
from scipy import linspace ,sin, cos , exp, random, meshgrid, zeros
from scipy.optimize import fmin
from matplotlib.pyplot import plot, show, legend, figure, cm, contour, clabel
from scipy import optimize as opt
from mpl_toolkits.mplot3d import Axes3D

def f(x):
   return sin(5*x[1]/3) * exp(1) * cos( x[0]*5/3) *x[0] *x[1]

#def f(x):
 #   return -sin(sin(5*x[1]/3) * exp(1) * cos( x[0]*5/3))/20



#n liczba powtórzeń
n=2500
tmin=[] #tablica lokalnych minimów
w=0 #licznik
for i in range (n):
    x00 = [random.randn(1)*5, random.rand(1)*5]
    x_m = fmin(f, x00)

    for i in range(len(tmin)):
        if (x_m[1]==tmin[i][1] and x_m[0]==tmin[i][0]):
            w=w+1
    if w==0:
        tmin.append(x_m)
    w=0

xm = 0
ym = 0
zm = 0
#wspolrzedne globalnego min
ax = Axes3D(figure(figsize=(8, 5)))

for i in range(len(tmin)):
    if zm > f(tmin[i]):
        xm=tmin[i][0]
        ym = tmin[i][1]
        zm= f(tmin[i])
    ax.plot([tmin[i][0]], [tmin[i][1]], [f(tmin[i])], color='k', marker='o', markersize=5, label='final')
#szukanie globalnego z lokalnych plus dodawanie punktw lokalnych

zp=[xm, ym]
def sortSecond(val):
    return val[0]
tmin.sort(key = sortSecond)
x0 = random.randn(2)
x_min = zp
delta = f(tmin[0])
deltaplus=f(tmin[len(tmin)-1])
x_knots = linspace(-15, 15, 41)
y_knots = linspace(-6, 6, 41)
X, Y = meshgrid(x_knots, y_knots)
Z = zeros(X.shape)
for i in range(Z.shape[0]):
    for j in range(Z.shape[1]):
        Z[i][j] = f([X[i, j], Y[i, j]])


ax.autoscale_view(tight=None, scalex=True, scaley=True, scalez=True)
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0.4)
ax.plot([zp[0]], [zp[1]], [f(zp)], color='g', marker='o', markersize=5, label='initial')
ax.legend()
show()