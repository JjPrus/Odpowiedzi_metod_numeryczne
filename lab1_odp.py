#-*- coding: utf-8 -*-
from numpy import *
from numpy.random import *
from matplotlib.pyplot import *


x=0
y=0
for x in range(56, 101):
     y = 2*x*x + 2*x + 2
     print(f"X, Y: {x, y}")


x1=linspace(56, 100)
y1 = 2*x1*x1 + 2*x1 + 2
plot(x1,y1)
#show()

fac=int(input("insert value, u gona get factorial of it "))
n=int(fabs(fac))

if(fac<0):
    print("ERROR_YOU_DO_NOT_LIKE_COMPLEXES")
    fac=(-1)**fac
else:
    fac=1
for i in range (1, int(n+1)):
    fac=fac*i
print("urs factorial: ")
print(int(fac))

def lst (tab):
    min=tab [0]
    for x in tab:
        if min>x:
            min=x
    i=tab.index(x)
    print("Lst val and idx of it:")
    return (print(min, i))
argray=[]
print("Give me array, get lst val of it  ")
n=int(input("How many var?"))
for i in range(n):
    x=int(input())
    argray.append(x)
lst(argray)

def wykres(n):
    x_values = zeros(n)
    y_values = zeros(n)
    circ_size= zeros(n)
    for i in range(n - 1):
        x_values[i + 1] = x_values[i] + randn()
        y_values[i + 1] = y_values[i] + x_values[i]**2 - e**x_values[i]
        circ_size[i + 1]=circ_size[i]+ x_values[i]
    scatter(x_values, y_values, circ_size, c='yellow')
    axis('off')
    show()
    figure(dpi=128, figsize=(7, 4))

wykres(6789)