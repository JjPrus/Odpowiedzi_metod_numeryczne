from scipy import optimize as opt


def funkcja_celu():
    return x[0]**2+x[1]**2

#minimize(funkcja_celu, punkt_poczatkowy, method='Nulcler Meed')
#differential_evolution

x0=randn(1)
print(opt.minimize(funkcja_celu(), x0, 'Nelder-Mead'))