#!/usr/bin/python

from math import *
from sympy import *
import numpy as np
import scipy.integrate as integrate
from scipy.optimize import minimize_scalar,minimize
import matplotlib.pyplot as plt

# We want to find the optimal curve of minimium length that connects point X0 to Xf 

t = np.linspace(0, 5, 10)

def pend(t,X):
     dydt =diff(X)/sqrt(1+pow(diff(X),2))
     return dydt


def main():
	asol = integrate.odeint(pend,1, t)
	print(asol)
	plt.plot(t, asol, 'b', label='theta(t)')
	plt.legend(loc='best')
	plt.xlabel('t')
	plt.grid()
	plt.show()	

if __name__ == '__main__':
    main()


