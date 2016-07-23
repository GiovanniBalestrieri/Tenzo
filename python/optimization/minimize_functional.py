#!/usr/bin/python

from math import *
from sympy import *
import numpy as np
import scipy.integrate as integrate
from scipy.optimize import minimize_scalar,minimize
import matplotlib.pyplot as plt

# We want to find the optimal curve of minimium length that connects point X0 to Xf 

# in our case the matrix A is given as follows:
# [ a11 a12]  =  [  0  1 ]
# [ a21 a22]     [ -1 -2 ]

a11 = 0
a12 = 1
a21 = -1
a22 = -2

def pend(X, t, a11, a12, a21, a22):
     dydt = [a11*X[0] + a12*X[1], a21*X[0] + a22*X[1]]
     return dydt


def main():
	a_t = np.arange(0, 25.0, 0.01)
	asol = integrate.odeint(pend, [1, 0], a_t, args=(a11,a12,a21,a22))
	print(asol)
	plt.plot(a_t, asol[:, 0], 'b', label='theta(t)')
	plt.plot(a_t, asol[:, 1], 'g', label='omega(t)')
	plt.legend(loc='best')
	plt.xlabel('t')
	plt.grid()
	plt.show()	

if __name__ == '__main__':
    main()

#cons = ({'type': 'ineq', 'fun': lambda x:  x[0] - 2 * x[1] + 2},{'type': 'ineq', 'fun': lambda x: -x[0] - 2 * x[1] + 6},{'type': 'ineq', 'fun': lambda x: -x[0] + 2 * x[1] + 2})

#bnds = ((0, None), (0, None))

#def fcn(x):
	#return (x[0] - 1)**2 + (x[1] - 2.5)**2
#	return sqrt(1+diff(x,x)^2)

#def objfunctional(tf):
#        res = integrate.quad(function,0,tf)
        #print("Integrating ...\n Solution:")
        #print(res)
#        return res[0]

#def mini():
#        res = minimize(objfunctional,bounds=(0,4),method='bounded')
#        return res
#res = minimize(fcn, (2, 0), method='SLSQP')#, bounds=bnds)#,constraints=cons)

#print(res)

