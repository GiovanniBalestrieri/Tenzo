#!/usr/bin/python

from math import *
from sympy import *
import numpy as np
import scipy.integrate as integrate
from scipy.optimize import minimize_scalar,minimize
import matplotlib.pyplot as plt


# Minimun distance
def pend(X, t):
     dydt = [ X[1],X[1]**2/(1+X[1]**2) ]
     return dydt


def main():
	#a_t = np.arange(0, 25.0, 0.01)
	X1 = np.linspace(0,4,101)
	#T = np.array([X1,X2,X3,X4])
	#print(X1)
	X0 = [1,1]
	print(X0)
	print(pend(X0,1))
	
	asol = integrate.odeint(pend,[5,0.1], X1)
	
	# Plot solutions
	plt.plot(X1, asol[:, 0], 'b', label='X(t)')
	plt.plot(X1, asol[:, 1], 'g', label='Y(t)')
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

