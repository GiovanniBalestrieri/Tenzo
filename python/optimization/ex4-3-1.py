#!/usr/bin/python

from math import *
from sympy import *
import numpy as np
import scipy.integrate as integrate
from scipy.optimize import minimize_scalar,minimize
import matplotlib.pyplot as plt

def pend(X, t):
     dydt = [X[2],X[3], X[0]*X[1]*X[3]/(X[3]*t**2+X[3]**2) -4*t*X[2]**2/(2*(X[2]*t**2+X[3]**2)) - X[3]*X[0]**2*X[2]/(2*(X[2]*t**2+X[3]**2)),-X[1]**2/2*X[2]-X[3]/(X[2]**2*t**2+X[2]*X[3]**2)*((X[0]*X[1]*X[2]**2)/2-2*t*X[2]**2-(X[3]*X[0]**2*X[2])/2) ]
     return dydt


def main():
	#a_t = np.arange(0, 25.0, 0.01)
	X1 = np.linspace(0,2,101)
	#T = np.array([X1,X2,X3,X4])
	#print(X1)
	X0 = [1,1,1,1]
	print(X0)
	print(pend(X0,1))
	
	#a_t = np.mgrid[0:10.0:0.01,0:10:0.01,0:10:0.01,0:10:0.01]  
	#print(a_t[0])
	asol = integrate.odeint(pend,[0,0,1,1], X1)
	#print(asol)
	plt.plot(X1, asol[:, 0], 'b', label='X(t)')
	plt.plot(X1, asol[:, 1], 'g', label='Y(t)')
	plt.plot(X1, asol[:, 2], 'b*', label='vX(t)')
	plt.plot(X1, asol[:, 3], 'g*', label='vY(t)')
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

