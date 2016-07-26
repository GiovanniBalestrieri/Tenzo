#!/usr/bin/python

from math import *
from sympy import *
import numpy as np
import scipy.integrate as integrate
from scipy.optimize import minimize_scalar,minimize
import matplotlib.pyplot as plt

def pend(X, t):
     dydt = [X[0]*X[1]*X[3]/(X[3]*t**2+X[3]**2) -4*t*X[2]**2/(2*(X[2]*t**2+X[3]**2)) - X[3]*X[0]**2*X[2]/(2*(X[2]*t**2+X[3]**2)),-X[1]**2/2*X[2]-X[3]/(X[2]**2*t**2+X[2]*X[3]**2)*((X[0]*X[1]*X[2]**2)/2-2*t*X[2]**2-(X[3]*X[0]**2*X[2])/2) ]
     return dydt


def main():
	#a_t = np.arange(0, 25.0, 0.01)
	T = np.mgrid[-6:7,0:10:0.1]
	X1 = np.linspace(0,10,100)
	X2 = np.linspace(0,10,100)
	X3 = np.linspace(0,10,100)
	X4 = np.linspace(0,10,100)
	XX = np.array([X1,X2,X3,X4])
	print(XX)
	print(T[1])
	print(T[0])
	#a_t = np.mgrid[0:10.0:0.01,0:10:0.01,0:10:0.01,0:10:0.01]  
	#print(a_t[0])
	#asol = integrate.odeint(pend, [1,1,0,0], a_t)
	#print(asol)
	#plt.plot(a_t, asol[:, 0], 'b', label='theta(t)')
	#plt.plot(a_t, asol[:, 1], 'g', label='omega(t)')
	plt.legend(loc='best')
	plt.xlabel('t')
	plt.grid()
	#plt.show()	

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

