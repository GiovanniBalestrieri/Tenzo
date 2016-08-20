#!/usr/bin/python

from numpy import zeros,float64,array
from run_kut5 import *
from linInterp import *
from printSoln import *
import matplotlib.pyplot as plt

def initCond(u):
	# Initial values of [y,y',y''];
	# use 'u' if unknown
	#return array([0.0, 0.0, u])

	return array([0.0, 0.0])

def r(u):
	# Boundary condition residual--see Eq. (8.3)
	X,Y = integrate(F,xStart,initCond(u),xStop,h)
	y = Y[len(Y) - 1]
	r = y[0] - 2.0
	return r

def F(x,y):
	# First-order differential equations
	F = zeros((3))
	F[0] = y[1]
	F[1] = y[1]**4/(1+y[1]**2)
	return F

t0 = 0.0 # Start of integration
tf = 5.0 # End of integration
u1 = 1.0 # 1st trial value of unknown init. cond.
u2 = 2.0 # 2nd trial value of unknown init. cond.
h = -0.1 # initial step size
freq = 2 # printout frequency

def main():
	#u = linInterp(r,u1,u2)
	#X,Y = integrate(F,xStart,initCond(u),xStop,h)
	X,Y = integrate(F,t0,tf,h)
	printSoln(X,Y,freq)
	plt.plot(Y, 'g', label='x2(t)')
	plt.legend(loc='best')
	plt.xlabel('t')
	plt.grid()
	plt.show()	

if __name__ == '__main__':
    main()

