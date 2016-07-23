#!/usr/bin/python

from math import *
import scipy.integrate as integrate
from scipy.optimize import minimize_scalar,minimize

def function(x):
	y = -sin(x)
	return y

def objfunctional(tf):
	res = integrate.quad(function,0,tf)
	#print("Integrating ...\n Solution:")
	#print(res)
	return res[0]

def mini():
	res = minimize(objfunctional,bounds=(0,4),method='bounded')
	return res

print(mini())


