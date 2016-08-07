import scikits.bvp_solver
import numpy,math

# next we define the important constants
X1T0 = 0
X1Tf = 10
X2T0 = 0
X2Tf = -2
TF = 10
wAltitude, wSpeed = 1,+0.2
def function(a , T):
    return numpy.array([T[1] ,        # evaluate dT1/dA
                       (wAltitude/wSpeed)*T[0]])    # evaluate dT2/dAa

def boundary_conditions(Ta,Tb):
    return (numpy.array([Ta[0] - X1T0]),  #evaluate the difference between the temperature of the hot stream on the
            numpy.array([Tb[1] - X2Tf]))#evaluate the difference between the temperature of the cold stream on the

"""
Next we create the ProblemDefinition object, by passing the relevant information and callbacks to its constructor.
"""
problem = scikits.bvp_solver.ProblemDefinition(num_ODE = 2,
                                      num_parameters = 0,
                                      num_left_boundary_conditions = 1,
                                      boundary_points = (0, TF),
                                      function = function,
                                      boundary_conditions = boundary_conditions)

solution = scikits.bvp_solver.solve(problem,
                            solution_guess = ((X1T0 + X2Tf)/2.0,
                                              (X1T0 + X2Tf)/2.0))
"""
The "solve" function returns a Solution object which can be passed an array of points at which to evaluate the solution.
"""
A = numpy.linspace(0,TF, 101)
T = solution(A)
print T

"""
We can plot the solution using pylab with the following code
"""
import pylab
pylab.plot(A, T[0,:],'b-',label=('x(t)'))
pylab.plot(A, T[1,:],'v',label=('v(t)'))
pylab.xlabel('t')
pylab.grid()
pylab.show()
