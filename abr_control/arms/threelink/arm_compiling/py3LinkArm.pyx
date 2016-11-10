import numpy as np
cimport numpy as np
 
cdef extern from "threelinkarm.cpp": 
    cdef cppclass Sim:
        Sim(double dt, double* params)
        void reset(double* out, double* ic)
        void step(double* out, double* u)
 
cdef class pySim:
    cdef Sim* thisptr
    def __cinit__(self, double dt = .00001, 
                        np.ndarray[double, mode="c"] params=None):
        """
        param float dt: simulation timestep, must be < 1e-5
        param array params: MapleSim model internal parameters
        """
        if params: self.thisptr = new Sim(dt, &params[0])
        else: self.thisptr = new Sim(dt, NULL)
 
    def __dealloc__(self):
        del self.thisptr
 
    def reset(self, np.ndarray[double, mode="c"] out, 
                    np.ndarray[double, mode="c"] ic=None):
        """
        Reset the state of the simulation.
        param np.ndarray out: where to store the system output
            NOTE: output is of form [time, output]
        param np.ndarray ic: the initial conditions of the system
        """
        if ic is not None: self.thisptr.reset(&out[0], &ic[0])
        else: self.thisptr.reset(&out[0], NULL)
 
    def step(self, np.ndarray[double, mode="c"] out, 
                   np.ndarray[double, mode="c"] u):
        """
        Step the simulation forward one timestep.
        param np.ndarray out: where to store the system output
            NOTE: output is of form [time, output]
        param np.ndarray u: the control signal
        """
        self.thisptr.step(&out[0], &u[0])
