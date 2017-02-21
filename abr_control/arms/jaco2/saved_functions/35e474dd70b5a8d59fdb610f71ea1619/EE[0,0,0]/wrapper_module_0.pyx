import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_0.h':
    void autofunc(double q0, double q1, double q2, double q3, double q4, double q5, double x, double y, double z, double *out_4814970229430216916)

def autofunc_c(double q0, double q1, double q2, double q3, double q4, double q5, double x, double y, double z):

    cdef np.ndarray[np.double_t, ndim=2] out_4814970229430216916 = np.empty((4,1))
    autofunc(q0, q1, q2, q3, q4, q5, x, y, z, <double*> out_4814970229430216916.data)
    return out_4814970229430216916