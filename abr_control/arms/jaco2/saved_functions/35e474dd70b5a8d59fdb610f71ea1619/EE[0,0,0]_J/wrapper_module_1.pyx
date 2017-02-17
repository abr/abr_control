import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_1.h':
    void autofunc(double q0, double q1, double q2, double q3, double q4, double q5, double x, double y, double z, double *out_7281159987016857799)

def autofunc_c(double q0, double q1, double q2, double q3, double q4, double q5, double x, double y, double z):

    cdef np.ndarray[np.double_t, ndim=2] out_7281159987016857799 = np.empty((6,6))
    autofunc(q0, q1, q2, q3, q4, q5, x, y, z, <double*> out_7281159987016857799.data)
    return out_7281159987016857799