import numpy as np
cimport numpy as np

cdef extern from "jaco2_rs485.h":
    cdef cppclass Jaco2:
        Jaco2()
        # main functions
        void Connect()
        void InitForceMode()
        void ApplyU(float u[6])
        void Disconnect()

        float pos[6]
        float vel[6]

cdef class pyJaco2: 
    cdef Jaco2* thisptr # hold a C++ instance
    def __cinit__(self):
        self.thisptr = new Jaco2()
        
    def __dealloc__(self):
        del self.thisptr
 
    def Connect(self):
        self.thisptr.Connect()
        
    def InitForceMode(self):
        self.thisptr.InitForceMode()
    
    def ApplyU(self, np.ndarray[float, mode="c"] u):
        self.thisptr.ApplyU(&u[0])
    
    def Disconnect(self):
        self.thisptr.Disconnect()
        
    def GetFeedback(self):
        return np.hstack([self.thisptr.pos, self.thisptr.vel])
