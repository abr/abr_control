import numpy as np
cimport numpy as np

cdef extern from "jaco2_rs485.h":
    cdef cppclass Jaco2:
        Jaco2()
        # main functions
        void Connect()
        void InitForceMode(float expected_torques[6])
        void ApplyQ(float target_q[6])
        void ApplyU(float u[6])
        void Disconnect()

        float pos[6]
        float vel[6]


# TODO: make position control available

cdef class pyJaco2: 
    cdef Jaco2* thisptr # hold a C++ instance
    def __cinit__(self):
        self.thisptr = new Jaco2()
        
    def __dealloc__(self):
        del self.thisptr
 
    def Connect(self):
        self.thisptr.Connect()
        
    def InitForceMode(self, np.ndarray[float, mode="c"] expected_torques):
        self.thisptr.InitForceMode(&expected_torques[0])
    
    def ApplyQ(self, np.ndarray[float, mode="c"] target_q):
        self.thisptr.ApplyQ(&target_q[0])
    
    def ApplyU(self, np.ndarray[float, mode="c"] u):
        self.thisptr.ApplyU(&u[0])
    
    def Disconnect(self):
        self.thisptr.Disconnect()
        
    def GetFeedback(self):
        feedback = {'q': self.thisptr.pos,
                    'dq': self.thisptr.vel}
        return feedback
