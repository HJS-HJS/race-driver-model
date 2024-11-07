import copy
import numpy as np
import osqp
from scipy import sparse

def shortest_path(path, lengh):
    N = path.shape[0]

    H_S = np.zeros((N, N))
    B_S = np.zeros(N)

    _path  = np.vstack((path, path[0])) 
    _lengh = np.vstack((lengh, lengh[0])) 

    for i in range(N):
        i_1 = i
        i_2 = (i + 1) % N
        # 
        _Lx1 = _lengh[i_1,0]
        _Lx2 = _lengh[i_2,0]
        _Ly1 = _lengh[i_1,1]
        _Ly2 = _lengh[i_2,1]
        # H
        H_S[i_1,i_1] +=  _Lx1**2   + _Ly1**2
        H_S[i_2,i_1] += -_Lx1*_Lx2 + -_Ly1*_Ly2
        H_S[i_1,i_2] += -_Lx1*_Lx2 + -_Ly1*_Ly2
        H_S[i_2,i_2] +=  _Lx2**2   +  _Ly2**2
        # B
        B_S[i_1] += - 2*(_path[i_2, 0] - _path[i_1, 0]) * _Lx1 - 2*(_path[i_2, 1] - _path[i_1, 1]) * _Ly1
        B_S[i_2] += + 2*(_path[i_2, 0] - _path[i_1, 0]) * _Lx2 + 2*(_path[i_2, 1] - _path[i_1, 1]) * _Ly2
        
    return H_S, B_S

def min_curve_path(path, lengh):
    N = path.shape[0]

    H_C = np.zeros((N, N))
    B_C = np.zeros(N)
    
    _path  = np.vstack((path, path[0])) 
    _lengh = np.vstack((lengh, lengh[0])) 

    for i in range(N):
        i_0 = (i - 1) % N
        i_1 = i
        i_2 = (i + 1) % N

        # 
        _Lx0 = _lengh[i_0,0]
        _Lx1 = _lengh[i_1,0]
        _Lx2 = _lengh[i_2,0]
        _Ly0 = _lengh[i_0,1]
        _Ly1 = _lengh[i_1,1]
        _Ly2 = _lengh[i_2,1]
        
        # Central approximation
        # H 
        H_C[i_0,i_0] += (   _Lx0**2  ) * 1 + (   _Ly0**2  ) * 1
        H_C[i_0,i_1] += (-2*_Lx0*_Lx1) * 1 + (-2*_Ly0*_Ly1) * 1
        H_C[i_0,i_2] += (   _Lx0*_Lx2) * 1 + (   _Ly0*_Ly2) * 1

        H_C[i_1,i_0] += (-2*_Lx0*_Lx1) * 1 + (-2*_Ly0*_Ly1) * 1
        H_C[i_1,i_1] += ( 4*_Lx1**2  ) * 1 + ( 4*_Ly1**2  ) * 1
        H_C[i_1,i_2] += (-2*_Lx2*_Lx1) * 1 + (-2*_Ly2*_Ly1) * 1

        H_C[i_2,i_0] += (   _Lx2*_Lx0) * 1 + (   _Ly2*_Ly0) * 1
        H_C[i_2,i_1] += (-2*_Lx2*_Lx1) * 1 + (-2*_Ly2*_Ly1) * 1
        H_C[i_2,i_2] += (   _Lx2**2  ) * 1 + (   _Ly2**2  ) * 1
        # B

        B_C[i_0] += + 2 * (_path[i_2, 0] + _path[i_0, 0] -2 * _path[i_1, 0]) * _Lx0 + 2 * (_path[i_2, 1] + _path[i_0, 1] -2 * _path[i_1, 1]) * _Ly0
        B_C[i_1] += - 4 * (_path[i_2, 0] + _path[i_0, 0] -2 * _path[i_1, 0]) * _Lx1 - 4 * (_path[i_2, 1] + _path[i_0, 1] -2 * _path[i_1, 1]) * _Ly1
        B_C[i_2] += + 2 * (_path[i_2, 0] + _path[i_0, 0] -2 * _path[i_1, 0]) * _Lx2 + 2 * (_path[i_2, 1] + _path[i_0, 1] -2 * _path[i_1, 1]) * _Ly2

    return H_C, B_C

def optimize(H, B):
    N = B.shape[0]

    m = osqp.OSQP()
    m.setup(P = sparse.csc_matrix(H),
            q = B,
            A = sparse.csc_matrix(np.eye(N)),
            # l=np.zeros(N),
            l=-np.ones(N),
            u=np.ones(N),
            eps_abs = 1e-35,
            eps_rel = 1e-20,
            max_iter = 100000,
            verbose=False,
    )
    
    result = m.solve()
    return result.x
