import copy
import numpy as np
import osqp
from scipy import sparse

def shortest_path(path, lengh):
    """
    최소 거리 경로(Shortest Path)를 계산하는 함수

    입력:
        - path: 기준 경로 (N x 2 행렬, 각 행은 (x, y) 좌표)
        - lengh: 경계 거리 (N x 2 행렬, 각 행은 (dx, dy))

    출력:
        - H_S: 최소 거리 경로의 Hessian 행렬 (N x N)
        - B_S: 최소 거리 경로의 선형 항 벡터 (N,)
    """
    N = path.shape[0]

    H_S = np.zeros((N, N))  # Hessian 행렬 초기화
    B_S = np.zeros(N)  # 선형 항 벡터 초기화

    # 경계를 닫힌 곡선으로 만들기 위해 첫 번째 점을 다시 추가
    _path  = np.vstack((path, path[0])) 
    _lengh = np.vstack((lengh, lengh[0])) 

    # 최소 거리 경로 계산
    for i in range(N):
        i_1 = i
        i_2 = (i + 1) % N

        # 현재 및 다음 지점의 경계 거리
        _Lx1, _Ly1 = _lengh[i_1]
        _Lx2, _Ly2 = _lengh[i_2]

        # Hessian 행렬 업데이트
        H_S[i_1, i_1] +=  _Lx1**2   + _Ly1**2
        H_S[i_2, i_1] += -_Lx1*_Lx2 + -_Ly1*_Ly2
        H_S[i_1, i_2] += -_Lx1*_Lx2 + -_Ly1*_Ly2
        H_S[i_2, i_2] +=  _Lx2**2   +  _Ly2**2

        # 선형 항 벡터 업데이트
        B_S[i_1] += -2 * (_path[i_2, 0] - _path[i_1, 0]) * _Lx1 - 2 * (_path[i_2, 1] - _path[i_1, 1]) * _Ly1
        B_S[i_2] +=  2 * (_path[i_2, 0] - _path[i_1, 0]) * _Lx2 + 2 * (_path[i_2, 1] - _path[i_1, 1]) * _Ly2
        
    return H_S, B_S

def min_curve_path(path, lengh):
    """
    최소 곡률 경로(Minimum Curvature Path)를 계산하는 함수

    입력:
        - path: 기준 경로 (N x 2 행렬, 각 행은 (x, y) 좌표)
        - lengh: 경계 거리 (N x 2 행렬, 각 행은 (dx, dy))

    출력:
        - H_C: 최소 곡률 경로의 Hessian 행렬 (N x N)
        - B_C: 최소 곡률 경로의 선형 항 벡터 (N,)
    """
    N = path.shape[0]

    H_C = np.zeros((N, N))  # Hessian 행렬 초기화
    B_C = np.zeros(N)  # 선형 항 벡터 초기화
    
    # 경계를 닫힌 곡선으로 만들기 위해 첫 번째 점을 다시 추가
    _path  = np.vstack((path, path[0])) 
    _lengh = np.vstack((lengh, lengh[0])) 

    # 최소 곡률 경로 계산
    for i in range(N):
        i_0 = (i - 1) % N
        i_1 = i
        i_2 = (i + 1) % N

        # 이전, 현재, 다음 지점의 경계 거리
        _Lx0, _Ly0 = _lengh[i_0]
        _Lx1, _Ly1 = _lengh[i_1]
        _Lx2, _Ly2 = _lengh[i_2]

        # Hessian 행렬 업데이트 (중앙 차분 근사)
        H_C[i_0, i_0] += (   _Lx0**2  ) + (   _Ly0**2  )
        H_C[i_0, i_1] += (-2*_Lx0*_Lx1) + (-2*_Ly0*_Ly1)
        H_C[i_0, i_2] += (   _Lx0*_Lx2) + (   _Ly0*_Ly2)

        H_C[i_1, i_0] += (-2*_Lx0*_Lx1) + (-2*_Ly0*_Ly1)
        H_C[i_1, i_1] += ( 4*_Lx1**2  ) + ( 4*_Ly1**2  )
        H_C[i_1, i_2] += (-2*_Lx2*_Lx1) + (-2*_Ly2*_Ly1)

        H_C[i_2, i_0] += (   _Lx2*_Lx0) + (   _Ly2*_Ly0)
        H_C[i_2, i_1] += (-2*_Lx2*_Lx1) + (-2*_Ly2*_Ly1)
        H_C[i_2, i_2] += (   _Lx2**2  ) + (   _Ly2**2  )

        # 선형 항 벡터 업데이트
        B_C[i_0] += + 2 * (_path[i_2, 0] + _path[i_0, 0] - 2 * _path[i_1, 0]) * _Lx0 + 2 * (_path[i_2, 1] + _path[i_0, 1] - 2 * _path[i_1, 1]) * _Ly0
        B_C[i_1] += - 4 * (_path[i_2, 0] + _path[i_0, 0] - 2 * _path[i_1, 0]) * _Lx1 - 4 * (_path[i_2, 1] + _path[i_0, 1] - 2 * _path[i_1, 1]) * _Ly1
        B_C[i_2] += + 2 * (_path[i_2, 0] + _path[i_0, 0] - 2 * _path[i_1, 0]) * _Lx2 + 2 * (_path[i_2, 1] + _path[i_0, 1] - 2 * _path[i_1, 1]) * _Ly2

    return H_C, B_C

def optimize(H, B):
    """
    최적화 문제를 풀어 최적의 alpha 값을 계산하는 함수

    입력:
        - H: Hessian 행렬 (N x N)
        - B: 선형 항 벡터 (N,)

    출력:
        - alpha: 최적화된 결과 벡터 (N,)
    """
    N = B.shape[0]

    # OSQP 최적화 문제 생성
    m = osqp.OSQP()
    m.setup(P=sparse.csc_matrix(H),  # Hessian 행렬 (P)
            q=B,  # 선형 항 벡터 (q)
            A=sparse.csc_matrix(np.eye(N)),  # 제약 조건 행렬 (A)
            l=-np.ones(N),  # 하한 제약 조건 (l)
            u=np.ones(N),  # 상한 제약 조건 (u)
            eps_abs=1e-35,  # 절대 오차 허용치
            eps_rel=1e-20,  # 상대 오차 허용치
            max_iter=100000,  # 최대 반복 횟수
            verbose=False,  # 출력 비활성화
    )
    
    # 최적화 문제 풀기
    result = m.solve()

    # 최적화된 alpha 값 반환
    return result.x
