import numpy as np
import matplotlib.pyplot as plt
from utils.utils import *  # 유틸리티 함수 임포트
from utils.path_optimizer import shortest_path, min_curve_path, optimize  # 경로 최적화 관련 함수 임포트

# 트랙 설정
track = "cb"

# GPS 데이터를 이용해 기준 경로(ref_path), 경계 거리(lenght), 트랙 중심(center), 파일 경로(file_path) 가져오기
ref_path, lenght, center, file_path = gps_path(track)

# 트랙의 안쪽 및 바깥쪽 경계 계산
bound_in = ref_path - lenght
bound_out = ref_path + lenght

# 최소 거리 경로(Shortest Path) 및 최소 곡률 경로(Minimum Curvature Path) 계산
H_S, B_S = shortest_path(ref_path, lenght)
H_C, B_C = min_curve_path(ref_path, lenght)

# 경로 최적화를 위한 가중치(epsilon) 설정
e_set = [0.00, 0.15, 0.30, 0.60, 1.00]
# e = 0.00 → 최소 곡률 경로 사용
# e = 1.00 → 최소 거리 경로 사용

# 트랙 파일 경로 설정
_file_path = file_path + '_track'

# 경계선을 닫힌 형태로 만들기 위해 시작점을 다시 추가
bound_in = np.vstack((bound_in, bound_in[0]))
bound_out = np.vstack((bound_out, bound_out[0]))

# 여러 가중치(e)에 대해 경로 최적화 수행
for e in e_set:
    # 최소 곡률 경로(H_C, B_C)와 최소 거리 경로(H_S, B_S)를 가중치에 따라 혼합
    H = (1 - e) * H_C + e * H_S
    B = (1 - e) * B_C + e * B_S

    # 최적화 함수 실행하여 최적의 경로(alpha) 계산
    alpha = optimize(H, B)

    # 최적화된 경로 생성
    course_race = ref_path + lenght * alpha[:, np.newaxis]

    # 결과 저장을 위한 파일 경로 설정
    _file_path = file_path + '_' + str(int(e * 100))

    # 최적화된 경로를 CSV 파일로 저장
    save_path(course_race, center, _file_path + '.csv')

    # 경로를 닫힌 형태로 만들기 위해 시작점을 다시 추가
    course_race = np.vstack((course_race, course_race[0]))

    # 경로 및 경계 시각화
    plt.figure()
    plt.plot(bound_in[:, 0], bound_in[:, 1], label="bound_in", color="blue")  # 안쪽 경계선
    plt.plot(bound_out[:, 0], bound_out[:, 1], label="bound_out", color="blue")  # 바깥쪽 경계선
    plt.plot(course_race[:, 0], course_race[:, 1], label="optimized path", color="orange", linewidth=3)  # 최적화된 경로

    # 그래프 설정
    plt.grid(True)
    plt.gca().set_aspect('equal')  # 축 비율을 동일하게 설정
    plt.legend()
    plt.title("Track: " + track + ", e: " + str(int(e * 100)) + "%")  # 제목 설정

    # 결과 이미지 저장
    plt.savefig(_file_path + '.png', dpi=400)

    # 그래프 표시
    plt.show()
