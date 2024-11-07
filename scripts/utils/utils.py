import copy
import numpy as np
import csv

def super_ellipse_path():
    a:float = 60.0
    b:float = 72.0
    n:int = 3
    theta:float = np.pi * 2 / 18
    L:float = 3.0
    N = 50

    _theta = np.linspace(0, 2 * np.pi, N, endpoint=False) - theta
    
    _r = (np.abs(np.cos(_theta) / a)**n + np.abs(np.sin(_theta) / b)**n)**(-1/n)

    points = np.array([
        _r * np.cos(_theta),
        _r * np.sin(_theta),
    ])

    lengh_vector = np.zeros_like(points)

    for idx in range(N):
        i_0 = idx
        i_1 = (idx + 1) % N
        _vec = np.array([points[0][i_1] - points[0][i_0],
                         points[1][i_1] - points[1][i_0]])
        _vec = _vec / np.linalg.norm(_vec) * L
        lengh_vector[:,i_0] = np.array([_vec[1], -_vec[0]])
    
    return points.T, lengh_vector.T
    
def example_path():

    points = file_reader("../path/xy/example.csv")

    N:int = len(points)
    L:float = 0.5

    lengh_vector = np.zeros_like(points)
    for i in range(N - 1):
        _vec = np.array([points[i + 1][0] - points[i][0],
                         points[i + 1][1] - points[i][1]])
        if _vec[0] == 0:
            lengh_vector[i,:] = lengh_vector[i-1,:]
            continue
        _vec = _vec / np.linalg.norm(_vec) * L
        lengh_vector[i,:] = np.array([_vec[1], -_vec[0]])
    _vec = np.array([points[0][0] - points[-1][0],
                     points[0][1] - points[-1][1]])
    _vec = _vec / np.linalg.norm(_vec) * L
    lengh_vector[-1, :] = np.array([_vec[1], -_vec[0]])
    return points, lengh_vector

def gps_path(path:str="cb", min_lengh:float = 1.5):

    points_in  = file_reader("../path/gps/track_in_"  + path + ".csv")
    points_out = file_reader("../path/gps/track_out_" + path + ".csv")
    
    center, points_in, points_out = gps_to_xy(points_in, points_out)

    points_in = shorten_path(points_in, 2.5)

    N:int   = len(points_in)

    points = []
    l_vector = []

    # find left limit point
    for idx in range(N):
        i = find_closest_point(points_in[idx], points_out)
        d = find_distance(points_in[idx], points_out[i], points_out[(i + 1)%len(points_out)])
        i_0 = idx
        i_1 = (idx + 1) % N

        _vec = np.array([points_in[i_1][0] - points_in[i_0][0],
                         points_in[i_1][1] - points_in[i_0][1]])
        _vec = _vec / np.linalg.norm(_vec)
        _vec = np.array([-_vec[1], _vec[0]])
        
        d = np.linalg.norm(d) * 0.9
        if d < 0 : d = 0
        if d > 3 : d = 3
        
        points.append(points_in[i_0] - _vec * 0.5)
        l_vector.append(_vec * d)

    return np.array(points), np.array(l_vector), center, "../path/optimal_path/track_race_"  + path

def save_path(points, center, file_path):
    points = xy_to_gps(points, center)
    file_writer(file_path, points)

def file_reader(file_path:str):
    f = open(file_path, "r")
    reader = csv.reader(f)
    
    points = []
    for row in reader:
        points.append([float(row[0]), float(row[1])])

    return np.array(points)

def file_writer(file_path:str, path):

    path = interpolate_path(path, 6)

    f = open(file_path, "w")
    writer = csv.writer(f)

    writer.writerows(path)
    f.close()
    print("Save optimized file at:", file_path)
    return

def gps_to_xy(gps_in, gps_out):
    
    center = np.mean(gps_in, axis=0)

    cos = np.cos(center[0] * np.pi / 180)
    const =  np.pi * 6378.135 / 180 * 1000

    path_in  = gps_in  - center
    path_out = gps_out - center

    path_in *= const
    path_out*= const

    path_in[:,1] *= cos
    path_out[:,1] *= cos

    return center, path_in, path_out

def xy_to_gps(path, center):
    
    cos = np.cos(center[0] * np.pi / 180)
    const =  np.pi * 6378.135 / 180 * 1000

    _path = copy.deepcopy(path)
    _path[:,1] /= cos
    _path      /= const

    return _path + center

def find_closest_point(point, path):
    return np.argmin(np.linalg.norm(path - point, axis=1))

def find_distance(point, point1, point2):
    vec1 = point  - point1
    vec2 = point2 - point1
    
    # Convert vec2 as unit vector
    vec2 = vec2 / np.linalg.norm(vec2)

    return vec1 - (vec1 @ vec2) * vec2

def interpolate_path(path, n:int = 2):
    _path  = np.vstack((path, path[0]))
    _path_interpolated = []
    for i in range(len(_path) - 2):
        for _i in range(n):
            _path_interpolated.append(_path[i] + _i * (_path[i + 1] - _path[i]) / n)
    return  np.array(_path_interpolated)

def shorten_path(path, unit_lenth:float=2):
    _d_lenght = 0.005
    _interpolated_path = []
    N = len(path)

    for i in range(len(path)):
        i_s = i
        i_n = (i + 1) % N
        n = int(np.linalg.norm(path[i_s] - path[i_n]) / _d_lenght)
        for _i in range(n):
            _interpolated_path.append(path[i_s] + _i * (path[i_n] - path[i_s]) / n)

    _interpolated_path = np.array(_interpolated_path)

    while True:
        shorten_path = _interpolated_path[::int(unit_lenth / _d_lenght)]
        _lengh1 = np.linalg.norm(shorten_path[0] - shorten_path[1])
        _lengh2 = np.linalg.norm(shorten_path[-1] - shorten_path[0])
        if _lengh1 * 0.96 > _lengh2:
            unit_lenth += _d_lenght/3
        else: 
            return shorten_path
