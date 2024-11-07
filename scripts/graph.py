import os
import glob

import numpy as np
import csv
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

def file_reader(file_path:str):
    f = open(file_path, "r")
    reader = csv.reader(f)
    
    points = []
    for row in reader:
        points.append([float(row[0]), float(row[1])])

    return np.array(points)

label = ["0",
         "15",
         "30",
         "60",
         "100",
         "in",
         ]

file_path = "../evaluate_data"
csv_files = glob.glob(os.path.join(file_path, 'cb*.csv'))

csv_files.sort(key=lambda x: (os.path.basename(x).split('_')[1]))

data = []

for file in csv_files:
    data.append(file_reader(file))

data = np.array(data)

plt.figure()

jump  = 100
width = 5
for i in range(len(data)):
    _, unique_indices = np.unique(data[i][:, 0], return_index=True)
    _data = data[i][unique_indices, :]

    plt.plot(_data[:,0][::jump], _data[:,1][::jump], marker='o', label='e '+str(label[i]), linewidth=width)

plt.xlabel('Time')
plt.ylabel('Speed (km/h)')
plt.title('Hourly Speed Graph')
plt.legend()
plt.grid(True)
plt.show()