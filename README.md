# race-driver-model
A module that creates a optimized global path for racing competition.

-  This is the module used in the competition. ["Autonomous Robot Racing Competitions"](https://ieeexplore.ieee.org/abstract/document/10474524)

-  It was created with reference to the paper. ["Race driver model"](https://www.sciencedirect.com/science/article/pii/S0045794908000163)

-  This code was written based on python 3.8.

### Input

 - GPS longitude, latitude point set as csv fils.
 - You can check the example files at [/path/gps/](./path/gps/)
 
### Results

| ![Image 1](./figures/track_race_cb_track.png) | ![Image 2](./figures/track_race_cb_0.png) | ![Image 3](./figures/track_race_cb_15.png) |
|-------------------------|-------------------------|-------------------------|
| <p align="center">Path creation bounds</p> | <p align="center">epsilon = 0% __(Minimum curvature path)__</p> | <p align="center">epsilon = 15%</p> |
| ![Image 4](./figures/track_race_cb_30.png) | ![Image 5](./figures/track_race_cb_60.png) | ![Image 6](./figures/track_race_cb_100.png) |
| <p align="center">epsilon = 30%</p> | <p align="center">epsilon = 60%</p> | <p align="center">epsilon = 100% __(Shortest Path)__</p> |

 - Minimum curvature path if epsilon is close to 0%
 - Shortest path if epsilon is close to 100%
