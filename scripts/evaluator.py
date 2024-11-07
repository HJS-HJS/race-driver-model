import numpy as np
import csv
import rospy
from carla_msgs.msg import CarlaEgoVehicleStatus
from nav_msgs.msg import Odometry

class RaceLineEvaluatror(object):

    def __init__(self):
        
        self.track_name= "sm"
        self.e         = "000"
        self.startline = np.array([
            28.0,
            165.0,
        ])
        self.start1 = False
        self.start2 = False
        self.speed = 0
        self.lap = 4
        self.current_lap = 0
        self.timer = rospy.get_time()
        self.data = []
        
        rospy.Subscriber('/carla/ego_vehicle/odometry',       Odometry,              self.position_cb, queue_size=1)
        rospy.Subscriber('/carla/ego_vehicle/vehicle_status', CarlaEgoVehicleStatus, self.status_cb,   queue_size=1)

    def position_cb(self, msg: Odometry):
        pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

        if self.check_startline(pose) and not self.start1:
            if self.start2: 
                self.current_lap += 1
                print("Lap:", self.current_lap)
                print("Spent time:", rospy.get_time() - self.timer)
                if self.current_lap == self.lap:
                    self.file_writer(self.data)
                    rospy.signal_shutdown("Evaluate Finished")
                else:
                    self.start2 = False
            else:
                self.start1 = True
        elif not self.check_startline(pose) and self.start1:
            print('\nstart recording\n')
            if self.current_lap == 0:
                self.timer = rospy.get_time()
                self.data = []
            self.start1 = False
            self.start2 = True

        self.data.append([rospy.get_time() - self.timer, self.speed])

    def status_cb(self, msg: CarlaEgoVehicleStatus):
        self.speed = msg.velocity * 60 * 60 / 1000 #[m/s] to [km/h]

    def file_writer(self, data):
        file_path = "../evaluate_data/" + self.track_name + "_" + self.e + "_" + str(data[-1][0]) + ".csv"
        f = open(file_path, "w")
        writer = csv.writer(f)

        writer.writerows(data)
        f.close()
        print("Save data file at:", file_path)

    def check_startline(self, point):
        line_dist = np.abs(self.startline[1] - point[1])
        point_dist = np.linalg.norm(self.startline - point)
                           
        if (line_dist < 0.1) and (point_dist < 5.0):
            return True
        else: return False


if __name__ == '__main__':
    rospy.init_node('RaceLineEvaluatror')
    evaluator = RaceLineEvaluatror()
    
    rospy.spin()
