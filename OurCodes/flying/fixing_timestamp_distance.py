from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
import os
import math

root = "new_1Intermediate"


for path_no in os.listdir(root):
    for wind_speed in os.listdir(os.path.join(root,path_no)):
        for wind_angle in os.listdir(os.path.join(root,path_no,wind_speed)):
            for drone in os.listdir(os.path.join(root,path_no,wind_speed,wind_angle)):
                csv_path = os.path.join(root,path_no,wind_speed,wind_angle,drone)
                data = pd.read_csv(csv_path)
                if "new_1Intermediate/path_3/wind_speed_1/wind_direction_180" in csv_path:
                    first_timestamp = data["time.ms"].iloc[0]
                    for row in range(len(data)):
                        data["time.ms"][row] -= first_timestamp
                        if row == 0:
                            data["travel_dist"][row] = 0
                        else:
                            data["travel_dist"][row] = data["travel_dist"][row-1] + math.sqrt((data["stateEstimate.x"][row]-data["stateEstimate.x"][row-1])**2 + (data["stateEstimate.y"][row]-data["stateEstimate.y"][row-1])**2 + (data["stateEstimate.z"][row]-data["stateEstimate.z"][row-1])**2)
                    print(csv_path)
                    data.to_csv(csv_path) 