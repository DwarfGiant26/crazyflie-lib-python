from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
import os

root = "new_2Intermediate"

for path_no in os.listdir(root):
    for wind_speed in os.listdir(os.path.join(root,path_no)):
        for wind_angle in os.listdir(os.path.join(root,path_no,wind_speed)):
            for drone in os.listdir(os.path.join(root,path_no,wind_speed,wind_angle)):
                csv_path = os.path.join(root,path_no,wind_speed,wind_angle,drone)
                data = pd.read_csv(csv_path)
                if "new_1Intermediate/path_3/wind_speed_1/wind_direction_180" in csv_path:
                    data = data[data["time.ms"] > 2000]
                    data.to_csv(csv_path)
                data = data.iloc[:,3:]
                data.to_csv(csv_path)

