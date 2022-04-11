from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
import os

root = "new_1Intermediate"

for path_no in os.listdir(root):
    for wind_speed in os.listdir(os.path.join(root,path_no)):
        for wind_angle in os.listdir(os.path.join(root,path_no,wind_speed)):
            for axis in ["x","y","z"]:
                for drone in os.listdir(os.path.join(root,path_no,wind_speed,wind_angle)):
                    csv_path = os.path.join(root,path_no,wind_speed,wind_angle,drone)
                    data = pd.read_csv(csv_path)
                    plt.plot(data["time.ms"],data[f"stateEstimate.{axis}"])
                plt.xlabel(os.path.join(root,path_no,wind_speed,wind_angle)+axis)
                plt.vlines(x = 6000,ymax=3,ymin=-3)
                print(os.path.join(root,path_no,wind_speed,wind_angle)+axis)
                plt.show()

