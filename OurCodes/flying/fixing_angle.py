import os
import pandas as pd

def listdir_nohidden(path):
    return [name for name in os.listdir(path) if name[0]!='.']

for root in ["new_1Intermediate","new_2Intermediate"]:
    for path_no in listdir_nohidden(root):
        for wind_speed in listdir_nohidden(os.path.join(root,path_no)):
            for wind_angle in listdir_nohidden(os.path.join(root,path_no,wind_speed)):
                if wind_angle[0] == '.': continue
                for drone in listdir_nohidden(os.path.join(root,path_no,wind_speed,wind_angle)):
                    csv_path = os.path.join(root,path_no,wind_speed,wind_angle,drone)
                    print(csv_path)
                    data = pd.read_csv(csv_path)
                    for row in range(len(data)):
                        angle = data["relative_wind_direction"][row]
                        if angle != 'None' and angle != 'WindDirection.NONE':
                            
                        if int(angle) > 360:
                            angle = 720 - angle
                        if data["global_wind_direction"][row]!= 'None' and data["global_wind_direction"][row]!= 'WindDirection.NONE':
                            if int(data["global_wind_direction"][row]) > 360:
                                print(data["global_wind_direction"][row])

