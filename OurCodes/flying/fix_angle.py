import os
import pandas as pd

root = "1Intermediate"

for path_no in os.listdir(root):
    for wind_speed in os.listdir(os.path.join(root,path_no)):
        for wind_angle in os.listdir(os.path.join(root,path_no,wind_speed)):
            for drone in os.listdir(os.path.join(root,path_no,wind_speed,wind_angle)):

                # read and change angle
                # angle has to be substracted by 90 degrees
                csv_path = os.path.join(root,path_no,wind_speed,wind_angle,drone)
                print(csv_path)
                data = pd.read_csv(csv_path)
                for row in range(len(data)):
                    angle = data.iloc[row]["global_wind_direction"]
                    if angle != 'None' and angle != 'WindDirection.NONE':
                        angle = int(angle)
                        new_angle = angle - 90
                        if new_angle < 0:
                            new_angle = 360 - new_angle
                    else:
                        new_angle = 'None'
                    data.iloc[row]["global_wind_direction"] = new_angle
                    new_global_angle = new_angle

                    angle = data.iloc[row]["relative_wind_direction"]
                    if angle != 'None' and angle != 'WindDirection.NONE':
                        angle = int(angle)
                        new_angle = angle - 90
                        if new_angle < 0:
                            new_angle = 360 - new_angle
                    else:
                        new_angle = 'None'
                    data.iloc[row]["relative_wind_direction"] = new_angle

                #fix z axis below zero assuming non uniform 


                # write down to file
                data.to_csv(os.path.join("new_"+root,path_no,wind_speed,"wind_direction_"+str(new_global_angle),drone))
            
                    