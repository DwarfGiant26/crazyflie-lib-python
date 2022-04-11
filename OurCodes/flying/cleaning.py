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
                    angle = data["global_wind_direction"][row]
                    if angle != 'None' and angle != 'WindDirection.NONE':
                        angle = int(angle)
                        new_angle = angle - 90
                        if new_angle < 0:
                            new_angle = 360 - new_angle
                    else:
                        new_angle = 'None'
                    data["global_wind_direction"][row] = new_angle
                    new_global_angle = new_angle

                    angle = data["relative_wind_direction"][row]
                    if angle != 'None' and angle != 'WindDirection.NONE':
                        angle = int(angle)
                        new_angle = angle - 90
                        if new_angle < 0:
                            new_angle = 360 - new_angle
                    else:
                        new_angle = 'None'
                    data["relative_wind_direction"][row] = new_angle

                #fix z axis below zero assuming non uniform error across the data, and make it 0 if it is negative
                for row in range(len(data)):
                    data["stateEstimate.z"][row] = max(data["stateEstimate.z"][row],0)


                # write down to file
                data.to_csv(os.path.join("new_"+root,path_no,wind_speed,"wind_direction_"+str(new_global_angle),drone))
            
                    