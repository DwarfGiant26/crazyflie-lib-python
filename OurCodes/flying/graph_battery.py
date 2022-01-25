from matplotlib import pyplot as plt
import pandas as pd
import numpy as np

time_dataE3 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E3.csv", usecols = ['time.ms'])
volt_dataE3 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E3.csv", usecols = ['pm.vbat'])
dist_dataE3 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E3.csv", usecols = ['travel_dist'])
x_E3 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E3.csv", usecols = ['stateEstimate.x'])
y_E3 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E3.csv", usecols = ['stateEstimate.y'])
z_E3 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E3.csv", usecols = ['stateEstimate.z'])

time_dataE5 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E5.csv", usecols = ['time.ms'])
volt_dataE5 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E5.csv", usecols = ['pm.vbat'])
dist_dataE5 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E5.csv", usecols = ['travel_dist'])
x_E5 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E5.csv", usecols = ['stateEstimate.x'])
y_E5 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E5.csv", usecols = ['stateEstimate.y'])
z_E5 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E5.csv", usecols = ['stateEstimate.z'])


time_dataE7 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E7.csv", usecols = ['time.ms'])
volt_dataE7 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E7.csv", usecols = ['pm.vbat'])
dist_dataE7 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E7.csv", usecols = ['travel_dist'])
rel_dataE7 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E7.csv", usecols = ['relative_wind_direction'])
x_E7 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E7.csv", usecols = ['stateEstimate.x'])
y_E7 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E7.csv", usecols = ['stateEstimate.y'])
z_E7 = pd.read_csv("/home/alan/drone/Ours/OurCodes/flying/1Intermediate/path_1/wind_speed_0/wind_direction_None/drone_E7.csv", usecols = ['stateEstimate.z'])


# LINE OF BEST FIT
# time_data = pd.read_csv("batteryE7.csv", usecols = ['time.ms'])
# volt_data = pd.read_csv("batteryE7.csv", usecols = ['pm.vbat'])

# time_data += pd.read_csv("battery1.csv", usecols = ['time.ms'])
# volt_data += pd.read_csv("battery1.csv", usecols = ['pm.vbat'])

# time_data += pd.read_csv("batteryE3.csv", usecols = ['time.ms'])
# volt_data += pd.read_csv("batteryE3.csv", usecols = ['pm.vbat'])


# X_values = time_data
# Y_values = [0, 1, 4, 9, 16]

# BATTERY GRAPHING
# plt.plot(time_dataE5/1000, volt_dataE5, color='blue')
# plt.plot(time_dataE7/1000, volt_dataE7, color='pink')
# plt.plot(time_dataE3/1000, volt_dataE3, color='green')
# plt.plot([time_dataE3.iloc[0]/1000,time_dataE3.iloc[-1]/1000], [4.15,4.15], color='red')

# DISTNACE GRAPHING:

plt.plot(time_dataE3/1000, dist_dataE3, color='red')
plt.plot(time_dataE5/1000, dist_dataE5, color='green')
plt.plot(time_dataE7/1000, dist_dataE7, color='blue')
plt.xlabel("Time in Seconds")
plt.ylabel("Distance") 

# How many seconds would it take to fully charge:
# First find the number of seconds:
# np.interp(3.7, time_data, volt_data)

# X, Y COORDINATES MAPPING
# plt.plot(x_E3, y_E3, color='red')
# plt.plot(x_E5, y_E5, color='green')
# plt.plot(x_E7, y_E7, color='blue')
# plt.xlabel("X axis")
# plt.ylabel("Y axis")

# plt.plot(time_dataE3/1000, z_E3, color='red')
# plt.plot(time_dataE5/1000, z_E5, color='green')
# plt.plot(time_dataE7/1000, z_E7, color='blue')
# plt.xlabel("Time")
# plt.ylabel("Z axis")

plt.show()

