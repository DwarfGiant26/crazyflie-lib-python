from matplotlib import pyplot as plt
import pandas as pd
import numpy as np

# ---------------------------------------------------------------------------
# -----------------------------------SETTINGS--------------------------------
# ---------------------------------------------------------------------------
path_no = 3
speed_no = 0
angle = 0

parent_folder = "/Users/sarahbradley/Documents/SCSLab/crazyflie-lib-python/OurCodes/flying"

dataE3 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_{speed_no}/wind_direction_None/drone_E3.csv")
dataE5 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_{speed_no}/wind_direction_None/drone_E5.csv")
dataE7 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_{speed_no}/wind_direction_None/drone_E7.csv")

dataE3_wind1 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_1/wind_direction_{angle}/drone_E3.csv")
dataE5_wind1 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_1/wind_direction_{angle}/drone_E5.csv")
dataE7_wind1 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_1/wind_direction_{angle}/drone_E7.csv")

dataE3_wind2 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_2/wind_direction_{angle}/drone_E3.csv")
dataE5_wind2 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_2/wind_direction_{angle}/drone_E5.csv")
dataE7_wind2 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_2/wind_direction_{angle}/drone_E7.csv")



# ---------------------------------------------------------------------------
# ------------------------------BATTERY MAPPING------------------------------
# ---------------------------------------------------------------------------

# Changing milliseconds to seconds.
dataE3["time.ms"] = dataE3["time.ms"].map(lambda x : x/1000)
dataE5["time.ms"] = dataE5["time.ms"].map(lambda x : x/1000)
dataE7["time.ms"] = dataE7["time.ms"].map(lambda x : x/1000)

dataE3_wind1["time.ms"] = dataE3_wind1["time.ms"].map(lambda x : x/1000)
dataE5_wind1["time.ms"] = dataE5_wind1["time.ms"].map(lambda x : x/1000)
dataE7_wind1["time.ms"] = dataE7_wind1["time.ms"].map(lambda x : x/1000)

dataE3_wind2["time.ms"] = dataE3_wind2["time.ms"].map(lambda x : x/1000)
dataE5_wind2["time.ms"] = dataE5_wind2["time.ms"].map(lambda x : x/1000)
dataE7_wind2["time.ms"] = dataE7_wind2["time.ms"].map(lambda x : x/1000)

# Graphing voltage over time.
batterygraph = dataE3.plot(x="time.ms", y="pm.vbat", color='orange')
dataE5.plot(x="time.ms", y="pm.vbat", ax = batterygraph, color='orange')
dataE7.plot(x="time.ms", y ="pm.vbat", ax= batterygraph, color='orange')

dataE3_wind1.plot(x="time.ms", y="pm.vbat", ax = batterygraph, color='blue')
dataE5_wind1.plot(x="time.ms", y="pm.vbat", ax = batterygraph, color='blue')
dataE7_wind1.plot(x="time.ms", y ="pm.vbat", ax= batterygraph, color='blue')

dataE3_wind2.plot(x="time.ms", y="pm.vbat", ax = batterygraph, color='green')
dataE5_wind2.plot(x="time.ms", y="pm.vbat", ax = batterygraph, color='green')
dataE7_wind2.plot(x="time.ms", y ="pm.vbat", ax= batterygraph, color='green')

# batterygraph.set_xlim(0, None)
# batterygraph.legend(["E3","E5","E7"])
batterygraph.set_title("Voltage Over Time")
batterygraph.set_xlabel("Time in Seconds")
batterygraph.set_ylabel("Voltage")

# Other syntax for plotting multiple graphs in one axes:
# fig = plt.figure()
# for frame in [dataE3, dataE5, dataE7]:
#     plt.plot(frame['time.ms'], frame['pm.vbat'])

plt.show()


# ---------------------------------------------------------------------------
# -------------------------------DISTANCE MAPPING----------------------------
# ---------------------------------------------------------------------------

# distancegraph = dataE3.plot(x="time.ms", y="travel_dist")
# dataE5.plot(x="time.ms", y="travel_dist", ax = distancegraph)
# dataE7.plot(x="time.ms", y="travel_dist", ax = distancegraph)
# distancegraph.legend(["E3","E5","E7"])
# distancegraph.set_title("Distance Over Time")
# distancegraph.set_xlabel("Time in Seconds")
# distancegraph.set_ylabel("Distance in Metres")
# plt.show()

# How many seconds would it take to fully charge:
# First find the number of seconds:
# np.interp(3.7, time_data, volt_data)

# ---------------------------------------------------------------------------
# ---------------------------X, Y COORDINATES MAPPING------------------------
# ---------------------------------------------------------------------------

# x_ycoord = dataE3.plot(x="stateEstimate.x", y="stateEstimate.y")
# dataE5.plot(x="stateEstimate.x", y="stateEstimate.y", ax = x_ycoord)
# dataE7.plot(x="stateEstimate.x", y="stateEstimate.y", ax = x_ycoord)
# x_ycoord.legend(["E3","E5","E7"])
# x_ycoord.set_title("Drone Flight Paths: Y over X")
# x_ycoord.set_xlabel("X Axis")
# x_ycoord.set_ylabel("Y Axis")
# plt.show()

# ---------------------------------------------------------------------------
# ----------------------------Z COORDINATES MAPPING--------------------------
# ---------------------------------------------------------------------------

# z_coord = dataE3.plot(x="time.ms", y="stateEstimate.z")
# dataE5.plot(x="time.ms", y="stateEstimate.z", ax = z_coord)
# dataE7.plot(x="time.ms", y="stateEstimate.z", ax = z_coord)
# z_coord.legend(["E3","E5","E7"])
# z_coord.set_title("Drone Flight Paths: Z Over Time")
# z_coord.set_xlabel("Time in Seconds")
# z_coord.set_ylabel("Z Axis")
# plt.show()


