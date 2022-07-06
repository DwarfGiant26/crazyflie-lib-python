from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
from scipy.interpolate import make_interp_spline, BSpline

# ---------------------------------------------------------------------------
# -----------------------------------SETTINGS--------------------------------
# ---------------------------------------------------------------------------
path_no = 3
speed_no = 1
angle = 90

parent_folder = "/Users/sarahbradley/Documents/SCSLab/crazyflie-lib-python/OurCodes/flying"
parent_folder = "/home/alan/drone/Ours/OurCodes/flying"

dataE3 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_{speed_no}/wind_direction_{angle}/drone_E3.csv")
dataE5 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_{speed_no}/wind_direction_{angle}/drone_E5.csv")
dataE7 = pd.read_csv(f"{parent_folder}/new_1Intermediate/path_{path_no}/wind_speed_{speed_no}/wind_direction_{angle}/drone_E7.csv")

# ---------------------------------------------------------------------------
# ------------------------------CHARGING OVER TIME---------------------------
# ---------------------------------------------------------------------------
# Graph showing the charging times of batteries
# the required files are in testing.
# Will need to change these into panda dataframes and plot that way.

# chargingE7 = pd.read_csv(f"/Users/sarahbradley/Documents/SCSLab/crazyflie-lib-python/OurCodes/testing/batteryE7.csv")
# chargingE5 = pd.read_csv(f"/Users/sarahbradley/Documents/SCSLab/crazyflie-lib-python/OurCodes/testing/battery1.csv")
# chargingE3 = pd.read_csv(f"/Users/sarahbradley/Documents/SCSLab/crazyflie-lib-python/OurCodes/testing/batteryE3.csv")

# # Convert milliseconds to seconds:
# chargingE3["time.ms"] = chargingE3["time.ms"].map(lambda x : x/1000)
# chargingE5["time.ms"] = chargingE5["time.ms"].map(lambda x : x/1000)
# chargingE7["time.ms"] = chargingE7["time.ms"].map(lambda x : x/1000)

# # Plot three drone charging curves:
# charginggraph = chargingE3.plot(x="time.ms", y="pm.vbat")
# chargingE5.plot(x="time.ms", y="pm.vbat", ax = charginggraph)
# chargingE7.plot(x="time.ms", y ="pm.vbat", ax= charginggraph)

# # Line of best fit from three charging curves:
# aggregate_avg = {"pm.vbat": (chargingE3["pm.vbat"] + chargingE5["pm.vbat"] + chargingE7["pm.vbat"]).map(lambda x : x/3), 
#             "time.ms": (chargingE7["time.ms"]+chargingE3["time.ms"]+chargingE5["time.ms"]).map(lambda x : x/3)} 
# avg_charge_df = pd.DataFrame(aggregate_avg)
# avg_charge_df.plot(x="time.ms", y="pm.vbat", ax = charginggraph)
# # batterygraph.set_xlim(0, None)

# # Formatting:
# charginggraph.legend(["E3","E5","E7","Aggregate Average"])
# charginggraph.set_title("Charging Curve")
# charginggraph.set_xlabel("Time in Seconds")
# charginggraph.set_ylabel("Voltage")


# ---------------------------------------------------------------------------
# ------------------------------BATTERY MAPPING------------------------------
# ---------------------------------------------------------------------------

# Changing milliseconds to seconds.
dataE3["time.ms"] = dataE3["time.ms"].map(lambda x : x/1000)
dataE5["time.ms"] = dataE5["time.ms"].map(lambda x : x/1000)
dataE7["time.ms"] = dataE7["time.ms"].map(lambda x : x/1000)

# Graphing voltage over time.
batterygraph = dataE3.plot(x="time.ms", y="pm.vbat")
dataE5.plot(x="time.ms", y="pm.vbat", ax = batterygraph)
dataE7.plot(x="time.ms", y ="pm.vbat", ax= batterygraph)
# batterygraph.set_xlim(0, None)
batterygraph.legend(["E3","E5","E7"])
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

distancegraph = dataE3.plot(x="time.ms", y="travel_dist")
dataE5.plot(x="time.ms", y="travel_dist", ax = distancegraph)
dataE7.plot(x="time.ms", y="travel_dist", ax = distancegraph)
distancegraph.legend(["E3","E5","E7"])
distancegraph.set_title("Distance Over Time")
distancegraph.set_xlabel("Time in Seconds")
distancegraph.set_ylabel("Distance in Metres")
plt.show()

# How many seconds would it take to fully charge:
# First find the number of seconds:
# np.interp(3.7, time_data, volt_data)

# ---------------------------------------------------------------------------
# ---------------------------X, Y COORDINATES MAPPING------------------------
# ---------------------------------------------------------------------------

x_ycoord = dataE3.plot(x="stateEstimate.x", y="stateEstimate.y")
dataE5.plot(x="stateEstimate.x", y="stateEstimate.y", ax = x_ycoord)
dataE7.plot(x="stateEstimate.x", y="stateEstimate.y", ax = x_ycoord)
x_ycoord.legend(["E3","E5","E7"])
x_ycoord.set_title("Drone Flight Paths: Y over X")
x_ycoord.set_xlabel("X Axis")
x_ycoord.set_ylabel("Y Axis")
plt.show()

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


distance_voltage = dataE3.plot(x="travel_dist", y="pm.vbat")
# dataE5.plot(x="travel_dist", y="pm.vbat", ax = waiting_time)
# dataE7.plot(x="travel_dist", y="pm.vbat", ax = waiting_time)
# waiting_time.legend(["E3","E5","E7"])
distance_voltage.set_title("Voltage over Distance")
distance_voltage.set_xlabel("Distance")
distance_voltage.set_ylabel("Voltage")


