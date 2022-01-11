from matplotlib import pyplot as plt
import pandas as pd

time_dataE7 = pd.read_csv("batteryE7.csv", usecols = ['time.ms'])
volt_dataE7 = pd.read_csv("batteryE7.csv", usecols = ['pm.vbat'])
plt.plot(time_dataE7, volt_dataE7, color='pink')

time_dataE5 = pd.read_csv("battery1.csv", usecols = ['time.ms'])
volt_dataE5 = pd.read_csv("battery1.csv", usecols = ['pm.vbat'])
plt.plot(time_dataE5, volt_dataE5, color='green')

time_dataE3 = pd.read_csv("batteryE3.csv", usecols = ['time.ms'])
volt_dataE3 = pd.read_csv("batteryE3.csv", usecols = ['pm.vbat'])
plt.plot(time_dataE3, volt_dataE3, color='red')

# LINE OF BEST FIT
time_data = pd.read_csv("batteryE7.csv", usecols = ['time.ms'])
volt_data = pd.read_csv("batteryE7.csv", usecols = ['pm.vbat'])

time_data += pd.read_csv("battery1.csv", usecols = ['time.ms'])
volt_data += pd.read_csv("battery1.csv", usecols = ['pm.vbat'])

time_data += pd.read_csv("batteryE3.csv", usecols = ['time.ms'])
volt_data += pd.read_csv("batteryE3.csv", usecols = ['pm.vbat'])


# X_values = time_data
# Y_values = [0, 1, 4, 9, 16]
plt.plot(time_data/3, volt_data/3, color='blue')
# plt.plot(time_data1, volt_data1, color='red')
plt.xlabel("Time in ms")
plt.ylabel("Battery") 

plt.show()

