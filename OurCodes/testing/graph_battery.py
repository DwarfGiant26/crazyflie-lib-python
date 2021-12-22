from matplotlib import pyplot as plt
import pandas as pd



time_data = pd.read_csv("battery1.csv", usecols = ['time.ms'])
volt_data = pd.read_csv("battery1.csv", usecols = ['pm.vbat'])
print(time_data)
print(volt_data)

# X_values = time_data
# Y_values = [0, 1, 4, 9, 16]
plt.plot(time_data, volt_data, color='pink')
plt.xlabel("Time in ms")
plt.ylabel("Battery")

plt.show()

