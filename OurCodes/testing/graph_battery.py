from matplotlib import pyplot as plt
import pandas as pd



time_data = pd.read_csv("battery1.csv", usecols = ['time.ms'])
volt_data = pd.read_csv("battery1.csv", usecols = ['pm.vbat'])
print(time_data)
volt_data

X_values = [0, 1, 2, 3, 4]
Y_values = [0, 1, 4, 9, 16]
plt.plot(x_values, y_values)
plt.show()

