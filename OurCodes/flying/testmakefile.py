import os

os.mkdir('new_1Intermediate')
os.chdir('new_1Intermediate')
for i in range(1,6):
    os.mkdir(f'path_{i}')
    for j in range(3):
        os.mkdir(f'path_{i}/wind_speed_{j}')
        if j == 0:
            ls = ['None']
        else:
            ls = [0,90,180]
        for k in ls:
            os.mkdir(f'path_{i}/wind_speed_{j}/wind_direction_{k}')