import os

os.mkdir('2Intermediate')
os.chdir('2Intermediate')
for i in range(1,6):
    os.mkdir(f'path_{i}')
    for j in range(4):
        os.mkdir(f'path_{i}/wind_speed_{j}')
        if j == 0:
            ls = ['None']
        else:
            ls = [90,180,270]
        for k in ls:
            os.mkdir(f'path_{i}/wind_speed_{j}/wind_direction_{k}')