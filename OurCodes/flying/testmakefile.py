import os

for i in range(1,3):
    os.mkdir(f'cleaned_{i}Intermediate')
    os.chdir(f'cleaned_{i}Intermediate')
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
    os.chdir("..")