import os

os.mkdir('1Intermediate')
os.chdir('1Intermediate')
for i in range(1,16):
    os.mkdir(f'path_{i}')
    for j in range(4):
        os.mkdir(f'path_{i}/wind_speed_{j}')
        for k in ['None',0,90,270]:
            os.mkdir(f'path_{i}/wind_speed_{j}/wind_direction_{k}')