import logging
import time
import csv
import struct
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

import matplotlib.pyplot as plt

URI1 = 'radio://0/60/2M/E7E7E7E7E7'
URI2 = 'radio://0/60/2M/E7E7E7E7E5'
URI3 = 'radio://0/60/2M/E7E7E7E7E3'
DEFAULT_HEIGHT = 0.1
SAMPLE_PERIOD_MS = 10

is_deck_attached = False

logging.basicConfig(level=logging.ERROR)

# Missing battery percentage & current, drone speed
# Can only have 6 parameters at a time
log_parameters = [
    ('stateEstimate.x', 'float'),
    ('stateEstimate.y', 'float'),
    ('stateEstimate.z', 'float'),
    ('stabilizer.roll', 'float'),
    ('stabilizer.pitch', 'float'),
    ('stabilizer.yaw', 'float'),
    ('pm.vbat', 'FP16')
]

log_history = [[],[],[]]
log_cycles = 0
bat_volt = [-1,-1,-1] #battery voltage for uri1,uri2,and uri3 respectively


def init_log_history():
    pass


def write_log_history():
    for i in range(3):
        with open(f'multiple1_drone_{i}.csv', mode='w') as csv_file:
            fieldnames = ['time.ms'] + [param[0] for param in log_parameters]
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            writer.writeheader()
            for item in log_history[i]:
                writer.writerow(item)


def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        # time.sleep(1)
        mc.stop()


def logconf_callback_1(timestamp, data, logconf):
    global log_history, log_cycles
    data['time.ms'] = timestamp
    bat_volt[0] = float(data['pm.vbat'])
    
    curr_coor = (data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'])
    last_coor = (log_history[0][-1]['stateEstimate.x'], log_history[0][-1]['stateEstimate.y'], log_history[0][-1]['stateEstimate.z'])
    travel_dist[0] += distance(curr_coor,last_coor)
    data['travel_dist'] = travel_dist[0]

    log_history[0].append(data)
    log_cycles += 1

def logconf_callback_2(timestamp, data, logconf):
    global log_history, log_cycles
    data['time.ms'] = timestamp
    bat_volt[1] = float(data['pm.vbat'])
    
    curr_coor = (data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'])
    last_coor = (log_history[1][-1]['stateEstimate.x'], log_history[1][-1]['stateEstimate.y'], log_history[1][-1]['stateEstimate.z'])
    travel_dist[1] += distance(curr_coor,last_coor)
    data['travel_dist'] = travel_dist[1]

    log_history[1].append(data)
    log_cycles += 1

def logconf_callback_3(timestamp, data, logconf):
    global log_history, log_cycles
    data['time.ms'] = timestamp
    bat_volt[2] = float(data['pm.vbat'])

    curr_coor = (data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'])
    last_coor = (log_history[2][-1]['stateEstimate.x'], log_history[2][-1]['stateEstimate.y'], log_history[2][-1]['stateEstimate.z'])
    travel_dist[2] += distance(curr_coor,last_coor)
    data['travel_dist'] = travel_dist[2]
    
    log_history[2].append(data)
    log_cycles += 1


def param_deck_flow(name, value_str):
    value = int(value_str)
    global is_deck_attached
    if value:
        is_deck_attached = True
        print('Deck is attached!')
    else:
        is_deck_attached = False
        print('Deck is NOT attached!')

def distance(a,b):
    import math
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

if __name__ == '__main__':


    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected to E7")
        with SyncCrazyflie(URI2, cf=Crazyflie(rw_cache='./cache')) as scf2:
            print("Connected to E5")
            with SyncCrazyflie(URI3, cf=Crazyflie(rw_cache='./cache')) as scf3:
                print("Connected to E3")

                print("reached sync")
                scf.cf.param.add_update_callback(
                    group="deck", name="bcLighthouse4", cb=param_deck_flow)
                time.sleep(1)

                scf2.cf.param.add_update_callback(
                    group="deck", name="bcLighthouse4", cb=param_deck_flow)
                time.sleep(1)

                scf3.cf.param.add_update_callback(
                    group="deck", name="bcLighthouse4", cb=param_deck_flow)
                time.sleep(1)


                logconf1 = LogConfig(name='Parameters', period_in_ms=SAMPLE_PERIOD_MS)
                for param in log_parameters:
                    logconf1.add_variable(param[0], param[1])
                logconf2 = LogConfig(name='Parameters', period_in_ms=SAMPLE_PERIOD_MS)
                for param in log_parameters:
                    logconf2.add_variable(param[0], param[1])
                logconf3 = LogConfig(name='Parameters', period_in_ms=SAMPLE_PERIOD_MS)
                for param in log_parameters:
                    logconf3.add_variable(param[0], param[1])

                scf.cf.log.add_config(logconf1)
                scf2.cf.log.add_config(logconf2)
                scf3.cf.log.add_config(logconf3)
                logconf1.data_received_cb.add_callback(logconf_callback_1)
                logconf2.data_received_cb.add_callback(logconf_callback_2)
                logconf3.data_received_cb.add_callback(logconf_callback_3)

                #if is_deck_attached:
                logconf1.start()
                logconf2.start()
                logconf3.start()

                cf = scf.cf
                cf2 = scf2.cf
                cf3 = scf3.cf

                cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                cf.param.set_value('kalman.resetEstimation', '0')
                time.sleep(2)
                cf2.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                cf2.param.set_value('kalman.resetEstimation', '0')
                time.sleep(2)
                cf3.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                cf3.param.set_value('kalman.resetEstimation', '0')
                time.sleep(2)
                print("set kalman values")

                #specify start, intermediate, and destination
                start = [(0,0,0),(0,0,0),(0,0,0)] #for scf, scf2, and scf3 respectively
                intermediate = (0,0,0)
                destination = [(0,0,0),(0,0,0),(0,0,0)]
                travel_dist = [0,0,0]
                
                upper_bat_thresh = 4.15 #battery percentage in which we stop charging cause we consider it to be fully charged
                
                #distance from drone to the helipads in the top of the building when the drone is first hovering in the source node and when it first arrive in the other node
                hi_relative_height = 0.2 
                #distance from drone to the helipads in the top of the building when the drone is trying to land(has to be a small number so that drone does not bounce)
                lo_relative_height = 0.05

                drones = [cf,cf2,cf3]
                #find the drone closest to the intermediate to go first
                dist = []
                #calculate the distance between each drone to intermediate
                for i in range(3):
                    dist.append((distance(start[i],intermediate),drones[i],i))
                #sort based on the distance
                dist.sort()
                #now the dist is list of sorted (distance from start to intermediate, cf1/2/3, drone id) that is sorted based out of distance from starting position of the drone to intermediate node
                #drone id -> 0 for uri1, 1 for uri2, and 2 for uri3

                #renaming to make things easier to read
                first_drone = dist[0][1]
                second_drone = dist[1][1]
                third_drone = dist[2][1]
                first_drone_id = dist[0][2]
                second_drone_id = dist[1][2]
                third_drone_id = dist[2][2]

                #first drone go to intermediate
                #hover
                for y in range(30):
                    first_drone.commander.send_position_setpoint(start[first_drone_id][0], start[first_drone_id][1], start[first_drone_id][2]+hi_relative_height, 0)
                    time.sleep(0.1)
                #go
                for y in range(30):
                    first_drone.commander.send_position_setpoint(intermediate[0], intermediate[1], intermediate[2]+hi_relative_height, 0)
                    time.sleep(0.1)
                #slowly landing
                for y in range(30):
                    first_drone.commander.send_position_setpoint(intermediate[0], intermediate[1], intermediate[2]+lo_relative_height, 0)
                    time.sleep(0.1)

                #wait for first drone to charge
                while bat_volt[first_drone_id] < upper_bat_thresh:
                    time.sleep(0.1)

                #first drone go to destination, and second drone go to intermediate
                #hover
                for y in range(30):
                    first_drone.commander.send_position_setpoint(intermediate[0], intermediate[1], intermediate[2]+hi_relative_height, 0)
                    second_drone.commander.send_position_setpoint(start[first_drone_id][0], start[first_drone_id][1], start[first_drone_id][2]+hi_relative_height, 0)
                    time.sleep(0.1)
                #go
                for y in range(30):
                    first_drone.commander.send_position_setpoint(destination[first_drone_id][0], destination[first_drone_id][1], destination[first_drone_id][2]+hi_relative_height, 0)
                    second_drone.commander.send_position_setpoint(intermediate[0], intermediate[1], intermediate[2]+hi_relative_height, 0)
                    time.sleep(0.1)
                #slowly landing
                for y in range(30):
                    first_drone.commander.send_position_setpoint(destination[first_drone_id][0], destination[first_drone_id][1], destination[first_drone_id][2]+lo_relative_height, 0)
                    second_drone.commander.send_position_setpoint(intermediate[0], intermediate[1], intermediate[2]+lo_relative_height, 0)
                    time.sleep(0.1)

                #wait for second drone to charge
                while bat_volt[second_drone_id] < upper_bat_thresh:
                    time.sleep(0.1)

                #second drone go to destination, and third drone go to intermediate
                #hover
                for y in range(30):
                    second_drone.commander.send_position_setpoint(intermediate[0], intermediate[1], intermediate[2]+hi_relative_height, 0)
                    third_drone.commander.send_position_setpoint(start[third_drone_id][0], start[third_drone_id][1], start[third_drone_id][2]+hi_relative_height, 0)
                    time.sleep(0.1)
                #go
                for y in range(30):
                    second_drone.commander.send_position_setpoint(destination[second_drone_id][0], destination[second_drone_id][1], destination[second_drone_id][2]+hi_relative_height, 0)
                    third_drone.commander.send_position_setpoint(intermediate[0], intermediate[1], intermediate[2]+hi_relative_height, 0)
                    time.sleep(0.1)
                #slowly landing
                for y in range(30):
                    second_drone.commander.send_position_setpoint(destination[second_drone_id][0], destination[second_drone_id][1], destination[second_drone_id][2]+lo_relative_height, 0)
                    third_drone.commander.send_position_setpoint(intermediate[0], intermediate[1], intermediate[2]+lo_relative_height, 0)
                    time.sleep(0.1)
                
                #wait for third drone to charge
                while bat_volt[third_drone_id] < upper_bat_thresh:
                    time.sleep(0.1)

                #third drone go to destination
                #hover
                for y in range(30):
                    third_drone.commander.send_position_setpoint(intermediate[0], intermediate[1], intermediate[2]+hi_relative_height, 0)
                    time.sleep(0.1)
                #go
                for y in range(30):
                    third_drone.commander.send_position_setpoint(destination[third_drone_id][0], destination[third_drone_id][1], destination[third_drone_id][2]+hi_relative_height, 0)
                    time.sleep(0.1)
                #slowly landing
                for y in range(30):
                    third_drone.commander.send_position_setpoint(destination[third_drone_id][0], destination[third_drone_id][1], destination[third_drone_id][2]+lo_relative_height, 0)
                    time.sleep(0.1)

                cf.commander.send_stop_setpoint()
                cf2.commander.send_stop_setpoint()
                cf3.commander.send_stop_setpoint()

                write_log_history()
                logconf1.stop()
                logconf2.stop()
                logconf3.stop()

                