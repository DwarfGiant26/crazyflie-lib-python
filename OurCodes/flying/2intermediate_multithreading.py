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
import math
import matplotlib.pyplot as plt
import os
import threading

URI1 = 'radio://0/60/2M/E7E7E7E7E3'
URI2 = 'radio://0/60/2M/E7E7E7E7E5'
URI3 = 'radio://0/60/2M/E7E7E7E7E7'
DEFAULT_HEIGHT = 0.1
SAMPLE_PERIOD_MS = 100

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


#specify start, intermediate, and destination
#wind speed option are 6.1, 7.6, 9.7
WIND_SPEED = 7.6
WIND_ANGLE = 90

waiting_time = [[0,0,0,0],[0,0,0,0],[0,0,0,0]] #to use: waiting_time[droneid][whichnode]. Note: whichnode = 0 means start, = 1 means intermediate 1, = 2 means intermediate 2, =3 means destination.
travel_dist = [0,0,0]

""" #path 1
start = [(1.65,1.20,0.10),(0.4,0.53,0.83),(1.19,1.12,0.12)] #for scf, scf2, and scf3 / id no 1,2,and 3 / uri1, uri2, and uri3 respectively
intermediate = [(1.34,0.46,0.4),(0.70,0.76,0.75)]
destination = [(0.68,0.34,1.01),(1.87,-0.16,0.34),(1.29,-0.36,0.94)]
path_name = [['A1','B2','F1','F2'],['J1','B2','F1','B1'],['A/E1','B2','F1','C1']] """


#path 2
start = [(0.68,0.34,1.01),(1.87,-0.16,0.34),(1.29,-0.36,0.94)] #for scf, scf2, and scf3 / id no 1,2,and 3 / uri1, uri2, and uri3 respectively
intermediate = [(1.34,0.46,0.4),(0.70,0.76,0.75)]
destination = [(1.65,1.20,0.10),(0.4,0.53,0.83),(1.19,1.12,0.12)]
path_name = [['F2','B2','F1','A1'],['B1','B2','F1','J1'],['C1','B2','F1','A/E1']]

""" #path 2 not used
start = [(0.59,1.13,1.08),(1.24,-0.31,0.94),(0.70,-0.74,0.58)] #for scf, scf2, and scf3 / id no 1,2,and 3 / uri1, uri2, and uri3 respectively
intermediate = (1.34,0.46,0.4)
destination = [(0.70,0.76,0.75),(1.65,1.20,0.10),(1.47,1.18,0.12)]
travel_dist = [0,0,0]
waiting_time = [[0,0,0],[0,0,0],[0,0,0]] #to use: waiting_time[droneid][whichnode]. Note: whichnode = 0 means start, = 1 means intermediate, = 2 means destination.
path_name = [['E2','B2','C1'],['C1','B2','A1'],['G1','B2','A2']] #this is path for e3, e5, and e7 respectively
# order of flight: e5, e3, e7 """

""" #path 3
start = [(1.24,-0.31,0.94),(0.70,-0.74,0.58),(1.65,1.20,0.10)] #for scf, scf2, and scf3 / id no 1,2,and 3 / uri1, uri2, and uri3 respectively
intermediate = (0.70,0.76,0.75)
destination = [(0.04,0.53,0.83),(1.19,1.12,0.12),(0.59,1.13,1.28)]
travel_dist = [0,0,0]
waiting_time = [[0,0,0],[0,0,0],[0,0,0]] #to use: waiting_time[droneid][whichnode]. Note: whichnode = 0 means start, = 1 means intermediate, = 2 means destination.
path_name = [['C1','F1','J1'],['G1','F1','A/B1'],['A1','F1','E2']] #this is path for e3, e5, and e7 respectively
"""

""" #path 4
start = [(1.24,-0.31,0.58),(0.70,-0.74,0.10),(1.65,1.20,0.94)] #for scf, scf2, and scf3 / id no 1,2,and 3 / uri1, uri2, and uri3 respectively
intermediate = [(0.70,0.76,0.75),(0.70,0.76,0.75)]
destination = [(0.04,0.53,0.12),(1.19,1.12,1.28),(0.59,1.13,0.83)]
travel_dist = [0,0,0]

path_name = [['C1','F1','J1'],['G1','F1','A/B1'],['A1','F1','E2']] #this is path for e3, e5, and e7 respectively
# order of flight: e7,e3,e5  """

""" #path 4 not used
start = [(0.04,0.53,0.83),(0.70,-0.74,0.58),(1.24,-0.31,0.94)] #for scf, scf2, and scf3 / id no 1,2,and 3 / uri1, uri2, and uri3 respectively
intermediate = (0.70,0.76,0.75)
destination = [(0.68,0.34,1.01),(1.19,1.12,0.12),(0.04,0.53,0.83)]
travel_dist = [0,0,0]
waiting_time = [[0,0,0],[0,0,0],[0,0,0]] #to use: waiting_time[droneid][whichnode]. Note: whichnode = 0 means start, = 1 means intermediate, = 2 means destination.
path_name = [['C1','F2','F1'],['A1','F2','J1'],['E1B','F2','G1']] #this is path for e3, e5, and e7 respectively
# order of flight:  """

relative_wind_direction = [0,0,0]
first_timestamp = [0,0,0]
#this contains waiting node for droneid 0,1,and 2 respectively. the value represent which node it is in (look at the other comment on whichnode), 
#if the value is -1 then it is currently not waiting in any node
# 0 is for start, 1 is for the first intermediate, 2 is for the second intermediate, and 3 is for the destination.
waiting_node = [0,0,0]  

upper_bat_thresh = 4.15 #battery percentage in which we stop charging cause we consider it to be fully charged

#distance from drone to the helipads in the top of the building when the drone is first hovering in the source node and when it first arrive in the other node
hi_relative_height = 0.5 
#distance from drone to the helipads in the top of the building when the drone is trying to land(has to be a small number so that drone does not bounce)
lo_relative_height = 0.1
safety_sleep = 10
first_drone_id = -1 # this is just for declaration

hover_time = 3 #in seconds
hover_time *= 10 #convert to deci seconds
hover_time = int(hover_time)

from enum import Enum
class WindDirection(Enum):
    NONE = 0
    HEAD = 1
    TAIL = 2
    CROSS = 3
    DIAGONAL_HEAD = 4
    DIAGONAL_TAIL = 5

def find_angle(x1, y1, x2, y2) -> float:
    "Finds the angle between two points."
    myradians = math.atan2(y2-y1, x2-x1)
    mydegrees = math.degrees(myradians)
    return mydegrees


def determine_direction(start, end) -> WindDirection:
    "Determines if the wind is a head wind, tail wind, side wind, diagonal against wind, diagonal with wind, or no wind."

    if WIND_SPEED == 0:
        return WindDirection.NONE

    # Calculate drone angle
    drone_angle = find_angle(start[0], start[1], end[0], end[1])

    # Calculate wind angle relative to drone angle
    rel_wind_angle = WIND_ANGLE - drone_angle
    
    return rel_wind_angle


def polar_to_cartesian(radius: float, angle: float):
    "Converts polar coordinate radius and angle to Cartesian X and Y"
    x = radius * math.cos(angle * (math.pi / 180))
    y = radius * math.sin(angle * (math.pi / 180))
    return x, y

def init_log_history():
    pass


def write_log_history():
    folder_name = f"/home/alan/drone/Ours/OurCodes/flying/2Intermediate/path_2/wind_speed_2/wind_direction_{WIND_ANGLE}"
    
    drone_names = ['E3','E5','E7']
    for i in range(3):
        with open(f'{folder_name}/drone_{drone_names[i]}.csv', mode='w') as csv_file:
            fieldnames = ['time.ms'] + [param[0] for param in log_parameters] + ['drone_id','node_name','wind_speed','global_wind_direction','relative_wind_direction','travel_dist','waiting_time']
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

    data['drone_id'] = 0
    
    bat_volt[0] = float(data['pm.vbat'])
    if waiting_node != -1:
        data['node_name'] = path_name[0][waiting_node[0]]
    else:
        data['node_name'] = 'fly'
    data['wind_speed'] = WIND_SPEED
    data['global_wind_direction'] = WIND_ANGLE
    data['relative_wind_direction'] = relative_wind_direction[0]

    curr_coor = (data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'])
    if len(log_history[0]) > 0:
        last_coor = (log_history[0][-1]['stateEstimate.x'], log_history[0][-1]['stateEstimate.y'], log_history[0][-1]['stateEstimate.z'])
        data['time.ms'] = timestamp - first_timestamp[0]
    else:
        last_coor = start[0][:-1] + (0,)
        data['time.ms'] = 0
        first_timestamp[0] = timestamp
    travel_dist[0] += distance(curr_coor,last_coor)
    data['travel_dist'] = travel_dist[0]

    #waiting time
    if waiting_node[0] != -1: #if it is not currently flying
        waiting_time[0][waiting_node[0]] += SAMPLE_PERIOD_MS
    
    data['waiting_time'] = '[' + '|'.join(map(str,waiting_time[0])) + ']' 
    #format -> [waitingTimeInStart|waitingTimeInIntermediate1|waitingtimeinter2|waitingTimeInDestination] all waiting time is currently in ms

    log_history[0].append(data)
    log_cycles += 1

def logconf_callback_2(timestamp, data, logconf):
    global log_history, log_cycles
    data['drone_id'] = 1
    data['time.ms'] = timestamp
    bat_volt[1] = float(data['pm.vbat'])
    if waiting_node != -1:
        data['node_name'] = path_name[1][waiting_node[1]]
    else:
        data['node_name'] = 'fly'
    data['wind_speed'] = WIND_SPEED
    data['global_wind_direction'] = WIND_ANGLE
    data['relative_wind_direction'] = relative_wind_direction[1]
    
    curr_coor = (data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'])
    if len(log_history[1]) > 0:
        last_coor = (log_history[1][-1]['stateEstimate.x'], log_history[1][-1]['stateEstimate.y'], log_history[1][-1]['stateEstimate.z'])
        data['time.ms'] = timestamp - first_timestamp[1]
    else:
        last_coor = start[1][:-1] + (0,) 
        data['time.ms'] = 0
        first_timestamp[1] = timestamp
    travel_dist[1] += distance(curr_coor,last_coor)
    data['travel_dist'] = travel_dist[1]

    #waiting time
    if waiting_node[1] != -1: #if it is not currently flying
        waiting_time[1][waiting_node[1]] += SAMPLE_PERIOD_MS
    
    data['waiting_time'] = '[' + '|'.join(map(str,waiting_time[1])) + ']' 

    log_history[1].append(data)
    log_cycles += 1

def logconf_callback_3(timestamp, data, logconf):
    global log_history, log_cycles
    data['drone_id'] = 2
    data['time.ms'] = timestamp
    bat_volt[2] = float(data['pm.vbat'])
    if waiting_node != -1:
        data['node_name'] = path_name[2][waiting_node[2]]
    else:
        data['node_name'] = 'fly'
    data['wind_speed'] = WIND_SPEED
    data['global_wind_direction'] = WIND_ANGLE
    data['relative_wind_direction'] = relative_wind_direction[2]

    curr_coor = (data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'])
    if len(log_history[2]) > 0:
        last_coor = (log_history[2][-1]['stateEstimate.x'], log_history[2][-1]['stateEstimate.y'], log_history[2][-1]['stateEstimate.z'])
        data['time.ms'] = timestamp - first_timestamp[2]
    else:
        last_coor = start[2][:-1] + (0,)
        data['time.ms'] = 0
        first_timestamp[2] = timestamp
    travel_dist[2] += distance(curr_coor,last_coor)
    data['travel_dist'] = travel_dist[2]
    
    #waiting time
    if waiting_node[2] != -1: #if it is not currently flying
        waiting_time[2][waiting_node[2]] += SAMPLE_PERIOD_MS
    
    data['waiting_time'] = '[' + '|'.join(map(str,waiting_time[2])) + ']' 

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

def determine_order(intermediates, start, drones):
    undecided_id = [0,1,2]
    result = []
    #iterate over both intermediate
    for intermediate in intermediates:
        #look for the closest drone
        dist = []
        #calculate the distance between each drone to intermediate
        for i in range(3):
            if i in undecided_id:
                dist.append((distance(start[i],intermediate),drones[i],i))
        #sort based on the distance
        dist.sort()
        print(dist)
        closest_drone = dist[0][1]
        closest_drone_id = dist[0][2]
        result.append(closest_drone)
        result.append(closest_drone_id)
        undecided_id.remove(closest_drone_id)
    result.append(drones[undecided_id[0]])
    result.append(undecided_id[0])

    return result

# this is for the second one to leave the charger
def wait_for_drone(drone,drone_id):

    while bat_volt[drone_id] < upper_bat_thresh:
        time.sleep(0.1)
     
    #we can know which intermediate it is currently in by looking at their id
    if drone_id == first_drone_id:
        curr_intermediate = intermediate[0]
    else:
        curr_intermediate = intermediate[1]

    #go to destination
    waiting_node[drone_id] = -1
    #hover
    for y in range(hover_time):
        drone.commander.send_position_setpoint(curr_intermediate[0], curr_intermediate[1], curr_intermediate[2]+hi_relative_height, 0)
        time.sleep(0.1)
    relative_wind_direction[drone_id] = determine_direction(curr_intermediate,destination[drone_id])
    #go
    for y in range(hover_time):
        drone.commander.send_position_setpoint(destination[drone_id][0], destination[drone_id][1], destination[drone_id][2]+hi_relative_height, 0)
        time.sleep(0.1)
    #slowly landing
    for y in range(hover_time):
        drone.commander.send_position_setpoint(destination[drone_id][0], destination[drone_id][1], lo_relative_height, 0)
        time.sleep(0.1)
    

# this is for the first one to leave the charger and the third drone to go from start
# drone a is the first one to leave the charger, while drone b is the drone that is going to the available charger
def drone_go(drone_a,drone_a_id,drone_b,drone_b_id):
    
    waiting_node[drone_a_id] = -1
    waiting_node[drone_b_id] = -1

    #we can know which intermediate it is currently in by looking at their id
    intermediate_index = 0
    if drone_a_id == first_drone_id:
        curr_intermediate = intermediate[0]
        intermediate_index = 1
    else:
        curr_intermediate = intermediate[1]
        intermediate_index = 2
        
    #dronea go to the destination, while the droneb go to intermediate
    #hover
    for y in range(hover_time):
        drone_a.commander.send_position_setpoint(curr_intermediate[0], curr_intermediate[1], curr_intermediate[2]+hi_relative_height, 0)
        drone_b.commander.send_position_setpoint(start[drone_b_id][0], start[drone_b_id][1], start[drone_b_id][2]+hi_relative_height, 0)
        time.sleep(0.1)
    relative_wind_direction[drone_a_id] = determine_direction(curr_intermediate,destination[drone_a_id])
    relative_wind_direction[drone_b_id] = determine_direction(start[drone_b_id],curr_intermediate)
    #go
    for y in range(hover_time):
        drone_a.commander.send_position_setpoint(destination[drone_a_id][0], destination[drone_a_id][1], destination[drone_a_id][2]+hi_relative_height, 0)
        drone_b.commander.send_position_setpoint(curr_intermediate[0], curr_intermediate[1], curr_intermediate[2]+hi_relative_height, 0)
        time.sleep(0.1)
    relative_wind_direction[drone_a_id] = 0
    relative_wind_direction[drone_b_id] = 0
    #slowly landing
    for y in range(hover_time):
        drone_a.commander.send_position_setpoint(destination[drone_a_id][0], destination[drone_a_id][1],lo_relative_height, 0)
        drone_b.commander.send_position_setpoint(curr_intermediate[0], curr_intermediate[1], lo_relative_height, 0)
        time.sleep(0.1)

    waiting_node[drone_a_id] = 3
    waiting_node[drone_b_id] = intermediate_index

    #wait for drone_b to charge
    time.sleep(safety_sleep)
    while bat_volt[drone_b_id] < upper_bat_thresh:
        time.sleep(0.1)

    # drone_b go to destination
    waiting_node[drone_b_id] = -1
    #hover
    print(f'hover {drone_b_id} go to {curr_intermediate[0], curr_intermediate[1], curr_intermediate[2]+hi_relative_height}')
    print(f'hover {drone_b_id} go to {start[drone_b_id][0], start[drone_b_id][1], start[drone_b_id][2]+hi_relative_height}')
    for y in range(hover_time):
        drone_b.commander.send_position_setpoint(curr_intermediate[0], curr_intermediate[1], curr_intermediate[2]+hi_relative_height, 0)
        time.sleep(0.1)
    relative_wind_direction[drone_b_id] = determine_direction(curr_intermediate,destination[drone_b_id])
    #go
    for y in range(hover_time):
        drone_b.commander.send_position_setpoint(destination[drone_b_id][0], destination[drone_b_id][1], destination[drone_b_id][2]+hi_relative_height, 0)
        time.sleep(0.1)
    relative_wind_direction[drone_b_id] = 0
    #slowly landing
    for y in range(hover_time):
        drone_b.commander.send_position_setpoint(destination[drone_b_id][0], destination[drone_b_id][1], lo_relative_height, 0)
        time.sleep(0.1)
    
    waiting_node[drone_b_id] = 2


if __name__ == '__main__':


    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected to E3")
        with SyncCrazyflie(URI2, cf=Crazyflie(rw_cache='./cache')) as scf2:
            print("Connected to E5")
            with SyncCrazyflie(URI3, cf=Crazyflie(rw_cache='./cache')) as scf3:
                print("Connected to E7")

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
                

                """
                To determine order:
                The we iterate over the list of intermediate node. In the first node, we look at which one is the closest, and that
                will be the first drone. Then in the second node, we look at which one is the closest out of the rest of the drone, and
                that will be the second drone. The unselected drone will automatically be the third drone.

                First and second drone go. Wait until one of them is fully charge, then create another process for the second drone
                that is still charging the second process will wait until the second drone is good to go, then make it go to the 
                destination. While the second process is still waiting, the first process make the first drone go to the destination,
                and the third drone go to the charging port that the first drone just use. 
                """

                #determine the order of drone flights
                drones = [cf,cf2,cf3]
                first_drone, first_drone_id, second_drone, second_drone_id, third_drone, third_drone_id = determine_order(intermediate,start,drones)
                #drone id -> 0 for uri1, 1 for uri2, and 2 for uri3

                

                print(f'first id {first_drone_id}')
                print(f'second id {second_drone_id}')
                print(f'third id {third_drone_id}')

                       
                waiting_node[first_drone_id] = -1 #first drone is no longer waiting
                waiting_node[second_drone_id] = -1 #second drone is no longer waiting
                #first and second drone go to intermediate
                #hover
                for y in range(hover_time):
                    first_drone.commander.send_position_setpoint(start[first_drone_id][0], start[first_drone_id][1], start[first_drone_id][2]+hi_relative_height, 0)
                    second_drone.commander.send_position_setpoint(start[second_drone_id][0], start[second_drone_id][1], start[second_drone_id][2]+hi_relative_height, 0)
                    time.sleep(0.1)
                relative_wind_direction[first_drone_id] = determine_direction(start[first_drone_id],intermediate[0])
                relative_wind_direction[second_drone_id] = determine_direction(start[second_drone_id],intermediate[1]) 
                #go
                for y in range(hover_time):
                    first_drone.commander.send_position_setpoint(intermediate[0][0], intermediate[0][1], intermediate[0][2]+hi_relative_height, 0)
                    second_drone.commander.send_position_setpoint(intermediate[1][0], intermediate[1][1], intermediate[1][2]+hi_relative_height, 0)
                    time.sleep(0.1)
                relative_wind_direction[first_drone_id] = 0
                relative_wind_direction[second_drone_id] = 0
                #slowly landing
                for y in range(hover_time):
                    first_drone.commander.send_position_setpoint(intermediate[0][0], intermediate[0][1], lo_relative_height, 0)
                    second_drone.commander.send_position_setpoint(intermediate[1][0], intermediate[1][1], lo_relative_height, 0)
                    time.sleep(0.1)

                waiting_node[first_drone_id] = 1 #first drone is waiting in first intermediate node
                waiting_node[second_drone_id] = 2 #second drone is waiting in second intermediate node

                #wait for one of the drone to finish charging
                time.sleep(safety_sleep) # for safety purposes it always have to sleep of a certain period of time
                while True:
                    if bat_volt[first_drone_id] >= upper_bat_thresh: # first one go
                        drone_a = first_drone
                        drone_a_id = first_drone_id
                        drone_b = second_drone
                        drone_b_id = second_drone_id
                        break
                    if bat_volt[second_drone_id] >= upper_bat_thresh: # second one go
                        drone_b = first_drone
                        drone_b_id = first_drone_id
                        drone_a = second_drone
                        drone_a_id = second_drone_id
                        break
                    time.sleep(0.1)
                
                
                #1 thread will wait for 1 drone to charge until full, and the other thread will make the other 2 drone fly
                
                thread1 = threading.Thread(target=wait_for_drone,args=(drone_b,drone_b_id))
                thread2 = threading.Thread(target=drone_go,args=(drone_a,drone_a_id,third_drone,third_drone_id))
                thread1.start()
                thread2.start()
                

                # to wait until both thread are finish or in other words all the drones are done flying
                from threading import Thread
                thread1.join() 
                thread2.join()
                #--------------------------- Flying done

                
                waiting_node[third_drone_id] = 2
        
                cf.commander.send_stop_setpoint()
                cf2.commander.send_stop_setpoint()
                cf3.commander.send_stop_setpoint()

                write_log_history()
                logconf1.stop()
                logconf2.stop()
                logconf3.stop()

                