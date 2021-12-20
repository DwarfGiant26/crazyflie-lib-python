# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to one crazyflie, sets the initial position/yaw
and flies a trajectory.

The initial pose (x, y, z, yaw) is configured in a number of variables and
the trajectory is flown relative to this position, using the initial yaw.

This example is intended to work with any absolute positioning system.
It aims at documenting how to take off with the Crazyflie in an orientation
that is different from the standard positive X orientation and how to set the
initial position of the kalman estimator.
"""
import math
import time
import csv
import logging

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
URI1 = 'radio://0/60/2M/E7E7E7E7E7'
URI2 = 'radio://0/60/2M/E7E7E7E7E5'
URI3 = 'radio://0/60/2M/E7E7E7E7E3'
DEFAULT_HEIGHT = 0.1
SAMPLE_PERIOD_MS = 10

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

logging.basicConfig(level=logging.ERROR)


# Change the sequence according to your setup
# THESE ARE THE COORDINATES OF TEH BUILDING IN REFERENC TO A SINGLE ORIGIN POSITION SHARED BY ALL 3 DRONES :D
#             x    y    z
sequence1 = [
    (-0.5, -0.5, 0.2),
    (0.4, 0.4, 0.1),
    # (-0.5, -0.5, 0.2),
    # (0, 0.4, 0.3),
    # (0, 0, 0.1),
]

sequence2 = [
    (0.4, 0.3, 0.2),
    (-1.1, 0.4, 0.1),
    # (0.4, 0.3, 0.3),
    # (0, 0.4, 0.3),
    # (0, 0, 0.1),
]

sequence3 = [
    (-1.2, 0.4, 0.3),
    (-0.5, -0.4, 0.1),
    # (-1.2, 0.4, 0.2),
    # (0, 0.4, 0.3),
    # (0, 0, 0.1),
]

sequence_ls = [sequence1, sequence2, sequence3]

def logconf_callback_1(timestamp, data, logconf):
    global log_history, log_cycles
    data['time.ms'] = timestamp
    # Convert FP16 to FP32
    
    log_history[0].append(data)
    log_cycles += 1

def logconf_callback_2(timestamp, data, logconf):
    global log_history, log_cycles
    data['time.ms'] = timestamp
    # Convert FP16 to FP32
    
    log_history[1].append(data)
    log_cycles += 1

def logconf_callback_3(timestamp, data, logconf):
    global log_history, log_cycles
    data['time.ms'] = timestamp 
    # Convert FP16 to FP32
    
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

def write_log_history():
    for i in range(3):
        with open(f'multiple1_drone_{i}.csv', mode='w') as csv_file:
            fieldnames = ['time.ms'] + [param[0] for param in log_parameters]
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            writer.writeheader()
            for item in log_history[i]:
                writer.writerow(item)

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def run_sequence(scf, scf2, scf3, sequence_ls, initial_drone_pos_ls):
    cf1 = scf.cf
    cf2 = scf2.cf
    cf3 = scf3.cf

    cf_ls = [cf1, cf2, cf3]

    for i in range(len(sequence1)):
        for _ in range(30):
            for j,cf in enumerate(cf_ls):

                x = sequence_ls[j][i][0] + initial_drone_pos_ls[j][0]
                y = sequence_ls[j][i][1] + initial_drone_pos_ls[j][1]
                z = sequence_ls[j][i][2] + initial_drone_pos_ls[j][2]
                yaw = initial_drone_pos_ls[j][3]

        # for position in sequence1:
        #     print('Setting position {}'.format(position))
            
                cf.commander.send_position_setpoint(x, y, z, yaw)
            
            time.sleep(0.1)

    cf1.commander.send_stop_setpoint()
    cf2.commander.send_stop_setpoint()
    cf3.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    # (initial x, initial y, initial z, inital yaw in degrees)
    # INITIAL OF DRONE 1
    initial_drone1 = (0, 0, 0, 0)

    # INITIAL OF DRONE 2
    initial_drone2 = (0, 0, 0, 0)

    # INITIAL OF DRONE 3
    initial_drone3 = (0,0, 0, 0)

    initial_drone_pos_ls = [initial_drone1, initial_drone2, initial_drone3]

    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf:
        with SyncCrazyflie(URI2, cf=Crazyflie(rw_cache='./cache')) as scf2:
            with SyncCrazyflie(URI3, cf=Crazyflie(rw_cache='./cache')) as scf3:
                # LOGGING DATA
                initial_pos1 = set_initial_position(scf, initial_drone_pos_ls[0][0], initial_drone_pos_ls[0][1], initial_drone_pos_ls[0][2], initial_drone_pos_ls[0][3])
                initial_pos2 = set_initial_position(scf2, initial_drone_pos_ls[1][0], initial_drone_pos_ls[1][1], initial_drone_pos_ls[1][2], initial_drone_pos_ls[1][3])
                initial_pos3 = set_initial_position(scf3, initial_drone_pos_ls[2][0], initial_drone_pos_ls[2][1], initial_drone_pos_ls[2][2], initial_drone_pos_ls[2][3])
                
                reset_estimator(scf)
                reset_estimator(scf2)
                reset_estimator(scf3)

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

                run_sequence(scf, scf2, scf3, sequence_ls, initial_drone_pos_ls)
                
                time.sleep(10)
                #should be after stop()
                write_log_history()
                logconf1.stop()
                logconf2.stop()
                logconf3.stop()