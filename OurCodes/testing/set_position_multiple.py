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

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
URI1 = 'radio://0/60/2M/E7E7E7E7E7'
URI2 = 'radio://0/60/2M/E7E7E7E7E5'
URI3 = 'radio://0/60/2M/E7E7E7E7E3'

# Change the sequence according to your setup
# THESE ARE THE COORDINATES OF TEH BUILDING IN REFERENC TO A SINGLE ORIGIN POSITION SHARED BY ALL 3 DRONES :D
#             x    y    z
sequence1 = [
    (0, 0.2, 0.4),
    (0, 0.2, 0.1),
    # (0, 0.4, 0.3),
    # (0, 0, 0.1),
]

sequence2 = [
    (0, 0.2, 0.3),
    (0, 0.2, 0.1),
    # (0, 0.4, 0.3),
    # (0, 0, 0.1),
]

sequence3 = [
    (0, 0.2, 0.2),
    (0, 0.2, 0.1),
    # (0, 0.4, 0.3),
    # (0, 0, 0.1),
]

sequence_ls = [sequence1, sequence2, sequence3]


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

                x = sequence_ls[j][i][0] - initial_drone_pos_ls[j][0]
                y = sequence_ls[j][i][1] - initial_drone_pos_ls[j][1]
                z = sequence_ls[j][i][2] - initial_drone_pos_ls[j][2]
                yaw = initial_drone_pos_ls[j][3]

        # for position in sequence1:
        #     print('Setting position {}'.format(position))
            
                cf.commander.send_position_setpoint(x, y, z, yaw)
            
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    # (initial x, initial y, initial z, inital yaw in degrees)
    # INITIAL OF DRONE 1
    initial_drone1 = (0, -0.2, 0, 0)

    # INITIAL OF DRONE 2
    initial_drone2 = (0, -0.2, 0, 0)

    # INITIAL OF DRONE 3
    initial_drone3 = (0, -0.2, 0, 0)

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
                run_sequence(scf, scf2, scf3, sequence_ls, initial_drone_pos_ls)
