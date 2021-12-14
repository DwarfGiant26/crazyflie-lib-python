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

URI = 'radio://0/80/2M/E7E7E7E7E7'
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

log_history = []
log_cycles = 0


def init_log_history():
    pass


def write_log_history():
    with open('flight_log.csv', mode='w') as csv_file:
        fieldnames = ['time.ms'] + [param[0] for param in log_parameters]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        writer.writeheader()
        for item in log_history:
            writer.writerow(item)


def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        # time.sleep(1)
        mc.stop()


def logconf_callback(timestamp, data, logconf):
    global log_history, log_cycles
    data['time.ms'] = timestamp
    # Convert FP16 to FP32
    log_history.append(data)
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


if __name__ == '__main__':

    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("reached sync")

        scf.cf.param.add_update_callback(
            group="deck", name="bcLighthouse4", cb=param_deck_flow)
        time.sleep(1)

        init_log_history()

        logconf = LogConfig(name='Parameters', period_in_ms=SAMPLE_PERIOD_MS)
        for param in log_parameters:
            logconf.add_variable(param[0], param[1])

        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(logconf_callback)

        #if is_deck_attached:
        logconf.start()

        while(data["pm.vbat"] < 4.1):
            time.sleep(1)

        logconf.stop()

        write_log_history()
