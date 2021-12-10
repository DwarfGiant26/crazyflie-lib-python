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

URI1 = 'radio://0/80/2M/E7E7E7E7E7'
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

log_history = []
log_cycles = 0


def init_log_history():
    pass


def write_log_history():
    with open('flight_log3.csv', mode='w') as csv_file:
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
    data['pm.vbat'] = 1
    """ np.frombuffer(struct.pack(
        "H", int(hex(data['pm.vbat']), 16)), dtype=np.float16)[0] """
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
    with SyncCrazyflie(URI2, cf=Crazyflie(rw_cache='./cache')) as scf:
        print(1)
        with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf2:
            with SyncCrazyflie(URI3, cf=Crazyflie(rw_cache='./cache')) as scf3:

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

                for y in range(30):
                    cf.commander.send_hover_setpoint(0, 0, 0, 0.3)
                    cf2.commander.send_hover_setpoint(0, 0, 0, 0.3)
                    cf3.commander.send_hover_setpoint(0, 0, 0, 0.3)
                    time.sleep(0.1)

                """ for _ in range(15):
                    cf.commander.send_hover_setpoint(0.6, -0.6, 0, 0.4)
                    time.sleep(0.1) """

                for _ in range(30):
                    cf.commander.send_hover_setpoint(0, 0, 50, 0.3)
                    cf2.commander.send_hover_setpoint(0, 0, -50, 0.3)
                    cf3.commander.send_hover_setpoint(0, 0, 50, 0.3)
                    time.sleep(0.1)

                for _ in range(10):
                    cf.commander.send_hover_setpoint(0, 0, 0, 0.02)
                    cf2.commander.send_hover_setpoint(0, 0, 0, 0.02)
                    cf3.commander.send_hover_setpoint(0, 0, 0, 0.02)
                    time.sleep(0.1)

                # for _ in range(10):
                #     cf.commander.send_hover_setpoint(0, 0, 0, 0.1)
                #     time.sleep(0.1)

                # for y in range(10):
                #     cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
                #     time.sleep(0.1)

                cf.commander.send_stop_setpoint()
                cf2.commander.send_stop_setpoint()
                cf3.commander.send_stop_setpoint()

                logconf.stop()

                write_log_history()
