import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_CLOSED_LOOP_CONTROL, MOTOR_TYPE_HIGH_CURRENT, AXIS_STATE_IDLE, AXIS_STATE_ENCODER_INDEX_SEARCH, CONTROL_MODE_VELOCITY_CONTROL, CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_VEL_RAMP, INPUT_MODE_POS_FILTER
import numpy as np
import time
import math
from sys import exit
import matplotlib.pyplot as plt
from odrive.utils import dump_errors
od = odrive.find_any()

# ================= Calibrate ===================
od.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH

while od.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

if od.axis0.error != 0:
    dump_errors(od)
    exit(0)
else:
    print("index search complete")


od.axis0.encoder.config.pre_calibrated = True


od.axis0.controller.config.vel_limit = 50.0
od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
od.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
od.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
od.axis0.controller.config.circular_setpoints = False
od.axis0.controller.input_pos = 0.25
od.axis0.controller.config.input_filter_bandwidth = 50.0  # [Hz]
od.axis0.controller.config.vel_limit = 2.0  # [turns/s]

# while True:
#     time.sleep(0.5)
#     od.axis0.controller.input_pos = 0.25

t_cur = time.time()
t_start = t_cur
t_sample_last = time.time()
t_setpoint_last = time.time()
dt_sample = 1e-3
tsp = 1e-2
timestamp_sample = []
timestamp_setpoint = []
q = []
q_setpoint = []
i = []
t_duration = 5.0


def pos_setpoint(t):
    # return 0.25
    return math.sin(3*t)*0.5 + 0.5

while True:
    t_cur = time.time()

    if t_cur-t_setpoint_last >= tsp:
        t_setpoint_last = t_cur
        qsp = pos_setpoint(t_cur - t_start)
        timestamp_setpoint.append(t_cur)
        od.axis0.controller.input_pos = qsp
        q_setpoint.append(qsp)

    if t_cur - t_sample_last >= dt_sample:
        t_sample_last = t_cur
        timestamp_sample.append(t_cur)
        q.append(od.axis0.encoder.pos_estimate)
        i.append(od.axis0.motor.current_control.Iq_measured)
        
    if t_cur - t_start > t_duration:
        break

timestamp_sample = np.array(timestamp_sample)
timestamp_setpoint = np.array(timestamp_setpoint)
q = np.array(q)
q_setpoint = np.array(q_setpoint)
i = np.array(i)
dt_sample = timestamp_sample[1:] - timestamp_sample[:-1]
dt_setpoint = timestamp_setpoint[1:] - timestamp_setpoint[:-1]
timestamp_sample = timestamp_sample - t_start  # Zero timestamp
timestamp_setpoint = timestamp_setpoint - t_start  # Zero timestamp

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(timestamp_sample, q, label="actual")
ax1.plot(timestamp_setpoint, q_setpoint, label="setpoint")
ax1.set_ylabel("q(t)")
ax2.plot(timestamp_sample, i)
ax2.set_ylabel("i(t)")
ax2.set_xlabel("t")

fig, ax = plt.subplots()
ax.hist(dt_sample)
plt.xlabel('Time [s]')
plt.ylabel('Amount [-]')
plt.title('Sample Rate Histogram')


fig, ax = plt.subplots()
ax.hist(dt_setpoint)
plt.xlabel('Time [s]')
plt.ylabel('Amount [-]')
plt.title('Setpoint Update Rate Histogram')

# Generate CSV file with header:
# timestamp, actual_q_0, actual_current_0

plt.show()

dump_errors(od,True)
