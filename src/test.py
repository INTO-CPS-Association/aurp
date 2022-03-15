import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_CLOSED_LOOP_CONTROL, MOTOR_TYPE_HIGH_CURRENT, AXIS_STATE_IDLE, AXIS_STATE_ENCODER_INDEX_SEARCH, CONTROL_MODE_VELOCITY_CONTROL, CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_VEL_RAMP
import numpy as np
import time
import math
from sys import exit
import matplotlib.pyplot as plt
od = odrive.find_any()

# ================= Config =====================
od.config.enable_brake_resistor = True
od.config.brake_resistance = 2.2
od.axis0.motor.config.torque_constant = 8.27 / 270
od.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
od.axis0.controller.config.vel_limit = 2.0
od.axis0.motor.config.current_lim = 5.0
od.axis0.motor.config.pole_pairs = 7
od.axis0.encoder.config.cpr = 8192
od.axis0.motor.config.calibration_current = 10.0

# ================= Calibrate ===================
od.axis0.encoder.config.use_index = True
od.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH

while od.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

print("index search complete")

od.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

while od.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

print(f"error is {od.axis0.error}")
print(f"encoder.config.phase_offset {od.axis0.encoder.config.phase_offset}")
print(f"od.encoder.config.direction: {od.axis0.encoder.config.direction}")

if od.axis0.error != 0:
    raise RuntimeError("Unable to calibrate")
    exit(-1)

od.axis0.encoder.config.pre_calibrated = True

# od.save_configuration()


# od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# od.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
# od.axis0.controller.config.vel_ramp_rate = 0.5
# od.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
# od.axis0.controller.input_vel = 2.0

od.axis0.controller.config.vel_limit = 50.0
od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
od.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
od.axis0.controller.config.circular_setpoints = False
od.axis0.controller.input_pos = 0.0

t_cur = time.time()
t_start = t_cur
t_sample_last = time.time()
t_setpoint_last = time.time()
ts = 1e-3
tsp = 1e-1
timestamp_sample = []
timestamp_setpoint = []
q = []
q_setpoint = []
i = []
t_duration = 5.0


def pos_setpoint(t):
    return 1.0
    # return math.sin(t)*0.5 + 0.5

while True:
    t_cur = time.time()

    if t_cur-t_setpoint_last >= tsp:
        t_setpoint_last = t_cur
        qsp = pos_setpoint(t_cur - t_start)
        timestamp_setpoint.append(t_cur)
        od.axis0.controller.input_pos = qsp
        q_setpoint.append(qsp)

    if t_cur-t_sample_last >= ts:
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
dt_sample = timestamp_sample[1:]-timestamp_sample[:-1]
dt_sample = timestamp_setpoint[1:]-timestamp_setpoint[:-1]
timestamp_sample = timestamp_sample-t_start
dt_setpoint = timestamp_setpoint-t_start

fig, (ax1,ax2) = plt.subplots(2,1,sharex=True)
ax1.plot(timestamp_sample,q, label="actual")
ax1.plot(timestamp_setpoint,q_setpoint,label="setpoint")
ax1.set_ylabel("q(t)")
ax2.plot(timestamp_sample,i)
ax2.set_ylabel("i(t)")
ax2.set_xlabel("t")

fig, ax = plt.subplots()
ax.hist(dt_sample)

fig, ax = plt.subplots()
ax.hist(dt_setpoint)

# timestamp, actual_q_0, actual_current_0


plt.show()