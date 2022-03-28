from ossaudiodev import control_labels
import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_CLOSED_LOOP_CONTROL, MOTOR_TYPE_HIGH_CURRENT, AXIS_STATE_IDLE, AXIS_STATE_ENCODER_INDEX_SEARCH, CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH
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

od.axis0.controller.config.vel_limit = 50.0  # [turns/s]
od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
od.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
od.axis0.controller.input_mode = INPUT_MODE_PASSTHROUGH
od.axis0.controller.config.vel_limit = 2.0  # [turns/s]

t_cur = time.time()
t_start = t_cur
t_sample_last = time.time()
t_setpoint_last = time.time()
dt_sample_target = 1e-3
dt_setpoint_target = 1e-3 / 3  # 3 kHz torque control
timestamp_sample = []
timestamp_setpoint = []
q = []
q_setpoint = []
qd = []
qd_setpoint = []
i = []
i_setpoint = []
t_duration = 5.0

# Position and velocity trajectories
def pos_setpoint(t):
    return math.sin(0.05*t_duration*t**3 + 2*t)*0.5 + 0.5

def vel_setpoint(t):
    return math.cos(0.05*t_duration*t**3 + 2*t)*0.025*t**3  # derivative of pos_setpoint


# Torque Controller
inertia_rotor = 1E-4  # inertia of the D5053 rotor according to ODrive
inertia_pendulum = 0.019  # inertia of rod with attached weights of 250 g + 200 g + 100 g = 550 g offset X mm
od.axis0.motor.config.inertia = inertia_rotor + inertia_pendulum
od.axis0.motor.config.torque_constant = 8.27 / 150
Kp = 10
Kd = 2
tau_lim = 1.0  # [Nm] Torque limit (D6374 is rated for 3.86 Nm)
def pd_control(err_pos, err_vel):
    tau = Kp*err_pos + Kd*err_vel
    return np.clip(tau, -tau_lim, tau_lim) # saturate output for safety

while True:
    t_cur = time.time()

    # Update controller values
    if t_cur - t_setpoint_last >= dt_setpoint_target:
        t_setpoint_last = t_cur
        q_err = pos_setpoint(t_cur - t_start) - od.axis0.encoder.pos_estimate
        qd_err = vel_setpoint(t_cur - t_start) - od.axis0.encoder.vel_estimate
        od.axis0.controller.input_torque = pd_control(q_err, qd_err)

        timestamp_setpoint.append(t_cur)
        q_setpoint.append(pos_setpoint(t_cur - t_start))
        qd_setpoint.append(vel_setpoint(t_cur - t_start))
        i_setpoint.append(pd_control(q_err, qd_err))

    # Sample measured data
    if t_cur - t_sample_last >= dt_sample_target:
        t_sample_last = t_cur
        timestamp_sample.append(t_cur)
        q.append(od.axis0.encoder.pos_estimate)
        qd.append(od.axis0.encoder.vel_estimate)
        i.append(od.axis0.motor.current_control.Iq_measured)
        
    if t_cur - t_start > t_duration:
        break

timestamp_sample = np.array(timestamp_sample)
timestamp_setpoint = np.array(timestamp_setpoint)
q = np.array(q)
q_setpoint = np.array(q_setpoint)
qd = np.array(qd)
qd_setpoint = np.array(qd_setpoint)
i = np.array(i)
i_setpoint = np.array(i_setpoint)
dt_sample = timestamp_sample[1:] - timestamp_sample[:-1]
dt_setpoint = timestamp_setpoint[1:] - timestamp_setpoint[:-1]
timestamp_sample = timestamp_sample - t_start  # Zero timestamp
timestamp_setpoint = timestamp_setpoint - t_start  # Zero timestamp

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(timestamp_sample, q, label="actual")
ax1.plot(timestamp_setpoint, q_setpoint, label="setpoint")
xmin = min(timestamp_sample[0], timestamp_setpoint[0])
xmax = max(timestamp_sample[0], timestamp_sample[-1])
ax1.xlim([timestamp_sample[0], timestamp_sample[-1]])
ax1.set_ylabel("Angular Position [turns]")
ax2.plot(timestamp_sample, i)
ax2.set_ylabel("Current [A]")
ax2.set_xlabel("Time [s]")

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

plt.show()

# Generate CSV file with header (convert unit of q from [turns] to [rad])
sampled_data = np.array([timestamp_sample, q * 2*np.pi, i])
header_txt = "timestamp actual_q_0 actual_current_0"
np.savetxt('onelink_data.csv', sampled_data, delimiter=' ', header=header_txt)

dump_errors(od,True)
