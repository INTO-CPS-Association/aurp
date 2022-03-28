import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_CLOSED_LOOP_CONTROL, MOTOR_TYPE_HIGH_CURRENT, AXIS_STATE_IDLE, AXIS_STATE_ENCODER_INDEX_SEARCH, CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH
import numpy as np
import time
import math
import matplotlib.pyplot as plt
from utils import assert_no_errors
from time import sleep

# CLEAR
try:
    odrv0 = odrive.find_any()
    odrv0.erase_configuration()
except odrive.fibre.libfibre.ObjectLostError:
    pass
odrv0 = odrive.find_any()
assert_no_errors(odrv0,"initial")

# BRAKE
odrv0.config.enable_brake_resistor = True
odrv0.config.brake_resistance = 2.2  # [A]

# configuration must be saved otherwise the error "MOTOR_ERROR_BRAKE_RESISTOR_DISARMED" is emitted
try:
    odrv0.save_configuration()
except odrive.fibre.libfibre.ObjectLostError:
    pass

odrv0 = odrive.find_any()

# CALIBRATION
odrv0.axis0.encoder.config.use_index = True
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

while(odrv0.axis0.current_state != AXIS_STATE_IDLE):
    sleep(0.1)

assert_no_errors(odrv0,"calibration")

odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.motor.config.pre_calibrated = True

# INDEX SEARCH
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
while(odrv0.axis0.current_state != AXIS_STATE_IDLE):
    sleep(0.1)
assert_no_errors(odrv0,"index search")
    

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL

odrv0.axis0.controller.input_torque = 1.0

assert_no_errors(odrv0,"torque control")

odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis0.controller.config.vel_limit = 20.0  # [turns/s]
# Torque Controller
inertia_rotor = 1E-4  # inertia of the D5053 rotor according to ODrive
inertia_pendulum = 0.001 # 0.019  # inertia of rod with attached weights of 250 g + 200 g + 100 g = 550 g offset X mm
odrv0.axis0.controller.config.inertia = inertia_rotor + inertia_pendulum
K_tau = 8.27 / 150
odrv0.axis0.motor.config.torque_constant = K_tau 


t_cur = time.time()
t_start = t_cur
t_sample_last = time.time()
t_setpoint_last = time.time()
dt_sample_target = 1e-4
dt_setpoint_target = 1e-4 / 3  # 3 kHz torque control
timestamp_sample = []
timestamp_setpoint = []
q = []
q_setpoint = []
qd = []
qd_setpoint = []
tau = []
tau_setpoint = []
t_duration = 5.0

# Position and velocity trajectories
def pos_setpoint(t):
    return math.sin(0.05*t_duration*t**3 + 2*t)*0.5 + 0.5

def vel_setpoint(t):
    return math.cos(0.05*t_duration*t**3 + 2*t)*0.025*t**3  # derivative of pos_setpoint



Kp = 3*10
Kd = 2
tau_lim = 3.5  # [Nm] Torque limit (D6374 is rated for 3.86 Nm)
def pd_control(err_pos, err_vel):
    tau = Kp*err_pos + Kd*err_vel
    return np.clip(tau, -tau_lim, tau_lim) # saturate output for safety

while True:
    t_cur = time.time()

    assert_no_errors(odrv0,"torque control")

    # Update controller values
    if t_cur - t_setpoint_last >= dt_setpoint_target:
        t_setpoint_last = t_cur
        q_err = pos_setpoint(t_cur - t_start) - odrv0.axis0.encoder.pos_estimate
        qd_err = vel_setpoint(t_cur - t_start) - odrv0.axis0.encoder.vel_estimate
        tauu = pd_control(q_err, qd_err)
        odrv0.axis0.controller.input_torque = tauu

        timestamp_setpoint.append(t_cur)
        q_setpoint.append(pos_setpoint(t_cur - t_start))
        qd_setpoint.append(vel_setpoint(t_cur - t_start))
        tau_setpoint.append(tauu)

    # Sample measured data
    if t_cur - t_sample_last >= dt_sample_target:
        t_sample_last = t_cur
        timestamp_sample.append(t_cur)
        q.append(odrv0.axis0.encoder.pos_estimate)
        qd.append(odrv0.axis0.encoder.vel_estimate)
        tau.append(odrv0.axis0.motor.current_control.Iq_measured*K_tau)
        
    if t_cur - t_start > t_duration:
        break

odrv0.axis0.controller.input_torque = 0.0


timestamp_sample = np.array(timestamp_sample)
timestamp_setpoint = np.array(timestamp_setpoint)
q = np.array(q)
q_setpoint = np.array(q_setpoint)
qd = np.array(qd)
qd_setpoint = np.array(qd_setpoint)
tau = np.array(tau)
tau_setpoint = np.array(tau_setpoint)
dt_sample = timestamp_sample[1:] - timestamp_sample[:-1]
dt_setpoint = timestamp_setpoint[1:] - timestamp_setpoint[:-1]
timestamp_sample = timestamp_sample - t_start  # Zero timestamp
timestamp_setpoint = timestamp_setpoint - t_start  # Zero timestamp

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(timestamp_sample, q, label="actual")
ax1.plot(timestamp_setpoint, q_setpoint, label="setpoint")
xmin = min(timestamp_sample[0], timestamp_setpoint[0])
xmax = max(timestamp_sample[0], timestamp_sample[-1])
ax1.set_xlim([timestamp_sample[0], timestamp_sample[-1]])
ax1.set_ylabel("Angular Position [turns]")
ax2.plot(timestamp_sample, tau, label="true")
ax2.plot(timestamp_sample,tau_setpoint, label="setpoint")
ax2.set_ylabel("Torque [Nm]")
ax2.set_xlabel("Time [s]")
ax2.legend()

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
sampled_data = np.array([timestamp_sample, q * 2*np.pi, tau])
header_txt = "timestamp actual_q_0 actual_current_0"
np.savetxt('onelink_data.csv', sampled_data, delimiter=' ', header=header_txt)