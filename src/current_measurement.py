from time import sleep, time
import numpy as np
import matplotlib.pyplot as plt
import math

import odrive
from odrive.enums import (
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_IDLE,
    AXIS_STATE_ENCODER_INDEX_SEARCH,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_TORQUE_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)
from utils import assert_no_errors

if __name__ == "__main__":
    # CLEAR
    # try:
    #     odrv0 = odrive.find_any()
    #     odrv0.erase_configuration()
    # except odrive.fibre.libfibre.ObjectLostError:
    #     pass
    odrv0 = odrive.find_any()
    assert_no_errors(odrv0, "initial")

    # BRAKE
    odrv0.config.enable_brake_resistor = True
    odrv0.config.brake_resistance = 2.2  # [A]
    odrv0.axis0.motor.config.torque_constant = 8.27 / 150

    # configuration must be saved otherwise the error "MOTOR_ERROR_BRAKE_RESISTOR_DISARMED" is emitted
    try:
        odrv0.save_configuration()
    except odrive.fibre.libfibre.ObjectLostError:
        pass

    odrv0 = odrive.find_any()

    # CALIBRATION
    # odrv0.axis0.encoder.config.use_index = True
    # odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    # while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    #     sleep(0.1)

    # assert_no_errors(odrv0, "calibration")

    # odrv0.axis0.encoder.config.pre_calibrated = True
    odrv0.axis0.motor.config.pre_calibrated = True

    # INDEX SEARCH
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
    while odrv0.axis0.current_state != AXIS_STATE_IDLE:
        sleep(0.1)
    assert_no_errors(odrv0, "index search")

    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    odrv0.axis0.controller.config.vel_limit = 5.0

    sleep(1.0)

    assert_no_errors(odrv0, "torque control")

    t_sample = 0.01
    t_cur = 0.0
    t_end = 5.0
    t_last = t_cur
    t_offset = time()
    current_estimate = []
    t = []
    position_setpoint = []
    position_estimate = []
    velocity_setpoint = []
    velocity_estimate = []
    torque_setpoint = []

    def pos_setpoint_t(t):
        return math.sin(0.05 * t_end * t**3 + 2 * t) * 0.5

    def torque_setpoint_t(t):
        if t < 1:
            return 0.5
        else:
            return 1.0

    while t_cur < t_end:

        assert_no_errors(odrv0, f"torque control at t={t_cur}")

        if (t_cur - t_last) >= t_sample:
            odrv0.axis0.controller.input_pos = pos_setpoint_t(t_cur)
            t.append(t_cur)

            position_estimate.append(odrv0.axis0.encoder.pos_estimate)
            velocity_estimate.append(odrv0.axis0.encoder.vel_estimate)
            current_estimate.append(odrv0.axis0.motor.current_control.Iq_measured)

            position_setpoint.append(odrv0.axis0.controller.pos_setpoint)
            velocity_setpoint.append(odrv0.axis0.controller.vel_setpoint)
            torque_setpoint.append(odrv0.axis0.controller.torque_setpoint)

            t_last = t_cur

        t_cur = time() - t_offset

    # convert to np arrays
    torque_estimated = (
        np.array(current_estimate) * odrv0.axis0.motor.config.torque_constant
    )
    current_setpoint = (
        np.array(torque_setpoint) / odrv0.axis0.motor.config.torque_constant
    )
    t = np.array(t) - np.min(t)
    position_estimate = np.array(position_estimate)

    # export
    sampled_data = np.array([t, position_estimate * 2 * np.pi, torque_estimated])
    header_txt = "timestamp actual_q_0 actual_current_0"
    np.savetxt("onelink_data.csv", sampled_data, delimiter=" ", header=header_txt)

    # plotting

    fig, ax = plt.subplots()
    ax.plot(t, current_estimate, marker="*")
    ax.set_xlabel("t[s]")
    ax.set_ylabel("i(t)[A]")

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

    ax1.plot(t, position_estimate, label="estimated")
    ax1.plot(t, position_setpoint, label="setpoint")
    ax1.set_ylabel("position(t) [turns]")

    ax2.plot(t, velocity_estimate, label="estimated")
    ax2.plot(t, velocity_setpoint, label="setpoint")
    ax2.axhline(odrv0.axis0.controller.config.vel_limit, color="green")
    ax2.set_ylabel("velocity(t) [turns/s]")

    ax3.plot(t, current_estimate, label="estimated")
    ax3.plot(t, current_setpoint, label="setpoint")
    ax3.axhline(odrv0.axis0.motor.config.current_lim, color="green", label="limit")
    ax3.set_xlabel("t[s]")
    ax3.set_ylabel("current(t) [A]")
    ax3.legend()

    plt.show()
