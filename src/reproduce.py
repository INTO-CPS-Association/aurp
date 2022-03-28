import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, CONTROL_MODE_POSITION_CONTROL, AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, AXIS_STATE_ENCODER_INDEX_SEARCH, CONTROL_MODE_TORQUE_CONTROL
from time import sleep
from odrive.utils import dump_errors
from utils import assert_no_errors


def experiment_1():

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
    odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

    odrv0.axis0.controller.input_pos = 1.0

    sleep(1.0)
    
    assert_no_errors(odrv0,"position control")

def experiment_2():

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

    sleep(1.0)
    
    assert_no_errors(odrv0,"torque control")


# experiment_1()
experiment_2()