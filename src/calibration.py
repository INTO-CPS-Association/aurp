# In this script, the ODrive control board is set up by calibrating the motor and encoder.
# The script needs only to be run once unless hardware changes are made to the system, 
# because the results of the calibration is saved on the ODrive control board.

import time
import odrive
from odrive.enums import AXIS_STATE_IDLE, AXIS_STATE_ENCODER_INDEX_SEARCH, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, MOTOR_TYPE_HIGH_CURRENT

from config import configure_odrive
from odrive.utils import dump_errors

# od = configure_odrive()
od = odrive.find_any()

# # ============= Brake Resistor =================
od.config.enable_brake_resistor = True
od.config.brake_resistance = 2.2  # [A]

# # ================= Axis 0 =====================
# # SMALL (270 KV) MOTOR
# od.axis0.motor.config.torque_constant = 8.27 / 150  # [Nm/A]
# od.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
# od.axis0.controller.config.vel_limit = 2.0  # [turns/s]
# od.axis0.motor.config.current_lim = 30.0  # [A]
# od.axis0.motor.config.pole_pairs = 7  # [-]
# od.axis0.encoder.config.cpr = 8192  # [counts per revolution]
# od.axis0.motor.config.calibration_current = 15.0  # [A]
# od.axis0.encoder.config.use_index = True

# od.axis0.motor.config.resistance_calib_max_voltage = 3
# od.axis0.motor.config.calibration_current = 15
# od.axis0.motor.config.requested_current_range = 25

# ============== Full Calibration ================
print("Initiating full calibration sequence...")
od.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
if od.axis0.error != 0:
    dump_errors(od)
    print(f"Error is: {od.axis0.error}")
    od.axis0.motor.config.pre_calibrated = True
    raise RuntimeError("Unable to perform the full calibration sequence.")
else:
    print("Full calibration sequence succesfully completed.")

# ============= INDEX SEARCH ================
print("Calibrating encoder...")
od.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
while od.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
if od.axis0.error != 0:
    dump_errors(od)
    print(f"Error is: {od.axis0.error}")
    od.axis0.encoder.config.pre_calibrated = True
    raise RuntimeError("Unable to perform encoder calibration.")
else:
    print("Encoder calibration (index search) succesfully completed with values:")
    print(f"    Phase offset: {od.axis0.encoder.config.phase_offset} counts")
    print(f"    Direction:    {od.axis0.encoder.config.direction}")
    
try:
    od.save_configuration()
except odrive.fibre.libfibre.ObjectLostError:
    pass
# od.reboot()