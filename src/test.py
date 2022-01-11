import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_CLOSED_LOOP_CONTROL, MOTOR_TYPE_HIGH_CURRENT, AXIS_STATE_IDLE, AXIS_STATE_ENCODER_INDEX_SEARCH
import time

od = odrive.find_any()

# ================= Config =====================
od.config.enable_brake_resistor = True
od.config.brake_resistance = 2.2
od.axis0.motor.config.torque_constant = 8.27 / 270
od.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
od.axis0.encoder.config.cpr = 8192


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

od.axis0.encoder.config.pre_calibrated = True
try:
    od.save_configuration()
except Exception as e:
    pass