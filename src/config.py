# In this script, the configuration settings of the AURP are specified.
# Make sure that the hardware connected to the ODrive control board 
# corresponds to the specifications in the script.

import odrive
from odrive.enums import MOTOR_TYPE_HIGH_CURRENT

def configure_odrive():
    od = odrive.find_any()
    try:
        od.erase_configuration()
    except odrive.fibre.libfibre.ObjectLostError:
        pass
    try:
        od = odrive.find_any()
        od.reboot()
    except odrive.fibre.libfibre.ObjectLostError:
        pass
    od = odrive.find_any()

    print("Configuring ODrive...")

    # ============= Brake Resistor =================
    od.config.enable_brake_resistor = True
    od.config.brake_resistance = 2.2  # [A]

    # ================= Axis 0 =====================
    # SMALL (270 KV) MOTOR
    # od.axis0.motor.config.torque_constant = 8.27 / 270  # [Nm/A]
    # od.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    # od.axis0.controller.config.vel_limit = 2.0  # [turns/s]
    # od.axis0.motor.config.current_lim = 30.0  # [A]
    # od.axis0.motor.config.pole_pairs = 7  # [-]
    # od.axis0.encoder.config.cpr = 8192  # [counts per revolution]
    # od.axis0.motor.config.calibration_current = 15.0  # [A]
    # od.axis0.encoder.config.use_index = True
    # od.axis0.config.startup_encoder_index_search = True

    # ================= Axis 1 =====================
    # LARGE (150 KV) MOTOR
    od.axis0.motor.config.torque_constant = 8.27 / 150  # [Nm/A]
    od.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    od.axis0.controller.config.vel_limit = 2.0  # [turns/s]
    od.axis0.motor.config.current_lim = 30.0  # [A]
    od.axis0.motor.config.pole_pairs = 7  # [-]
    od.axis0.encoder.config.cpr = 8192  # [counts per revolution]
    od.axis0.motor.config.calibration_current = 15.0  # [A]
    od.axis0.encoder.config.use_index = True

    # od.save_configuration()

    print("ODrive configured successfully.")
    return od
