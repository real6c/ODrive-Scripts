# ODrive EEPROM Configuration Reader
#
# Description:
# This script connects to an ODrive motor controller and reads comprehensive
# configuration parameters that are stored in its EEPROM. It prints
# general board settings as well as specific configurations for each
# motor axis (Axis 0 and Axis 1).
#
# Prerequisites:
# - ODrive Python library installed (`pip install odrive`)
# - An ODrive board connected to the computer via USB.
#
# For more information on the ODrive Python package, see the documentation:
# https://docs.odriverobotics.com/v/latest/guides/python-package.html

import odrive
from odrive.enums import * # Import enums for human-readable output
from odrive.utils import dump_errors
import time
import sys

def safe_get_attribute(obj, attr_path, default="N/A"):
    """
    Safely get an attribute from an object using dot notation.
    Returns default value if attribute doesn't exist.
    """
    try:
        attrs = attr_path.split('.')
        current = obj
        for attr in attrs:
            current = getattr(current, attr)
        return current
    except (AttributeError, TypeError):
        return default

def read_all_attributes():
    """
    Finds an ODrive, connects to it, and prints comprehensive
    configuration attributes from EEPROM.
    """
    print("=" * 60)
    print("ODrive EEPROM Configuration Reader")
    print("=" * 60)
    print("Waiting for ODrive board...")

    try:
        # Connect to the ODrive with a 10-second timeout
        odrv = odrive.find_any(timeout=10)
        print(f"✓ Successfully connected to ODrive!")
        print(f"  Serial Number: {odrv.serial_number:x}")
        print(f"  Hardware Version: {odrv.hw_version_major}.{odrv.hw_version_minor}.{odrv.hw_version_variant}")
        print()
    except Exception as e:
        print(f"✗ Error: ODrive not found. Please ensure it's connected and powered.")
        print(f"  Details: {e}")
        return False

    # Check for and print any existing errors on the board
    print("Checking for system errors...")
    dump_errors(odrv)
    print("Error check complete.\n")


    # --- VBUS Voltage Check ---
    # Check for under-voltage condition before proceeding
    MIN_VBUS_VOLTAGE = 8.0  # Default ODrive minimum voltage
    vbus_voltage = odrv.vbus_voltage
    if vbus_voltage < MIN_VBUS_VOLTAGE:
        print(f"⚠️  WARNING: VBUS voltage at {vbus_voltage:.2f}V is below recommended minimum ({MIN_VBUS_VOLTAGE}V).")
        print("   Please ensure proper power supply connection to DC terminals.")
        print("   Some parameters may not be accessible in this state.\n")
    else:
        print(f"✓ VBUS voltage: {vbus_voltage:.2f}V (Good)")

    # --- Read General Board Configuration ---
    print("\n" + "=" * 60)
    print("GENERAL BOARD CONFIGURATION")
    print("=" * 60)
    
    # Firmware and hardware info
    print(f"Firmware Version: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"Hardware Version: {odrv.hw_version_major}.{odrv.hw_version_minor}.{odrv.hw_version_variant}")
    print(f"Serial Number: {odrv.serial_number:x}")
    print(f"VBUS Voltage: {vbus_voltage:.2f} V")
    
    # DC bus configuration
    print(f"\nDC Bus Configuration:")
    print(f"  Max Positive Current: {safe_get_attribute(odrv.config, 'dc_max_positive_current', 'N/A')} A")
    print(f"  Max Negative Current: {safe_get_attribute(odrv.config, 'dc_max_negative_current', 'N/A')} A")
    print(f"  Undervoltage Trip Level: {safe_get_attribute(odrv.config, 'dc_bus_undervoltage_trip_level', 'N/A')} V")
    print(f"  Overvoltage Trip Level: {safe_get_attribute(odrv.config, 'dc_bus_overvoltage_trip_level', 'N/A')} V")
    
    # Brake resistor configuration
    print(f"\nBrake Resistor Configuration:")
    brake_enabled = safe_get_attribute(odrv.config, 'brake_resistor0.enable')
    if brake_enabled != "N/A":
        print(f"  Enabled: {brake_enabled}")
        if brake_enabled:
            resistance = safe_get_attribute(odrv.config, 'brake_resistor0.resistance')
            print(f"  Resistance: {resistance} Ohm")
    else:
        # Fallback for older firmware
        brake_enabled_old = safe_get_attribute(odrv.config, 'enable_brake_resistor')
        if brake_enabled_old != "N/A":
            print(f"  Enabled: {brake_enabled_old}")
            if brake_enabled_old:
                resistance = safe_get_attribute(odrv.config, 'brake_resistance')
                print(f"  Resistance: {resistance} Ohm")
        else:
            print("  Configuration not available (may depend on firmware version)")

    # --- Read Per-Axis Configuration ---
    print("\n" + "=" * 60)
    print("AXIS CONFIGURATION")
    print("=" * 60)
    
    # Check how many axes are available
    available_axes = [attr for attr in dir(odrv) if attr.startswith('axis')]
    print(f"Available axes: {available_axes}")
    
    for axis_name in available_axes:
        axis = getattr(odrv, axis_name)
        axis_num = axis_name.replace('axis', '')
        print(f"\n{'='*20} AXIS {axis_num} {'='*20}")
        
        # Axis Status
        print(f"Current State: {axis.current_state}")
        print(f"Is Armed: {axis.is_armed}")
        print(f"Is Homed: {axis.is_homed}")
        print(f"Active Errors: {axis.active_errors}")
        print(f"Disarm Reason: {axis.disarm_reason}")
        print(f"Procedure Result: {axis.procedure_result}")
        
        # Motor Configuration
        print(f"\n[MOTOR CONFIGURATION]")
        motor_config = axis.config.motor
        
        # Motor type with enum conversion
        try:
            motor_type = MotorType(motor_config.motor_type).name
        except (ValueError, TypeError):
            motor_type = f"Unknown ({motor_config.motor_type})"
        print(f"  Motor Type: {motor_type}")
        print(f"  Pole Pairs: {motor_config.pole_pairs}")
        print(f"  Direction: {motor_config.direction}")
        print(f"  Current Soft Max: {motor_config.current_soft_max} A")
        print(f"  Current Hard Max: {motor_config.current_hard_max} A")
        print(f"  Current Slew Rate Limit: {motor_config.current_slew_rate_limit} A/s")
        print(f"  Torque Constant: {motor_config.torque_constant} Nm/A")
        print(f"  Calibration Current: {motor_config.calibration_current} A")
        print(f"  Resistance Calib Max Voltage: {motor_config.resistance_calib_max_voltage} V")
        
        # Phase parameters with validation status
        print(f"  Phase Resistance: {motor_config.phase_resistance:.6f} Ohm")
        print(f"    Valid: {motor_config.phase_resistance_valid}")
        print(f"  Phase Inductance: {motor_config.phase_inductance:.6f} H")
        print(f"    Valid: {motor_config.phase_inductance_valid}")
        
        # Advanced motor settings
        print(f"  Current Control Bandwidth: {motor_config.current_control_bandwidth} Hz")
        print(f"  ACIM Autoflux Enable: {motor_config.acim_autoflux_enable}")
        if hasattr(motor_config, 'motor_model_l_d') and motor_config.motor_model_l_dq_valid:
            print(f"  Motor Model Ld: {motor_config.motor_model_l_d:.6f} H")
            print(f"  Motor Model Lq: {motor_config.motor_model_l_q:.6f} H")
        
        # Axis Configuration
        print(f"\n[AXIS CONFIGURATION]")
        axis_config = axis.config
        print(f"  Startup Motor Calibration: {axis_config.startup_motor_calibration}")
        print(f"  Startup Encoder Offset Calibration: {axis_config.startup_encoder_offset_calibration}")
        print(f"  Startup Encoder Index Search: {axis_config.startup_encoder_index_search}")
        print(f"  Startup Homing: {axis_config.startup_homing}")
        print(f"  Startup Closed Loop Control: {axis_config.startup_closed_loop_control}")
        print(f"  Startup Max Wait for Ready: {axis_config.startup_max_wait_for_ready} s")
        
        # Current and power limits
        print(f"  I Bus Soft Max: {axis_config.I_bus_soft_max} A")
        print(f"  I Bus Soft Min: {axis_config.I_bus_soft_min} A")
        print(f"  I Bus Hard Max: {axis_config.I_bus_hard_max} A")
        print(f"  I Bus Hard Min: {axis_config.I_bus_hard_min} A")
        print(f"  P Bus Soft Max: {axis_config.P_bus_soft_max} W")
        print(f"  P Bus Soft Min: {axis_config.P_bus_soft_min} W")
        print(f"  Torque Soft Max: {axis_config.torque_soft_max} Nm")
        print(f"  Torque Soft Min: {axis_config.torque_soft_min} Nm")
        
        # Controller Configuration
        print(f"\n[CONTROLLER CONFIGURATION]")
        controller_config = axis.controller.config
        try:
            control_mode = ControlMode(controller_config.control_mode).name
        except (ValueError, TypeError):
            control_mode = f"Unknown ({controller_config.control_mode})"
        print(f"  Control Mode: {control_mode}")
        
        try:
            input_mode = InputMode(controller_config.input_mode).name
        except (ValueError, TypeError):
            input_mode = f"Unknown ({controller_config.input_mode})"
        print(f"  Input Mode: {input_mode}")
        
        print(f"  Position Gain: {controller_config.pos_gain}")
        print(f"  Velocity Gain: {controller_config.vel_gain}")
        print(f"  Velocity Integrator Gain: {controller_config.vel_integrator_gain}")
        print(f"  Velocity Integrator Limit: {controller_config.vel_integrator_limit}")
        print(f"  Velocity Limit: {controller_config.vel_limit} counts/s")
        print(f"  Velocity Limit Tolerance: {controller_config.vel_limit_tolerance}")
        print(f"  Inertia: {controller_config.inertia}")
        print(f"  Input Filter Bandwidth: {controller_config.input_filter_bandwidth} Hz")
        print(f"  Homing Speed: {controller_config.homing_speed} counts/s")
        print(f"  Torque Ramp Rate: {controller_config.torque_ramp_rate} Nm/s")
        print(f"  Velocity Ramp Rate: {controller_config.vel_ramp_rate} counts/s²")
        
        # Encoder-related settings from axis config
        print(f"\n[ENCODER-RELATED SETTINGS]")
        print(f"  Observed Encoder Scale Factor: {axis.observed_encoder_scale_factor}")
        print(f"  Encoder Bandwidth: {axis_config.encoder_bandwidth} Hz")
        print(f"  Commutation Encoder Bandwidth: {axis_config.commutation_encoder_bandwidth} Hz")
        print(f"  Calib Range: {axis_config.calib_range}")
        print(f"  Calib Scan Distance: {axis_config.calib_scan_distance}")
        print(f"  Calib Scan Velocity: {axis_config.calib_scan_vel}")
        print(f"  Calibration Lockin: {axis_config.calibration_lockin}")
        
        # Additional axis settings
        print(f"\n[ADDITIONAL SETTINGS]")
        print(f"  Enable Watchdog: {axis_config.enable_watchdog}")
        if axis_config.enable_watchdog:
            print(f"  Watchdog Timeout: {axis_config.watchdog_timeout} s")
        print(f"  Enable Step/Dir: {axis_config.enable_step_dir}")
        print(f"  Enable Error GPIO: {axis_config.enable_error_gpio}")
        print(f"  Step GPIO Pin: {axis_config.step_gpio_pin}")
        print(f"  Dir GPIO Pin: {axis_config.dir_gpio_pin}")
        print(f"  Error GPIO Pin: {axis_config.error_gpio_pin}")
        print(f"  Enable Gain Scheduling: {controller_config.enable_gain_scheduling}")
        print(f"  Enable Overspeed Error: {controller_config.enable_overspeed_error}")
        print(f"  Enable Torque Mode Vel Limit: {controller_config.enable_torque_mode_vel_limit}")
        print(f"  Enable Velocity Limit: {controller_config.enable_vel_limit}")
        
        print("-" * 50)
    
    return True


if __name__ == "__main__":
    try:
        success = read_all_attributes()
        if success:
            print("\n" + "=" * 60)
            print("✓ EEPROM Configuration Reading Complete")
            print("=" * 60)
        else:
            print("\n" + "=" * 60)
            print("✗ EEPROM Configuration Reading Failed")
            print("=" * 60)
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nScript interrupted by user.")
        sys.exit(0)
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        sys.exit(1)






