#!/usr/bin/env python3
"""
Motor functions:
0. Connect to ODrive
1. Arm motor
2. Disarm motor
3. Trapezoidal trajectory control for accurate positioning
"""

import odrive
from odrive.enums import *
import time

def connect_odrive():
    # Connect to ODrive
    print("Connecting to ODrive...")
    odrv = odrive.find_any()
    print(f"Connected to ODrive with serial number: {odrv._dev.serial_number}")
    return odrv

def arm_motor(odrv):
    axis = odrv.axis0
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(1)
    print("\033[91mMotor ARMED\033[0m")  # Red text
    return axis

def disarm_motor(odrv):
    axis = odrv.axis0
    axis.requested_state = AxisState.IDLE
    time.sleep(0.5)
    print("\033[92mMotor DISARMED\033[0m")  # Green text
    return axis

def trap_traj(odrv, axis, velocity, acceleration, target_pos, print_output):
    # Set control mode to use trapezoidal trajectory for accurate positioning
    axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
    axis.controller.config.input_mode = InputMode.TRAP_TRAJ
    
    # Set velocity and acceleration limits for trapezoidal trajectory
    axis.controller.config.vel_limit = velocity  # turns/s
    axis.controller.config.vel_ramp_rate = acceleration  # turns/s² (acceleration/deceleration rate)
    
    # Configure trapezoidal trajectory parameters
    axis.trap_traj.config.vel_limit = velocity  # rev/s
    axis.trap_traj.config.accel_limit = acceleration  # rev/s²
    axis.trap_traj.config.decel_limit = acceleration  # rev/s²
    
    # Set target position (in turns as per docs)
    axis.controller.input_pos = target_pos  # turns
    
    print(f"Moving to position {target_pos} rev at {velocity} rev/s")
    
    # Monitor position
    while True:
        current_pos = axis.pos_estimate  # Already in turns
        current_vel = axis.vel_estimate  # Current velocity
        torque_estimate = axis.motor.torque_estimate  # Torque output
        motor_current = axis.motor.electrical_power / odrv.vbus_voltage  # Current draw (P/V)
        bus_voltage = odrv.vbus_voltage  # Bus voltage
        
        if print_output:
            print(f"POS: {current_pos:.3f} | TARGET: {target_pos:.3f} | VEL: {current_vel:.2f} | TORQUE: {torque_estimate:.3f} | I: {motor_current:.2f}A | V: {bus_voltage:.3f}V")
        
        if abs(current_pos - target_pos) < 0.1:  # Within 0.1 turns
            print("Target reached!")
            break
            
        time.sleep(0.1)
    
    # Print final position
    final_pos = axis.pos_estimate
    print(f"FINAL POSITION: {final_pos:.6f} turns")
    print(f"TARGET WAS: {target_pos:.6f} turns")
    print(f"ERROR: {abs(final_pos - target_pos):.6f} turns")
    print("\033[92mMotor DISARMED\033[0m")  # Green text

    return final_pos