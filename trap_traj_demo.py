#!/usr/bin/env python3
"""
Simple ODrive Motor Control
"""

import odrive
from odrive.enums import *
import time

# Constants
VELOCITY = 25  # rev/s - much slower and safer
ACCELERATION = 20  # rev/s² - acceleration rate (start conservative)
TARGET_POS = 150  # revolutions

def main():
    # Connect to ODrive
    print("Connecting to ODrive...")
    odrv = odrive.find_any()
    axis = odrv.axis0
    
    # Set control mode to use trapezoidal trajectory for accurate positioning
    axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
    axis.controller.config.input_mode = InputMode.TRAP_TRAJ
    
    # Arm the axis
    print("\033[91mArming motor...\033[0m")  # Red text
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(1)
    print("\033[91mMotor ARMED\033[0m")  # Red text
    
    # Set velocity and acceleration limits for trapezoidal trajectory
    axis.controller.config.vel_limit = VELOCITY  # turns/s
    axis.controller.config.vel_ramp_rate = ACCELERATION  # turns/s² (acceleration/deceleration rate)
    
    # Configure trapezoidal trajectory parameters
    axis.trap_traj.config.vel_limit = VELOCITY  # rev/s
    axis.trap_traj.config.accel_limit = ACCELERATION  # rev/s²
    axis.trap_traj.config.decel_limit = ACCELERATION  # rev/s²
    
    # Set target position (in turns as per docs)
    axis.controller.input_pos = TARGET_POS  # turns
    
    print(f"Moving to position {TARGET_POS} rev at {VELOCITY} rev/s")
    
    # Monitor position
    while True:
        current_pos = axis.pos_estimate  # Already in turns
        current_vel = axis.vel_estimate  # Current velocity
        torque_estimate = axis.motor.torque_estimate  # Torque output
        motor_current = axis.motor.electrical_power / odrv.vbus_voltage  # Current draw (P/V)
        bus_voltage = odrv.vbus_voltage  # Bus voltage
        
        print(f"POS: {current_pos:.3f} | TARGET: {TARGET_POS:.3f} | VEL: {current_vel:.2f} | TORQUE: {torque_estimate:.3f} | I: {motor_current:.2f}A | V: {bus_voltage:.3f}V")
        
        if abs(current_pos - TARGET_POS) < 0.1:  # Within 0.1 turns
            print("Target reached!")
            break
            
        time.sleep(0.1)
    
    # Disarm the motor
    print("\033[92mDisarming motor...\033[0m")  # Green text
    axis.requested_state = AxisState.IDLE
    time.sleep(0.5)
    
    # Print final position
    final_pos = axis.pos_estimate
    print(f"FINAL POSITION: {final_pos:.6f} turns")
    print(f"TARGET WAS: {TARGET_POS:.6f} turns")
    print(f"ERROR: {abs(final_pos - TARGET_POS):.6f} turns")
    print("\033[92mMotor DISARMED\033[0m")  # Green text

if __name__ == "__main__":
    main()
