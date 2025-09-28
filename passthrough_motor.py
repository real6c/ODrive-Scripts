#!/usr/bin/env python3
"""
ODrive Motor Control - PASSTHROUGH Mode for Maximum Precision + accel/decel for better accuracy with fast movements
"""

import odrive
from odrive.enums import *
import time

def move_motor(velocity=15, target_pos=0.0, accel_distance=2.0, decel_percentage=0.15, 
               min_start_velocity=5.0, min_end_velocity=1.0):
    """
    Move ODrive motor to target position with linear acceleration/deceleration
    
    Args:
        velocity: Maximum velocity in rev/s
        target_pos: Target position in revolutions
        accel_distance: Distance to ramp up to full speed in revolutions
        decel_percentage: Percentage of total distance for deceleration (0.0-1.0)
        min_start_velocity: Minimum velocity for start in rev/s
        min_end_velocity: Minimum velocity for end in rev/s
    """
    # Connect to ODrive (assumes motor is already armed)
    odrv = odrive.find_any()
    axis = odrv.axis0
    
    # Set initial velocity limit
    axis.controller.config.vel_limit = velocity  # turns/s
    
    # Set target position (in turns as per docs)
    axis.controller.input_pos = target_pos  # turns
    
    print(f"Moving to position {target_pos} rev at {velocity} rev/s")
    
    # Monitor position with speed control
    start_pos = axis.pos_estimate  # Starting position
    total_distance = abs(target_pos - start_pos)  # Total distance to travel
    decel_distance = total_distance * decel_percentage  # Percentage of total distance for deceleration
    start_time = time.time()
    max_duration = 60  # Maximum 60 seconds to reach target
    last_pos = start_pos
    stuck_counter = 0
    
    print(f"Total distance: {total_distance:.2f} turns")
    print(f"Deceleration distance: {decel_distance:.2f} turns ({decel_percentage*100:.0f}% of total)")
    
    while True:
        current_pos = axis.pos_estimate  # Already in turns
        distance_from_start = abs(current_pos - start_pos)
        distance_to_target = abs(current_pos - target_pos)
        
        # Calculate speed based on position with linear decay
        if distance_from_start < accel_distance:
            # Linear ramp up from start
            speed_ratio = distance_from_start / accel_distance
            current_speed = min_start_velocity + (velocity - min_start_velocity) * speed_ratio
        elif distance_to_target > decel_distance:
            # Full speed in the middle
            current_speed = velocity
        else:
            # Linear speed decay when close to target
            speed_ratio = distance_to_target / decel_distance
            current_speed = min_end_velocity + (velocity - min_end_velocity) * speed_ratio
        
        # Update velocity limit
        axis.controller.config.vel_limit = current_speed
        
        current_vel = axis.vel_estimate  # Current velocity
        torque_estimate = axis.motor.torque_estimate  # Torque output
        motor_current = axis.motor.electrical_power / odrv.vbus_voltage  # Current draw (P/V)
        bus_voltage = odrv.vbus_voltage  # Bus voltage
        
        print(f"POS: {current_pos:.3f} | TARGET: {target_pos:.3f} | VEL: {current_vel:.2f} | TORQUE: {torque_estimate:.3f} | I: {motor_current:.2f}A | V: {bus_voltage:.3f}V | ERRORS: {axis.active_errors}")
        
        # Check for errors during movement
        if axis.active_errors != 0:
            print(f"ERROR DETECTED: {axis.active_errors}")
            print(f"Disarm reason: {axis.disarm_reason}")
            break
        
        # Check for timeout
        elapsed_time = time.time() - start_time
        if elapsed_time > max_duration:
            print(f"TIMEOUT: Movement took longer than {max_duration} seconds")
            break
        
        # Check if motor is stuck (not moving for 2 seconds)
        if abs(current_pos - last_pos) < 0.001:  # Less than 0.001 turns movement
            stuck_counter += 1
            if stuck_counter > 20:  # 20 * 0.1s = 2 seconds
                print("MOTOR STUCK: No movement detected for 2 seconds")
                break
        else:
            stuck_counter = 0
            last_pos = current_pos
        
        # Check if motor is no longer armed
        if not axis.is_armed:
            print("MOTOR DISARMED: Motor became disarmed during movement")
            break
            
        if distance_to_target < 0.05:  # Within 0.05 turns (more realistic tolerance)
            print("Target reached!")
            break
            
        time.sleep(0.1)
    
    # Print final position
    final_pos = axis.pos_estimate
    print(f"FINAL POSITION: {final_pos:.6f} turns")
    print(f"TARGET WAS: {target_pos:.6f} turns")
    print(f"ERROR: {abs(final_pos - target_pos):.6f} turns")

if __name__ == "__main__":
    # Example usage with default parameters
    move_motor(velocity=15, target_pos=100.0, accel_distance=2.0, decel_percentage=0.15, min_start_velocity=5.0, min_end_velocity=1.0)
    
    # Example with custom parameters:
    # move_motor(velocity=20, target_pos=100.0, accel_distance=3.0, decel_percentage=0.2)
