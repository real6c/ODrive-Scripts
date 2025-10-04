import passthrough_motor
import odrive
from odrive.enums import *
import time

# Connect to ODrive and arm the motor
print("Connecting to ODrive...")
odrv = odrive.find_any()
axis = odrv.axis0

# Set control mode
axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
axis.controller.config.input_mode = InputMode.PASSTHROUGH

# Arm the motor
print("\033[91mArming motor...\033[0m")  # Red text
axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(1)

# Check if motor is armed, if not, try to clear errors and rearm
if not axis.is_armed:
    print("Motor not armed, clearing errors and retrying...")
    if axis.active_errors != 0:
        print(f"Active errors: {axis.active_errors}")
    if axis.disarm_reason != 0:
        print(f"Disarm reason: {axis.disarm_reason}")
    # Try to clear errors by going to idle first
    axis.requested_state = AxisState.IDLE
    time.sleep(0.5)
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(1)

if axis.is_armed:
    print("\033[91mMotor ARMED\033[0m")  # Red text
else:
    print("ERROR: Motor failed to arm!")
    exit(1)

# Main movement loop
curr_target_pos = 0.0
error_values = []

for i in range(5):
    if curr_target_pos == 0.0:
        curr_target_pos = 100.0
    else:
        curr_target_pos = 0.0
    
    # Get current position before movement
    start_pos = axis.pos_estimate
    
    # Move motor
    passthrough_motor.move_motor(velocity=10, target_pos=curr_target_pos, accel_distance=2.0, decel_percentage=0.15, min_start_velocity=5.0, min_end_velocity=1.0)
    
    # Calculate error and store it
    final_pos = axis.pos_estimate
    error = abs(final_pos - curr_target_pos)
    error_values.append(error)
    
    print(f"Movement {i+1}: Target={curr_target_pos:.1f}, Final={final_pos:.3f}, Error={error:.6f}")
    time.sleep(1)

# Print error statistics
if error_values:
    avg_error = sum(error_values) / len(error_values)
    min_error = min(error_values)
    max_error = max(error_values)
    print(f"\nERROR STATISTICS:")
    print(f"Average Error: {avg_error:.6f} turns")
    print(f"Minimum Error: {min_error:.6f} turns")
    print(f"Maximum Error: {max_error:.6f} turns")
    print(f"All Errors: {[f'{e:.6f}' for e in error_values]}")

# Disarm the motor at the end
print("\033[91mDisarming motor...\033[0m")  # Red text
axis.requested_state = AxisState.IDLE
time.sleep(0.5)
print("\033[91mMotor DISARMED\033[0m")  # Red text
