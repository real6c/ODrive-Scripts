"""
Check if an ODrive is connected without moving the motor.
"""

import odrive
from odrive.utils import dump_errors

print("Waiting for ODrive...")

try:
    # Find a connected ODrive (blocks until found)
    odrv0 = odrive.find_any(timeout=10)  # 10s timeout
except Exception as e:
    print("ODrive not found:", e)
    exit(1)

print(f"Found ODrive with serial number: {odrv0._dev.serial_number}")

# Print basic info without changing motor state
print(f"Firmware version: {odrv0.fw_version_major}.{odrv0.fw_version_minor}.{odrv0.fw_version_revision}")
print(f"Vbus voltage: {odrv0.vbus_voltage:.2f} V")
print(f"Axis0 state: {odrv0.axis0.current_state}")
#print(f"Axis1 state: {odrv0.axis1.current_state}")

# Show any existing errors
dump_errors(odrv0)

print("ODrive connection check complete. Motors were not moved.")

