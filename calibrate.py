import odrive
from odrive.enums import *

odrv = odrive.find_any()

# start motor calibration
odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# wait for completion
while odrv.axis0.current_state != AXIS_STATE_IDLE:
    pass

print("Motor calibration complete")
