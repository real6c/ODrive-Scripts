import motor_functions
import live_graph_view
import time

VELOCITY = 30
ACCELERATION = 20
ROTATION_DISTANCE = 100.0
NUM_BINS = 3 #0, 1, 2

def round_to_bin(pos):
    return round(pos / ROTATION_DISTANCE) * ROTATION_DISTANCE

def rotate(odrv, axis, direction): # -1 is ccw, 1 is cw
    current_pos = round_to_bin(axis.pos_estimate)
    target_pos = current_pos + direction * ROTATION_DISTANCE
    final_pos = motor_functions.trap_traj(odrv, axis, velocity=VELOCITY, acceleration=ACCELERATION, target_pos=target_pos, print_output=False)

def gotoBin(odrv, axis, currentBin, newBin):

    diff = (newBin - currentBin) % NUM_BINS
    if diff > NUM_BINS / 2:
        diff -= NUM_BINS

    if diff == 0:
        print("Already at desired bin, skipping.")
        return newBin

    rotate(odrv, axis, diff)
    
    return newBin

bin_sequence = [0, 1, 1, 2, 0, 2, 1]
last_bin = 0
odrv = motor_functions.connect_odrive()
#live_graph_view.start_live_plotting(odrv)
#time.sleep(5) #Wait for plot to start
axis = motor_functions.arm_motor(odrv)

#HOME MOTOR TO BIN 0
final_pos = motor_functions.trap_traj(odrv, axis, velocity=VELOCITY, acceleration=ACCELERATION, target_pos=0, print_output=False)
for bin in bin_sequence:
    axis = motor_functions.arm_motor(odrv)
    print(f"Last bin: {last_bin}, going to bin: {bin}")
    last_bin = gotoBin(odrv, axis, last_bin, bin)
    print("Sleeping...")
    motor_functions.disarm_motor(odrv)
    time.sleep(2)