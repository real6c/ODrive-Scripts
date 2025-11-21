import motor_functions
import live_graph_view
import time

VELOCITY = 40
ACCELERATION = 20
bin_positions = {
    0: 0,
    1: 100,
    2: 200,
}

def test_bins(odrv, axis):
    #Test cycling through bins 5 times
    for i in range(1):
        for bin_number in bin_positions:
            print(f"Going to bin {bin_number}")
            final_pos = motor_functions.trap_traj(odrv, axis, velocity=VELOCITY, acceleration=ACCELERATION, target_pos=bin_positions[bin_number], print_output=True)
            print(f"Reached bin {bin_number}, final position: {final_pos:.6f} turns")
            time.sleep(1)
        time.sleep(1)

    motor_functions.disarm_motor(odrv)

if __name__ == "__main__":
    odrv = motor_functions.connect_odrive()
    live_graph_view.start_live_plotting(odrv)
    time.sleep(5) #Wait for plot to start
    axis = motor_functions.arm_motor(odrv)
    test_bins(odrv, axis)
    #live_graph_view.stop_live_plotting()
    time.sleep(2)