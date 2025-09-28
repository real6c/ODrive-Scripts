# ODrive Motor Control

This repository contains Python scripts for controlling ODrive motors with precise position control and smooth acceleration/deceleration profiles.

## Setup

1. Create a virtual environment:
```bash
python3 -m venv venv
```

2. Activate the virtual environment:
```bash
source venv/bin/activate
```

3. Install the ODrive package:
```bash
pip install odrive
```

## Permissions Setup

If you have issues with connectivity, you may need to edit permissions on Ubuntu machines (this was tested with a Jetson Orin Nano):

1. Create the UDEV rule:
```bash
sudo nano /etc/udev/rules.d/99-odrive.rules
```

2. Paste this line inside:
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d32", MODE="0666", GROUP="plugdev"
```
NOTE: Make sure it matches lsusb output:
```
Bus 001 Device 004: ID 1209:0d32 Generic ODrive Robotics ODrive v3
```

3. Ensure user is in plugdev group:
```bash
sudo usermod -aG plugdev $USER
```

4. Reload UDEV rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Test connectivity:
Run the following script:
```bash
python check_connection.py
```

## Common issues:
If the motor refuses to arm, run this calibration script, it may fix the issue:
```bash
python calibrate.py
```


## Usage

Run the main motor control script:
```bash
python motor_main.py
```

This will perform 5 cycles of movement between positions 0 and 100 revolutions with error tracking and statistics.
