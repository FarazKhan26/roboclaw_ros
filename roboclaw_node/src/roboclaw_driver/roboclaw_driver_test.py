#!/usr/bin/env python
from roboclaw_driver import Roboclaw
import time

dev_name = "/dev/ttyACM0"
baud_rate = 38400   # from launchfile. Driver says 115200
address = 128
roboclaw = Roboclaw(dev_name, baud_rate)

if roboclaw.Open() == 0:
    print("Couldn't open roboclaw port. Check if it available with 'ls /dev/ | grep ttyACM0'")
else:
    print("Connection to roboclaw established.")
    # verify the address for communication
    print("Fetching roboclaw version")
    version = roboclaw.ReadVersion(address)
    print("Version is " + version[1])

    # Reset motors 
    roboclaw.SpeedM1M2(address, 0, 0)
    roboclaw.ResetEncoders(address)
    # Move wheels for 1 second
    print("Moving wheels for 1 second")
    roboclaw.SpeedM1M2(address, 100, 100)
    time.sleep(1)
    roboclaw.SpeedM1M2(address, 0, 0)

    print("Done")