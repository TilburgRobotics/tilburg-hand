"""
Simple demo to show how to control individual joints on the Tilburg Hand.
"""

import sys
import os

from time import sleep

sys.path.insert(0, "../libraries")
from tilburg_hand import TilburgHandMotorInterface, Finger, Unit


config_folder = os.path.abspath(os.path.dirname(__file__))
calibration_folder = os.path.join(os.path.expanduser("~"), 'tilburg_hand')
if len(sys.argv) != 2:
    print("Usage:\n\t", sys.argv[0], " <path-to-folder-with-config>")
    print("* Config folder should have a config.json file and a calibration.json"
          " file, as used/produced by motor_gui.")
    print("*   If no argument is supplied, the default config.json file will be "
          "used (", config_folder, "), and calibration will be saved in the "
          "user's home folder (", calibration_folder, ").")
    print("\n\n")

tilburg_hand_config_file_path = os.path.join(config_folder, "config.json")
tilburg_hand_calibration_file_path = os.path.join(calibration_folder, "calibration.json")

motors = TilburgHandMotorInterface(config_file=tilburg_hand_config_file_path,
                                   calibration_file=tilburg_hand_calibration_file_path,
                                   verbose=False)

ret = motors.connect()
if ret < 0:
    print("PROBLEMS CONNECTING TO THE MOTORS' BOARD")
    sys.exit()

motors.goto_zero_position()
sleep(0.3)

# mode = 'fast'
mode = 'slow'

t1 = 0.1 if mode == 'fast' else 0.3

motors.set_pos_single(Finger.INDEX_ABD, 1, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.MIDDLE_ABD, 1, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.RING_ABD, 1, unit=Unit.NORMALIZED)

sleep(t1)

motors.set_pos_single(Finger.RING_ABD, 0, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.MIDDLE_ABD, 0, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.INDEX_ABD, 0, unit=Unit.NORMALIZED)

sleep(t1)

motors.set_pos_single(Finger.RING_ABD, 0.5, unit=Unit.NORMALIZED)
motors.set_pos_single(Finger.MIDDLE_ABD, 0.5, unit=Unit.NORMALIZED)
motors.set_pos_single(Finger.INDEX_ABD, 0.5, unit=Unit.NORMALIZED)

sleep(1)


t1 = 0.1 if mode == 'fast' else 0.35
motors.set_pos_single(Finger.INDEX_DIP, 0.95, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.MIDDLE_DIP, 0.95, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.RING_DIP, 0.95, unit=Unit.NORMALIZED)
sleep(t1)

motors.set_pos_single(Finger.INDEX_PIP, 0.95, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.MIDDLE_PIP, 0.95, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.RING_PIP, 0.95, unit=Unit.NORMALIZED)
sleep(t1)

motors.set_pos_single(Finger.INDEX_MCP, 0.9, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.MIDDLE_MCP, 0.9, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.RING_MCP, 0.9, unit=Unit.NORMALIZED)
sleep(t1)

motors.set_pos_single(Finger.INDEX_MCP, 0, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.MIDDLE_MCP, 0, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.RING_MCP, 0, unit=Unit.NORMALIZED)
sleep(t1)

motors.set_pos_single(Finger.INDEX_PIP, 0, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.MIDDLE_PIP, 0, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.RING_PIP, 0, unit=Unit.NORMALIZED)
sleep(t1)

motors.set_pos_single(Finger.INDEX_DIP, 0, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.MIDDLE_DIP, 0, unit=Unit.NORMALIZED)
sleep(t1)
motors.set_pos_single(Finger.RING_DIP, 0, unit=Unit.NORMALIZED)
sleep(t1)


t4 = 0.1 if mode == 'fast' else 0.5
motors.set_pos_single(Finger.THUMB_CMC, 0.96, unit=Unit.NORMALIZED)
sleep(t4)
motors.set_pos_single(Finger.THUMB_CMC, 0, unit=Unit.NORMALIZED)
sleep(t4)

motors.set_pos_single(Finger.THUMB_ABD, 0, unit=Unit.NORMALIZED)
sleep(t4)
motors.set_pos_single(Finger.THUMB_ABD, 1, unit=Unit.NORMALIZED)
sleep(t4)
motors.set_pos_single(Finger.THUMB_ABD, 0.3, unit=Unit.NORMALIZED)
sleep(t4)

motors.set_pos_single(Finger.THUMB_MCP, 0.9, unit=Unit.NORMALIZED)
sleep(t4)
motors.set_pos_single(Finger.THUMB_MCP, 0, unit=Unit.NORMALIZED)
sleep(t4)

motors.set_pos_single(Finger.THUMB_IP, 0.95, unit=Unit.NORMALIZED)
sleep(t4)
motors.set_pos_single(Finger.THUMB_IP, 0, unit=Unit.NORMALIZED)
sleep(t4)


motors.goto_zero_position()
sleep(2)

motors.disconnect()
